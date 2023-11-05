#include "backend.h"
#include "log_report.h"
#include "relative_rotation.h"

namespace VIO {

bool Backend::EstimateGyroBiasForInitialization() {
    switch (options_.kMethodIndexToEstimateGyroBiasForInitialization) {
        case 1:
        default:
            return EstimateGyroBiasByMethodOneForInitialization();
        case 2:
            return EstimateGyroBiasByMethodTwoForInitialization();
    }
}

bool Backend::EstimateGyroBiasByMethodOneForInitialization() {
    ReportInfo("[Backend] Try to estimate bias of gyro by Method 1.");
    RecomputeImuPreintegration();

    // Iterate all frames, estimate q_wc of all frames.
    const uint32_t max_frames_idx = data_manager_->visual_local_map()->frames().back().id();
    const uint32_t min_frames_idx = data_manager_->visual_local_map()->frames().front().id();
    std::vector<FeatureType *> covisible_features;
    std::vector<Vec2> ref_norm_xy;
    std::vector<Vec2> cur_norm_xy;
    ref_norm_xy.reserve(200);
    cur_norm_xy.reserve(200);
    for (uint32_t i = min_frames_idx; i < max_frames_idx; ++i) {
        // Get covisible features only in left camera.
        data_manager_->visual_local_map()->GetCovisibleFeatures(i, i + 1, covisible_features);
        ref_norm_xy.clear();
        cur_norm_xy.clear();
        for (const auto &feature_ptr : covisible_features) {
            ref_norm_xy.emplace_back(feature_ptr->observe(i)[0].rectified_norm_xy);
            cur_norm_xy.emplace_back(feature_ptr->observe(i + 1)[0].rectified_norm_xy);
        }

        // Estimate pure rotation.
        Quat q_cr = Quat::Identity();
        using namespace VISION_GEOMETRY;
        RelativeRotation solver;
        RETURN_FALSE_IF(!solver.EstimateRotationByBnb(ref_norm_xy, cur_norm_xy, q_cr));

        // Update rotation of each frames.
        if (i == min_frames_idx) {
            data_manager_->visual_local_map()->frame(i)->q_wc().setIdentity();
        }
        // q_wc = q_wr * q_cr.inverse().
        data_manager_->visual_local_map()->frame(i + 1)->q_wc() = data_manager_->visual_local_map()->frame(i)->q_wc() * q_cr.inverse();
    }

    // Localize the left camera extrinsic.
    const Quat q_ic = data_manager_->camera_extrinsics().front().q_ic;

    // Iterate to estimate bias gyro.
    const uint32_t max_iteration = 5;
    for (uint32_t iter = 0; iter < max_iteration; ++iter) {
        Mat3 hessian = Mat3::Zero();
        Vec3 bias = Vec3::Zero();
        // Iterate all imu preintegration block to construct incremental function.
        auto new_frame_iter = std::next(data_manager_->frames_with_bias().begin());
        for (uint32_t i = min_frames_idx; i < max_frames_idx; ++i) {
            // Localize the frame with bias in 'frames_with_bias_' between frame i and i + 1.
            ImuPreintegrateBlock &imu_preint_block = new_frame_iter->imu_preint_block;
            ++new_frame_iter;
            const Mat3 dr_dbg = imu_preint_block.dr_dbg();
            const Quat q_bibj = imu_preint_block.q_ij();

            const Quat q_wi_i = data_manager_->visual_local_map()->frame(i)->q_wc() * q_ic.inverse();
            const Quat q_wi_j = data_manager_->visual_local_map()->frame(i + 1)->q_wc() * q_ic.inverse();

            const Vec3 residual = 2.0f * (q_bibj.inverse() * q_wi_i.inverse() * q_wi_j).vec();
            hessian += dr_dbg.transpose() * dr_dbg;
            bias += dr_dbg.transpose() * residual;
        }

        const Vec3 delta_bg = hessian.ldlt().solve(bias);
        RETURN_FALSE_IF(Eigen::isnan(delta_bg.array()).any());

        // Recompute imu preintegration block with new bias of gyro.
        for (auto &frame : data_manager_->frames_with_bias()) {
            frame.imu_preint_block.ResetIntegratedStates();
            frame.imu_preint_block.bias_gyro() += delta_bg;
            const int32_t max_idx = static_cast<int32_t>(frame.packed_measure->imus.size());
            for (int32_t i = 1; i < max_idx; ++i) {
                frame.imu_preint_block.Propagate(*frame.packed_measure->imus[i - 1], *frame.packed_measure->imus[i]);
            }
        }

        // Check convergence.
        BREAK_IF(delta_bg.squaredNorm() < 1e-6f);
    }

    // Report result.
    const Vec3 bias_gyro = data_manager_->frames_with_bias().back().imu_preint_block.bias_gyro();
    ReportInfo("[Backend] Estimate bias of gyro is " << LogVec(bias_gyro));

    return true;
}

bool Backend::EstimateGyroBiasByMethodTwoForInitialization() {
    ReportInfo("[Backend] Try to estimate bias of gyro by Method 2.");

    return false;
}

}
