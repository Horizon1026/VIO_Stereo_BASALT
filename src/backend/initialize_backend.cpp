#include "backend.h"
#include "log_report.h"
#include "gyro_bias_solver.h"
#include "relative_rotation.h"

namespace VIO {

bool Backend::TryToInitialize() {
    if (data_manager_->new_frames().size() < data_manager_->options().kMaxStoredNewFrames) {
        ReportWarn("[Backend] Backend cannot initialize for lack of new frames.");
        return false;
    }

    // Convert the new frames into a covisible graph.
    if (!ConvertNewFramesToCovisibleGraphForInitialization()) {
        ReportError("[Backend] Backend failed to convert new frames to covisible graph.");
        return false;
    }

    // Estiamte gyro bias.
    if (!EstimateGyroBiasForInitialization()) {
        ReportError("[Backend] Backend failed to estimate gyro bias.");
        return false;
    }

    return true;
}

bool Backend::ConvertNewFramesToCovisibleGraphForInitialization() {
    RETURN_FALSE_IF(data_manager_->visual_local_map() == nullptr);

    auto local_map_ptr = data_manager_->visual_local_map();
    for (const auto &frame : data_manager_->new_frames()) {
        RETURN_FALSE_IF(frame.visual_measure == nullptr);
        local_map_ptr->AddNewFrameWithFeatures(frame.visual_measure->features_id,
                                               frame.visual_measure->observes_per_frame,
                                               frame.time_stamp_s);
    }
    RETURN_FALSE_IF(!local_map_ptr->SelfCheck());

    return true;
}

bool Backend::EstimateGyroBiasForInitialization() {
    return EstimateGyroBiasByMethodTwoForInitialization();
}

bool Backend::EstimateGyroBiasByMethodOneForInitialization() {
    RecomputeImuPreintegration();

    // Iterate all frames.
    const uint32_t max_frames_idx = data_manager_->visual_local_map()->frames().back().id();
    const uint32_t min_frames_idx = data_manager_->visual_local_map()->frames().front().id();
    auto new_frame_iter = std::next(data_manager_->new_frames().begin());
    std::vector<FeatureType *> covisible_features;
    std::vector<Vec2> ref_norm_xy;
    std::vector<Vec2> cur_norm_xy;
    ref_norm_xy.reserve(200);
    cur_norm_xy.reserve(200);
    for (uint32_t i = min_frames_idx; i < max_frames_idx; ++i) {
        // Localize the frame with bias in 'new_frames_' between frame i and i + 1.
        ImuPreintegrateBlock &imu_preint_block = new_frame_iter->imu_preint_block;
        ++new_frame_iter;
        const Mat3 dr_dbg = imu_preint_block.dr_dbg();
        const Quat q_ij = imu_preint_block.q_ij();

        // Localize the left camera extrinsic.
        const Quat q_ic = data_manager_->camera_extrinsics().front().q_ic;

        // Get covisible features.
        data_manager_->visual_local_map()->GetCovisibleFeatures(i, i + 1, covisible_features);
        ref_norm_xy.clear();
        cur_norm_xy.clear();
        for (const auto &feature_ptr : covisible_features) {
            ref_norm_xy.emplace_back(feature_ptr->observe(i)[0].rectified_norm_xy);
            cur_norm_xy.emplace_back(feature_ptr->observe(i + 1)[0].rectified_norm_xy);
        }

        // Precompute summation terms for iteration.
        SummationTerms terms;
        for (uint32_t i = 0; i < ref_norm_xy.size(); ++i) {
            const Vec3 f_i = q_ij.inverse() * q_ic * Vec3(ref_norm_xy[i].x(), ref_norm_xy[i].y(), 1.0f);
            const Vec3 f_j = q_ic * Vec3(cur_norm_xy[i].x(), cur_norm_xy[i].y(), 1.0f);
            const Mat3 F = f_i * f_i.transpose();
            const float weight = 1.0f;

            terms.xx += weight * f_j.x() * f_j.x() * F;
            terms.yy += weight * f_j.y() * f_j.y() * F;
            terms.zz += weight * f_j.z() * f_j.z() * F;
            terms.xy += weight * f_j.x() * f_j.y() * F;
            terms.yz += weight * f_j.y() * f_j.z() * F;
            terms.zx += weight * f_j.z() * f_j.x() * F;
        }

        // Estimate gyro bias by iteration.
        // Prepare for optimizaiton.
        const int n = 3;
        Vec x(n);
        x = Vec3::Zero();

        // Use LM solver in eigen3 to optimize bias of gyro.
        GyroBiasSolveStep functor(terms, dr_dbg);
        Eigen::NumericalDiff<GyroBiasSolveStep> num_diff(functor);
        Eigen::LevenbergMarquardt<Eigen::NumericalDiff<GyroBiasSolveStep>, float> solver(num_diff);
        solver.resetParameters();
        solver.parameters.ftol = 1e-5f;
        solver.parameters.xtol = Eigen::NumTraits<float>::epsilon();
        solver.parameters.maxfev = 100;
        solver.minimize(x);

        // Reset imu preintegration with new bias of gyro.
        const Vec3 bias_g = x;
        imu_preint_block.ResetIntegratedStates();
        imu_preint_block.bias_gyro() = bias_g;
    }

    // Recompute imu preintegration with new bias of gyro.
    for (auto &frame : data_manager_->new_frames()) {
        const int32_t max_idx = static_cast<int32_t>(frame.packed_measure->imus.size());
        for (int32_t i = 1; i < max_idx; ++i) {
            frame.imu_preint_block.Propagate(*frame.packed_measure->imus[i - 1], *frame.packed_measure->imus[i]);
            ReportDebug("imu at " << frame.packed_measure->imus[i - 1]->time_stamp_s << "s, accel " <<
                LogVec(frame.packed_measure->imus[i - 1]->accel) << ", gyro " << LogVec(frame.packed_measure->imus[i - 1]->gyro));
        }
        frame.imu_preint_block.SimpleInformation();
    }

    return true;
}

bool Backend::EstimateGyroBiasByMethodTwoForInitialization() {
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
        // Get covisible features.
        data_manager_->visual_local_map()->GetCovisibleFeatures(i, i + 1, covisible_features);
        ref_norm_xy.clear();
        cur_norm_xy.clear();
        for (const auto &feature_ptr : covisible_features) {
            ref_norm_xy.emplace_back(feature_ptr->observe(i)[0].rectified_norm_xy);
            cur_norm_xy.emplace_back(feature_ptr->observe(i + 1)[0].rectified_norm_xy);
        }
        ReportDebug("ref_norm_xy and cur_norm_xy size is " << ref_norm_xy.size() << ", " << cur_norm_xy.size());

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

    // Print rotation of each frame for debug.
    for (uint32_t i = min_frames_idx; i <= max_frames_idx; ++i) {
        ReportDebug("frame " << i << " q_wc " << LogQuat(data_manager_->visual_local_map()->frame(i)->q_wc()));
    }

    // Localize the left camera extrinsic.
    const Quat q_ic = data_manager_->camera_extrinsics().front().q_ic;
    ReportDebug("q_ic is " << LogQuat(q_ic));

    // Iterate to estimate bias gyro.
    const uint32_t max_iteration = 5;
    for (uint32_t iter = 0; iter < max_iteration; ++iter) {
        Mat3 hessian = Mat3::Zero();
        Vec3 bias = Vec3::Zero();
        // Iterate all imu preintegration block to construct incremental function.
        auto new_frame_iter = std::next(data_manager_->new_frames().begin());
        for (uint32_t i = min_frames_idx; i < max_frames_idx; ++i) {
            // Localize the frame with bias in 'new_frames_' between frame i and i + 1.
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
        ReportDebug("delta bg is " << LogVec(delta_bg));

        // Recompute imu preintegration block with new bias of gyro.
        for (auto &frame : data_manager_->new_frames()) {
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

    // Print simple information for debug.
    for (auto &frame : data_manager_->new_frames()) {
        frame.imu_preint_block.SimpleInformation();
    }

    return true;
}

}