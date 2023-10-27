#include "backend.h"
#include "log_report.h"

#include "geometry_epipolar.h"
#include "relative_rotation.h"
#include "gyro_bias_solver.h"

namespace VIO {

bool Backend::RunOnce() {
    if (data_manager_ == nullptr) {
        ReportError("[Backend] Backend cannot link with data manager.");
        return false;
    }

    // If backend is not initialized, try to initialize.
    if (!status_.is_initialized) {
        if (TryToInitialize()) {
            status_.is_initialized = true;
            ReportInfo(GREEN "[Backend] Backend succeed to initialize." RESET_COLOR);
        } else {
            ReportWarn("[Backend] Backend failed to initialize.");
            return false;
        }
    }

    return true;
}

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
        ReportDebug("[Backend] Frame at " << frame.time_stamp_s << "s has " << frame.visual_measure->features_id.size() << " features.");
        local_map_ptr->AddNewFrameWithFeatures(frame.visual_measure->features_id,
                                               frame.visual_measure->observes_per_frame,
                                               frame.time_stamp_s);
    }
    RETURN_FALSE_IF(!local_map_ptr->SelfCheck());

    return true;
}

bool Backend::EstimateGyroBiasForInitialization() {
    // Compute imu preintegration.
    for (auto &frame : data_manager_->new_frames()) {
        frame.imu_preint_block.Reset();
        frame.imu_preint_block.SetImuNoiseSigma(imu_model_->options().kAccelNoise,
                                                imu_model_->options().kGyroNoise,
                                                imu_model_->options().kAccelRandomWalk,
                                                imu_model_->options().kGyroRandomWalk);
        const int32_t max_idx = static_cast<int32_t>(frame.packed_measure->imus.size());
        for (int32_t i = 1; i < max_idx; ++i) {
            frame.imu_preint_block.Propagate(*frame.packed_measure->imus[i - 1], *frame.packed_measure->imus[i]);
        }
        frame.imu_preint_block.Information();
    }

    // Iterate all frames.
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
        ReportDebug("[Backend] Frame " << i << " and " << i + 1 << " have " << covisible_features.size() << " covisible features.");

        // Debug: Estimate rotation and translation by 5pts methods.
        // Mat3 essential = Mat3::Zero();
        // Mat3 R_cr = Mat3::Identity();
        // Vec3 t_cr = Vec3::Zero();
        // std::vector<uint8_t> status;

        // using namespace VISION_GEOMETRY;
        // EpipolarSolver solver;
        // solver.options().kMethod = EpipolarSolver::EpipolarMethod::kRansac;
        // solver.options().kModel = EpipolarSolver::EpipolarModel::kFivePoints;
        // solver.EstimateEssential(ref_norm_xy, cur_norm_xy, essential, status);
        // solver.RecoverPoseFromEssential(ref_norm_xy, cur_norm_xy, essential, R_cr, t_cr);
        // ReportDebug("R_cr is\n" << R_cr);
        // ReportDebug("t_cr is " << t_cr.transpose());

        // Debug: Estimate pure rotation by eigen solver.
        // using namespace VISION_GEOMETRY;
        // RelativeRotation solver;
        // Quat q_cr = Quat::Identity();
        // solver.EstimateRotationByBnb(ref_norm_xy, cur_norm_xy, q_cr);
        // ReportDebug("R_cr is\n" << q_cr.matrix());

        // Precompute summation terms for iteration.
        SummationTerms terms;
        RelativeRotation::ComputeSummationTerms(ref_norm_xy, cur_norm_xy, terms);

        // Estimate gyro bias by iteration.
        // Prepare for optimizaiton.
        const int n = 3;
        Vec x(n);
        x = Vec3::Zero();

        // Use LM solver in eigen3 to optimize x.
        Mat3 dr_dbg = Mat3::Identity();
        GyroBiasSolveStep functor(terms, dr_dbg);
        Eigen::NumericalDiff<GyroBiasSolveStep> num_diff(functor);
        Eigen::LevenbergMarquardt<Eigen::NumericalDiff<GyroBiasSolveStep>, float> solver(num_diff);
        solver.resetParameters();
        solver.parameters.ftol = 5e-5f;
        solver.parameters.xtol = Eigen::NumTraits<float>::epsilon();
        solver.parameters.maxfev = 100;
        solver.minimize(x);

        // Recovery rotation matrix from cayley.
        Vec3 cayley = x;
        const Mat3 R_cr = Utility::ConvertCayleyToRotationMatrix(cayley);
        q_cr = Quat(R_cr);

    }

    return true;
}

}
