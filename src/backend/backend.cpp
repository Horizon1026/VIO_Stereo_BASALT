#include "backend.h"
#include "log_report.h"

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
            ResetToReintialize();
            ReportWarn("[Backend] Backend failed to initialize.");
            return true;
        }
    }

    return true;
}

void Backend::Reset() {
    // Clear stored states in data_manager.
    data_manager_->new_frames().clear();
    data_manager_->visual_local_map()->Clear();

    // Reset status.
    status_.is_initialized = false;
}

void Backend::ResetToReintialize() {
    // Clear stored states in data_manager.
    while (data_manager_->new_frames().size() >= data_manager_->options().kMaxStoredNewFrames) {
        data_manager_->new_frames().pop_front();
    }
    data_manager_->visual_local_map()->Clear();

    // Reset status.
    status_.is_initialized = false;
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
            ReportDebug("[Backend] Imu measure at " << frame.packed_measure->imus[i - 1]->time_stamp_s << " : accel " <<
                LogVec(frame.packed_measure->imus[i - 1]->accel) << ", gyro " << LogVec(frame.packed_measure->imus[i - 1]->gyro));
        }
        frame.imu_preint_block.Information();
    }

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
        // Get covisible features.
        data_manager_->visual_local_map()->GetCovisibleFeatures(i, i + 1, covisible_features);
        ref_norm_xy.clear();
        cur_norm_xy.clear();
        for (const auto &feature_ptr : covisible_features) {
            ref_norm_xy.emplace_back(feature_ptr->observe(i)[0].rectified_norm_xy);
            cur_norm_xy.emplace_back(feature_ptr->observe(i + 1)[0].rectified_norm_xy);
        }
        ReportDebug("[Backend] Frame " << i << " and " << i + 1 << " have " << covisible_features.size() << " covisible features.");

        // Precompute summation terms for iteration.
        SummationTerms terms;
        RelativeRotation::ComputeSummationTerms(ref_norm_xy, cur_norm_xy, terms);

        // Localize the frame with bias in 'new_frames_' between frame i and i + 1.
        ImuPreintegrateBlock &imu_preint_block = new_frame_iter->imu_preint_block;
        ++new_frame_iter;
        const Mat3 dr_dbg = imu_preint_block.dr_dbg();
        ReportDebug("[Backend] Imu preintegrate dr_dbg is\n" << dr_dbg);

        // Estimate gyro bias by iteration.
        // Prepare for optimizaiton.
        const int n = 3;
        Vec x(n);
        x = Vec3::Zero();

        // Use LM solver in eigen3 to optimize x.
        GyroBiasSolveStep functor(terms, dr_dbg);
        Eigen::NumericalDiff<GyroBiasSolveStep> num_diff(functor);
        Eigen::LevenbergMarquardt<Eigen::NumericalDiff<GyroBiasSolveStep>, float> solver(num_diff);
        solver.resetParameters();
        solver.parameters.ftol = 5e-5f;
        solver.parameters.xtol = Eigen::NumTraits<float>::epsilon();
        solver.parameters.maxfev = 100;
        solver.minimize(x);

        // Recovery rotation matrix from cayley.
        const Vec3 bias_g = x;
        imu_preint_block.bias_gyro() = bias_g;
        ReportDebug("[Backend] Estimated bias_g is " << LogVec(bias_g));

    }

    return true;
}

}
