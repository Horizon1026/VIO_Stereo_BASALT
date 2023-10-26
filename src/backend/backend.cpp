#include "backend.h"
#include "log_report.h"

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
    for (uint32_t i = min_frames_idx; i < max_frames_idx; ++i) {
        // Get covisible features.
        data_manager_->visual_local_map()->GetCovisibleFeatures(i, i + 1, covisible_features);
        ReportDebug("[Backend] Frame " << i << " and " << i + 1 << " have " << covisible_features.size() << " covisible features.");
        // Precompute summation terms for iteration.

        // Estimate gyro bias by iteration.

    }

    return true;
}

}
