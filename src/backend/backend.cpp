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

void Backend::RecomputeImuPreintegration() {
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
    }
}

}
