#include "backend.h"
#include "log_report.h"

namespace VIO {

void Backend::RecomputeImuPreintegration() {
    // Compute imu preintegration.
    for (auto &frame : data_manager_->frames_with_bias()) {
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
