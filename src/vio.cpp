#include "vio.h"

#include "slam_operations.h"
#include "log_report.h"

namespace VIO {

bool Vio::RunOnce() {
    // Try to load packed measurements.
    std::unique_ptr<PackedMeasurement> measure = std::make_unique<PackedMeasurement>();
    const bool res = data_loader_->PopPackedMeasurement(*measure);
    if (!res) {
        const float time_s_for_no_data = measure_invalid_timer_.TockInSecond();
        if (time_s_for_no_data > options_.max_tolerence_time_s_for_no_data) {
            ReportInfo("[Vio] Failed to load packed measures for " << time_s_for_no_data << " s. Skip this tick.");
        }
        return false;
    }
    measure_invalid_timer_.TockTickInSecond();

    // Check integrity of the packed measurements.
    if (measure->imus.empty() || measure->left_image == nullptr || measure->right_image == nullptr) {
        ReportWarn("[Vio] Packed measurements is not valid at " << vio_sys_timer_.TockInSecond() << " s.");
        return false;
    }

    // Preprocess image measurements.
    if (!frontend_->RunOnce(GrayImage(measure->left_image->image), GrayImage(measure->right_image->image))) {
        ReportWarn("[Vio] Visual frontend failed to run once at " << vio_sys_timer_.TockInSecond() << " s.");
        return false;
    }

    // Process image and imu measurements.
    if (!data_manager_->ProcessMeasure(measure, frontend_->output_data())) {
        ReportWarn("[Vio] Data manager failed to process image and imu measurements at " << vio_sys_timer_.TockInSecond() << " s.");
        return false;
    }

    // Heart beat.
    if (vio_heart_beat_timer_.TockInSecond() > options_.heart_beat_period_time_s) {
        ReportInfo("[Vio] Heart beat for " << vio_heart_beat_timer_.TockTickInSecond() << " s. Vio has running for " <<
            vio_sys_timer_.TockInSecond() << " s.");
    }

    return true;
}

}
