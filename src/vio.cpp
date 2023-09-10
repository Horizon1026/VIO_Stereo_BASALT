#include "vio.h"

#include "slam_operations.h"
#include "log_report.h"

namespace VIO {

bool Vio::RunOnce() {
    // Try to load packed measurements.
    PackedMeasurement measure;
    const bool res = data_loader_->PopPackedMeasurement(measure);
    if (!res) {
        ReportInfo("[Vio] Failed to load packed measures. Skip this tick.");
        return false;
    }

    // Check integrity of the packed measurements.
    if (measure.imus.empty() || measure.left_image == nullptr || measure.right_image == nullptr) {
        ReportWarn("[Vio] Packed measurements is not valid.");
        return false;
    }

    // Process image measurements.
    frontend_->RunOnce(measure.left_image->image, measure.right_image->image);

    return true;
}

}
