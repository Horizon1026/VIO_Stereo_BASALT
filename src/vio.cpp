#include "vio.h"

#include "slam_operations.h"
#include "log_report.h"

namespace VIO {

bool Vio::RunOnce() {
    // Try to load packed measurements.
    PackedMeasurement measure;
    const bool res = data_loader_->PopPackedMeasurement(measure);
    if (!res) {
        ReportWarn("[Vio] Load measure failed : imu " << measure.imus.size() << ", left " << LogPtr(measure.left_image.get()) <<
            ", right " << LogPtr(measure.right_image.get()));
        return false;
    }

    // Check integrity of the packed measurements.
    if (measure.imus.empty() || measure.left_image == nullptr || measure.right_image == nullptr) {
        ReportWarn("[Vio] Packed measurements is not valid.");
        return false;
    }

    // Process image measurements.
    ReportInfo("[Vio] Frontend run once.");
    frontend_->RunOnce(measure.left_image->image, measure.right_image->image);

    return true;
}

}
