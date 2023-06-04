#include "data_loader.h"
#include "log_report.h"

namespace VIO {

void DataLoader::Clear() {
    imu_buffer_.clear();
    left_image_buffer_.clear();
    right_image_buffer_.clear();
}

// Push measurements into dataloader.
bool DataLoader::PushImuMeasurement(const Vec3 &accel,
                                    const Vec3 &gyro,
                                    const float &time_stamp_s) {
    if (!imu_buffer_.empty() && imu_buffer_.back()->time_stamp_s > time_stamp_s) {
        ReportWarn("[Data Loader] Imu measurement pushed has invalid timestamp. Latest in buffer is "
            << imu_buffer_.back()->time_stamp_s << " s, but pushed is " << time_stamp_s << " s.");
        return false;
    }

    auto object_ptr = imu_pool_.Get();
    object_ptr->accel = accel;
    object_ptr->gyro = gyro;
    object_ptr->time_stamp_s = time_stamp_s;
    imu_buffer_.emplace_back(std::move(object_ptr));

    return true;
}

bool DataLoader::PushImageMeasurement(uint8_t *image_ptr,
                                      const int32_t image_width,
                                      const int32_t image_height,
                                      const float &time_stamp_s,
                                      const bool is_left_image) {
    const auto image_buffer_ptr = is_left_image ? &left_image_buffer_ : &right_image_buffer_;

    if (!image_buffer_ptr->empty() && image_buffer_ptr->back()->time_stamp_s > time_stamp_s) {
        ReportWarn("[Data Loader] Camera measurement pushed has invalid timestamp. Latest in buffer is "
            << image_buffer_ptr->back()->time_stamp_s << " s, but pushed is " << time_stamp_s << " s.");
        return false;
    }

    auto object_ptr = image_pool_.Get();
    object_ptr->time_stamp_s = time_stamp_s;
    object_ptr->image = Eigen::Map<MatImg>(image_ptr, image_width, image_height);
    image_buffer_ptr->emplace_back(std::move(object_ptr));

    return true;
}

// Pop measurements from dataloader.
bool DataLoader::PopSingleMeasurement(SingleMeasurement &measure) {
    const bool imu_buffer_empty = imu_buffer_.empty();
    const bool left_buffer_empty = left_image_buffer_.empty();
    const bool right_buffer_empty = right_image_buffer_.empty();

    // If all buffers are empty, nothing can be popped.
    if (imu_buffer_empty && left_buffer_empty && right_buffer_empty) {
        return false;
    }

    const float oldest_imu_timestamp_s = imu_buffer_empty ? -1 : imu_buffer_.front()->time_stamp_s;
    const float oldest_left_timestamp_s = left_buffer_empty ? -1 : left_image_buffer_.front()->time_stamp_s;
    const float oldest_right_timestamp_s = right_buffer_empty ? -1 : right_image_buffer_.front()->time_stamp_s;

    if (imu_buffer_empty || (oldest_imu_timestamp_s > oldest_left_timestamp_s && oldest_imu_timestamp_s > oldest_right_timestamp_s)) {
        // Pop image only.
        measure.imu = nullptr;

        if ((!left_buffer_empty && !right_buffer_empty) && std::fabs(oldest_left_timestamp_s - oldest_right_timestamp_s) < options_.kMaxToleranceTimeDifferenceOfStereoImageInSeconds) {
            // Pop both left and right image if their timestamp is nearby.
            measure.left_image = std::move(left_image_buffer_.front());
            measure.right_image = std::move(right_image_buffer_.front());
            left_image_buffer_.pop_front();
            right_image_buffer_.pop_front();
        } else {
            // Pop the oldest one only.
            if (!left_buffer_empty && oldest_left_timestamp_s < oldest_right_timestamp_s) {
                measure.right_image = nullptr;
                measure.left_image = std::move(left_image_buffer_.front());
                left_image_buffer_.pop_front();
            } else {
                measure.left_image = nullptr;
                measure.right_image = std::move(right_image_buffer_.front());
                right_image_buffer_.pop_front();
            }
        }
    } else if (!imu_buffer_empty) {
        // Pop imu only.
        measure.left_image = nullptr;
        measure.right_image = nullptr;
        measure.imu = std::move(imu_buffer_.front());
        imu_buffer_.pop_front();

        return true;
    }

    measure.imu = nullptr;
    measure.left_image = nullptr;
    measure.right_image = nullptr;

    return false;
}

bool DataLoader::PopPackedMeasurement(PackedMeasurement &measure) {

    return true;
}

}
