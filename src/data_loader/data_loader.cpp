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
    if (imu_buffer_.back()->time_stamp_s > time_stamp_s) {
        ReportWarn("[Data Loader] Imu measurement pushed has invalid timestamp. Latest in buffer is "
            << imu_buffer_.back()->time_stamp_s << " s, but pushed is " << time_stamp_s << " s.");
        return false;
    }

    imu_buffer_.emplace_back(imu_pool_.Get());
    imu_buffer_.back()->accel = accel;
    imu_buffer_.back()->gyro = gyro;
    imu_buffer_.back()->time_stamp_s = time_stamp_s;

    return true;
}

bool DataLoader::PushImageMeasurement(uint8_t *image_ptr,
                                      const int32_t image_width,
                                      const int32_t image_height,
                                      const float &time_stamp_s,
                                      const bool is_left_image) {
    const auto image_buffer_ptr = is_left_image ? &left_image_buffer_ : &right_image_buffer_;

    if (image_buffer_ptr->back()->time_stamp_s > time_stamp_s) {
        ReportWarn("[Data Loader] Camera measurement pushed has invalid timestamp. Latest in buffer is "
            << image_buffer_ptr->back()->time_stamp_s << " s, but pushed is " << time_stamp_s << " s.");
        return false;
    }

    image_buffer_ptr->emplace_back(image_pool_.Get());
    image_buffer_ptr->back()->time_stamp_s = time_stamp_s;
    image_buffer_ptr->back()->image = Eigen::Map<Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(image_ptr, image_width, image_height);

    return true;
}

}
