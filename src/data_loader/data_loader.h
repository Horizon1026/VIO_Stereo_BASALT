#ifndef _VIO_STEREO_ORB_SLAM3_DATA_LOADER_H_
#define _VIO_STEREO_ORB_SLAM3_DATA_LOADER_H_

#include "datatype_basic.h"
#include "datatype_image.h"
#include "datatype_image_pyramid.h"

#include "object_pool.h"

#include "imu_measurement.h"
#include "camera_measurement.h"

#include "deque"

namespace VIO {

using namespace SLAM_UTILITY;
using namespace SENSOR_MODEL;

struct SingleMeasurement {
    ObjectPtr<ImuMeasurement> imu = nullptr;
    ObjectPtr<CameraMeasurement> left_image = nullptr;
    ObjectPtr<CameraMeasurement> right_image = nullptr;
};

struct PackedMeasurement {
    std::vector<ObjectPtr<ImuMeasurement>> imus;
    ObjectPtr<CameraMeasurement> left_image = nullptr;
    ObjectPtr<CameraMeasurement> right_image = nullptr;
};

struct DataLoaderOptions {
    float kMaxToleranceTimeDifferenceOfStereoImageInSeconds = 0.005f;
};

/* Class Data Loader Declaration. */
class DataLoader final {

public:
    DataLoader() = default;
    ~DataLoader() = default;

    void Clear();

    // Push measurements into dataloader.
    bool PushImuMeasurement(const Vec3 &accel,
                            const Vec3 &gyro,
                            const float &time_stamp_s);
    bool PushImageMeasurement(uint8_t *image_ptr,
                              const int32_t image_width,
                              const int32_t image_height,
                              const float &time_stamp_s,
                              const bool is_left_image = true);

    // Pop measurements from dataloader.
    bool PopSingleMeasurement(SingleMeasurement &measure);
    bool PopPackedMeasurement(PackedMeasurement &measure);

    // Reference for member variables.
    DataLoaderOptions &options() { return options_; }
    std::deque<ObjectPtr<ImuMeasurement>> &imu_buffer() { return imu_buffer_; }
    std::deque<ObjectPtr<CameraMeasurement>> &left_image_buffer() { return left_image_buffer_; }
    std::deque<ObjectPtr<CameraMeasurement>> &right_image_buffer() { return right_image_buffer_; }
    ObjectPool<ImuMeasurement> &imu_pool() { return imu_pool_; }
    ObjectPool<CameraMeasurement> &image_pool() { return image_pool_; }

    // Const reference for member variables.
    const DataLoaderOptions &options() const { return options_; }
    const std::deque<ObjectPtr<ImuMeasurement>> &imu_buffer() const { return imu_buffer_; }
    const std::deque<ObjectPtr<CameraMeasurement>> &left_image_buffer() const { return left_image_buffer_; }
    const std::deque<ObjectPtr<CameraMeasurement>> &right_image_buffer() const { return right_image_buffer_; }
    const ObjectPool<ImuMeasurement> &imu_pool() const { return imu_pool_; }
    const ObjectPool<CameraMeasurement> &image_pool() const { return image_pool_; }

private:
    DataLoaderOptions options_;

    std::deque<ObjectPtr<ImuMeasurement>> imu_buffer_;
    std::deque<ObjectPtr<CameraMeasurement>> left_image_buffer_;
    std::deque<ObjectPtr<CameraMeasurement>> right_image_buffer_;

    ObjectPool<ImuMeasurement> imu_pool_;
    ObjectPool<CameraMeasurement> image_pool_;

};

}

#endif // end of _VIO_STEREO_ORB_SLAM3_DATA_LOADER_H_
