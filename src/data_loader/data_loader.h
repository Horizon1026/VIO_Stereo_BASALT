#ifndef _VIO_STEREO_ORB_SLAM3_DATA_LOADER_H_
#define _VIO_STEREO_ORB_SLAM3_DATA_LOADER_H_

#include "datatype_basic.h"
#include "datatype_image.h"
#include "datatype_image_pyramid.h"

#include "object_pool.h"

#include "imu_state.h"
#include "camera_basic.h"

#include "deque"

namespace VIO {

using namespace SLAM_UTILITY;
using namespace SENSOR_MODEL;

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

private:
    std::deque<ObjectPtr<ImuMeasurement>> imu_buffer_;
    std::deque<ObjectPtr<CameraMeasurement>> left_image_buffer_;
    std::deque<ObjectPtr<CameraMeasurement>> right_image_buffer_;

    ObjectPool<ImuMeasurement> imu_pool_;
    ObjectPool<CameraMeasurement> image_pool_;

};

}

#endif // end of _VIO_STEREO_ORB_SLAM3_DATA_LOADER_H_
