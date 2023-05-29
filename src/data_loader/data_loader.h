#ifndef _VIO_STEREO_ORB_SLAM3_DATA_LOADER_H_
#define _VIO_STEREO_ORB_SLAM3_DATA_LOADER_H_

#include "datatype_basic.h"
#include "datatype_image.h"
#include "datatype_image_pyramid.h"

#include "imu_state.h"

namespace VIO {

/* Class Data Loader Declaration. */
class DataLoader final {

public:
    DataLoader() = default;
    ~DataLoader() = default;

private:
    std::list<SENSOR_MODEL::ImuMeasurement> imu_buffer_;

};

}

#endif // end of _VIO_STEREO_ORB_SLAM3_DATA_LOADER_H_
