#ifndef _VIO_STEREO_ORB_SLAM3_DATA_MANAGER_H_
#define _VIO_STEREO_ORB_SLAM3_DATA_MANAGER_H_

#include "datatype_basic.h"
#include "covisible_graph.h"

#include "memory"

namespace VIO {

/* Class Data Manager Declaration. */
class DataManager final {

public:
    using CovisibleGraphType = SLAM_UTILITY::CovisibleGraph<Vec3, Vec2>;

public:
    DataManager() = default;
    ~DataManager() = default;

    // Reference for member varibles.
    CovisibleGraphType *local_map() { return local_map_.get(); }

private:
    std::unique_ptr<CovisibleGraphType> local_map_ = std::make_unique<CovisibleGraphType>();

};

}

#endif // end of _VIO_STEREO_ORB_SLAM3_DATA_MANAGER_H_
