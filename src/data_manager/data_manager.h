#ifndef _VIO_STEREO_BASALT_DATA_MANAGER_H_
#define _VIO_STEREO_BASALT_DATA_MANAGER_H_

#include "datatype_basic.h"
#include "covisible_graph.h"

#include "memory"

namespace VIO {

/* Class Data Manager Declaration. */
class DataManager final {

public:
    using FeatureParameter = Vec3;  // p_w or p_c?
    using FeatureObserve = std::vector<Vec2>;   // Left norm plane and right norm plane.
    using CovisibleGraphType = SLAM_UTILITY::CovisibleGraph<FeatureParameter, FeatureObserve>;

public:
    DataManager() = default;
    ~DataManager() = default;

    // Reference for member varibles.
    CovisibleGraphType *visual_local_map() { return visual_local_map_.get(); }

private:
    std::unique_ptr<CovisibleGraphType> visual_local_map_ = std::make_unique<CovisibleGraphType>();

};

}

#endif // end of _VIO_STEREO_BASALT_DATA_MANAGER_H_
