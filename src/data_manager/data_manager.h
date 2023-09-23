#ifndef _VIO_STEREO_BASALT_DATA_MANAGER_H_
#define _VIO_STEREO_BASALT_DATA_MANAGER_H_

#include "datatype_basic.h"
#include "covisible_graph.h"

#include "memory"
#include "deque"

namespace VIO {

using namespace SLAM_UTILITY;
using FeatureParameter = Vec3;  // p_w or p_c?
using FeatureObserve = std::vector<Vec2>;   // Left norm plane and right norm plane.
using FeatureType = VisualFeature<FeatureParameter, FeatureObserve>;
using CovisibleGraphType = CovisibleGraph<FeatureParameter, FeatureObserve>;

/* Class Data Manager Declaration. */
class DataManager final {

public:
    DataManager() = default;
    ~DataManager() = default;

    // Reference for member variables.
    CovisibleGraphType *visual_local_map() { return visual_local_map_.get(); }

private:
    // All keyframes and map points.
    std::unique_ptr<CovisibleGraphType> visual_local_map_ = std::make_unique<CovisibleGraphType>();

    // All new frames.
    std::deque<VisualFrame<FeatureType>> new_frames_;

};

}

#endif // end of _VIO_STEREO_BASALT_DATA_MANAGER_H_
