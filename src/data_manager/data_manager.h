#ifndef _VIO_STEREO_BASALT_DATA_MANAGER_H_
#define _VIO_STEREO_BASALT_DATA_MANAGER_H_

#include "datatype_basic.h"
#include "covisible_graph.h"
#include "imu.h"

#include "memory"
#include "deque"

namespace VIO {

using namespace SLAM_UTILITY;
using namespace SENSOR_MODEL;

/* Definition of Feature Points. */
struct FeatureParameter {
    Vec3 p_w = Vec3::Zero();
    float invdep = 1.0f;
    bool is_solved = false;
};
using FeatureObserve = std::vector<Vec2>;   // Left norm plane and right norm plane.
using FeatureType = VisualFeature<FeatureParameter, FeatureObserve>;
using CovisibleGraphType = CovisibleGraph<FeatureParameter, FeatureObserve>;

/* Definition of Frame and FrameWithBias. */
using FrameType = VisualFrame<FeatureType>;
struct FrameWithBias {
    FrameType frame;
    Vec3 bias_accel = Vec3::Zero();
    Vec3 bias_gyro = Vec3::Zero();
    ImuPreintegrateBlock imu_preint_block;
    std::vector<ImuMeasurement> raw_imu_data;
};

/* Class Data Manager Declaration. */
class DataManager final {

public:
    DataManager() = default;
    ~DataManager() = default;

    // Reference for member variables.
    CovisibleGraphType *visual_local_map() { return visual_local_map_.get(); }
    std::deque<FrameWithBias> &new_frames() { return new_frames_; }

private:
    // All keyframes and map points.
    // Keyframes : [ p_wc, q_wc ]
    // Feature Points : [ p_w | invdep ]
    std::unique_ptr<CovisibleGraphType> visual_local_map_ = std::make_unique<CovisibleGraphType>();
    // All new frames with bias.
    // Frames with bias : [ p_wc, q_wc, v_wc, bias_a, bias_g ]
    std::deque<FrameWithBias> new_frames_;

};

}

#endif // end of _VIO_STEREO_BASALT_DATA_MANAGER_H_
