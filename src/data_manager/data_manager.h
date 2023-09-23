#ifndef _VIO_STEREO_BASALT_DATA_MANAGER_H_
#define _VIO_STEREO_BASALT_DATA_MANAGER_H_

#include "datatype_basic.h"
#include "covisible_graph.h"
#include "imu.h"

#include "data_loader.h"
#include "visual_frontend.h"

#include "memory"
#include "deque"

namespace VIO {

using namespace SLAM_UTILITY;
using namespace SENSOR_MODEL;

/* Options for Data Manager. */
struct DataManagerOptions {
    uint32_t kMaxStoredKeyframes = 5;
    uint32_t kMaxStoredNewFrames = 3;
    bool kEnableRecordBinaryCurveLog = false;
};

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
    // Camera position, velocity, attitude combined with this frame.
    Quat q_wc = Quat::Identity();
    Vec3 p_wc = Vec3::Zero();
    Vec3 v_wc = Vec3::Zero();
    // Imu bias of accel and gyro is inside imu_preint_block.
    ImuPreintegrateBlock imu_preint_block;
    float time_stamp_s_ = 0.0f;
    // Measurement of raw imu(gyro, acc), raw image(left, right) and visual features.
    std::unique_ptr<PackedMeasurement> packed_measure = nullptr;
    std::unique_ptr<FrontendOutputData> visual_measure = nullptr;
};

/* Class Data Manager Declaration. */
class DataManager final {

public:
    DataManager() = default;
    ~DataManager() = default;

    // Transform packed measurements to a new frame.
    bool ProcessMeasure(std::unique_ptr<PackedMeasurement> &new_packed_measure,
                        std::unique_ptr<FrontendOutputData> &new_visual_measure);

    // Reference for member variables.
    DataManagerOptions &options() { return options_; }
    CovisibleGraphType *visual_local_map() { return visual_local_map_.get(); }
    std::deque<FrameWithBias> &new_frames() { return new_frames_; }

private:
    // Options for data manager.
    DataManagerOptions options_;
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
