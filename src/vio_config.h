#ifndef _VIO_STEREO_BASALT_CONFIG_H_
#define _VIO_STEREO_BASALT_CONFIG_H_

#include "datatype_basic.h"
#include "string"

namespace VIO {

struct VioOptionsOfCamera {
    float fx = 458.654f;
    float fy = 457.296f;
    float cx = 752.0f / 2.0f;
    float cy = 240.0f;
    float k1 = -0.28340811f;
    float k2 = 0.07395907f;
    float k3 = 0.0f;
    float p1 = 0.00019359f;
    float p2 = 1.76187114e-05f;
};

struct VioOptionsOfFeatureDetector {
    int32_t min_valid_feature_distance = 30;
    int32_t grid_filter_rows = 10;
    int32_t grid_filter_cols = 10;
};

struct VioOptionsOfFeatureTracker {
    int32_t half_row_size_of_patch = 6;
    int32_t half_col_size_of_patch = 6;
    uint32_t max_iterations = 15;
};

struct VioOptionsOfFrontend {
    uint32_t image_rows = 0;
    uint32_t image_cols = 0;
    bool enable_recording_curve_binlog = true;
    bool enable_recording_image_binlog = false;
    bool enable_drawing_track_result = false;
    bool select_keyframe = true;
    uint32_t max_feature_number = 100;
    uint32_t min_feature_number = 30;
    VioOptionsOfFeatureDetector feature_detector;
    VioOptionsOfFeatureTracker feature_tracker;
    std::string log_file_name = "frontend.binlog";
};

struct VioOptionsOfBackend {
    bool enable_recording_binlog = true;
    std::string log_file_name = "backend.binlog";
    // TODO:
};

struct VioOptionsOfDataLoader {
    std::string log_file_name = "data_loader.binlog";
};

/* Options for vio. */
struct VioOptions {
    std::string log_file_root_name = "../output/";
    VioOptionsOfCamera camera;
    VioOptionsOfFrontend frontend;
    VioOptionsOfBackend backend;
    VioOptionsOfDataLoader data_loader;
};

}

#endif // end of _VIO_STEREO_BASALT_CONFIG_H_
