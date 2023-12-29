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

struct VioOptionsOfImu {
    float noise_accel = 1e-2f;
    float noise_gyro = 1e-2f;
    float random_walk_accel = 1e-4f;
    float random_walk_gyro = 1e-4f;
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
    bool enable_drawing_track_result = true;
    bool select_keyframe = false;
    uint32_t max_feature_number = 100;
    uint32_t min_feature_number = 40;
    VioOptionsOfFeatureDetector feature_detector;
    VioOptionsOfFeatureTracker feature_tracker;
    bool enable_recording_curve_binlog = true;
    bool enable_recording_image_binlog = false;
    std::string log_file_name = "frontend.binlog";
};

struct VioOptionsOfBackend {
    /* Method index explaination: */
    // Method 1: Method in Vins-Mono.
    // Method 2: Robust vio initialization - Heyijia.
    // Method 3: Visual rotation directly estimate gyro bias.
    uint32_t method_index_to_estimate_gyro_bias_for_initialization = 3;

    Vec3 gravity_w = Vec3(0.0f, 0.0f, 9.8f);
    float max_valid_feature_depth_in_meter = 50.0f;
    float min_valid_feature_depth_in_meter = 0.1f;

    bool enable_recording_curve_binlog = true;
    std::string log_file_name = "backend.binlog";
};

struct VioOptionsOfDataLoader {
    uint32_t max_size_of_imu_buffer = 200;
    uint32_t max_size_of_image_buffer = 20;
    bool enable_recording_curve_binlog = true;
    std::string log_file_name = "data_loader.binlog";
    bool enable_recording_raw_data_binlog = true;
};

struct VioOptionsOfDataManager {
    uint32_t max_num_of_stored_keyframes = 6;
    uint32_t max_num_of_stored_new_frames = 3;
    bool enable_recording_curve_binlog = true;
    std::string log_file_name = "data_manager.binlog";
    std::vector<Mat3> all_R_ic = {};
    std::vector<Vec3> all_t_ic = {};
};

/* Options for vio. */
struct VioOptions {
    std::string log_file_root_name = "../output/";
    float max_tolerence_time_s_for_no_data = 2.0f;
    float heart_beat_period_time_s = 1.0f;
    VioOptionsOfCamera camera;
    VioOptionsOfImu imu;
    VioOptionsOfFrontend frontend;
    VioOptionsOfBackend backend;
    VioOptionsOfDataLoader data_loader;
    VioOptionsOfDataManager data_manager;
};

}

#endif // end of _VIO_STEREO_BASALT_CONFIG_H_
