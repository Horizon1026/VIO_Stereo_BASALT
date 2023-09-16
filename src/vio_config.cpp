#include "vio.h"

#include "pinhole.h"

#include "optical_flow_basic_klt.h"

#include "feature_point_detector.h"
#include "feature_fast.h"

#include "slam_operations.h"
#include "log_report.h"

namespace VIO {

bool Vio::ConfigAllComponents() {
    if (!ConfigComponentOfDataManager()) {
        ReportError("[Vio] Failed to config data manager.");
        return false;
    }

    if (!ConfigComponentOfDataLoader()) {
        ReportError("[Vio] Failed to config data loader.");
        return false;
    }

    if (!ConfigComponentOfFrontend()) {
        ReportError("[Vio] Failed to config visual frontend.");
        return false;
    }

    return true;
}

bool Vio::ConfigComponentOfDataManager() {
    data_manager_ = std::make_unique<DataManager>();

    ReportInfo("[Vio] Data manager initialized.");
    return true;
}

bool Vio::ConfigComponentOfDataLoader() {
    data_loader_ = std::make_unique<DataLoader>();
    data_loader_->options().kMaxToleranceTimeDifferenceOfStereoImageInSeconds = 0.005f;
    data_loader_->options().kMaxToleranceTimeDifferenceBetweenImuAndImageInSeconds = 0.001f;
    data_loader_->options().kMaxToleranceTimeDelayBetweenImuAndImageInSeconds = 1.0f;
    data_loader_->options().kEnableRecordBinaryCurveLog = options_.data_loader.enable_recording_curve_binlog;
    RETURN_FALSE_IF_FALSE(data_loader_->Initialize(options_.log_file_root_name + options_.data_loader.log_file_name));

    ReportInfo("[Vio] Data loader initialized.");
    return true;
}

bool Vio::ConfigComponentOfFrontend() {
    using CameraType = SENSOR_MODEL::Pinhole;
    using FeatureType = FEATURE_DETECTOR::FeaturePointDetector<FEATURE_DETECTOR::FastFeature>;
    using KltType = FEATURE_TRACKER::OpticalFlowBasicKlt;

    // Config visual frontend.
    frontend_ = std::make_unique<VisualFrontend>(options_.frontend.image_rows, options_.frontend.image_cols);
    frontend_->options().kEnableRecordBinaryCurveLog = options_.frontend.enable_recording_curve_binlog;
    frontend_->options().kEnableRecordBinaryImageLog = options_.frontend.enable_recording_image_binlog;
    frontend_->options().kEnableShowVisualizeResult = options_.frontend.enable_drawing_track_result;
    frontend_->options().kSelfSelectKeyframe = options_.frontend.select_keyframe;
    frontend_->options().kMaxStoredFeaturePointsNumber = options_.frontend.max_feature_number;
    frontend_->options().kMinDetectedFeaturePointsNumberInCurrentImage = options_.frontend.min_feature_number;
    RETURN_FALSE_IF_FALSE(frontend_->Initialize(options_.log_file_root_name + options_.frontend.log_file_name));

    // Config camera model.
    frontend_->camera_model() = std::make_unique<CameraType>();
    frontend_->camera_model()->SetIntrinsicParameter(
        options_.camera.fx, options_.camera.fy, options_.camera.cx, options_.camera.cy);
    frontend_->camera_model()->SetDistortionParameter(std::vector<float>{
        options_.camera.k1, options_.camera.k2, options_.camera.k3, options_.camera.p1, options_.camera.p2});

    // Config feature detector.
    frontend_->feature_detector() = std::make_unique<FeatureType>();
    frontend_->feature_detector()->options().kMinFeatureDistance = options_.frontend.feature_detector.min_valid_feature_distance;
    frontend_->feature_detector()->options().kGridFilterRowDivideNumber = options_.frontend.feature_detector.grid_filter_rows;
    frontend_->feature_detector()->options().kGridFilterColDivideNumber = options_.frontend.feature_detector.grid_filter_cols;

    // Config optical flow tracker.
    frontend_->feature_tracker() = std::make_unique<KltType>();
    frontend_->feature_tracker()->options().kMethod = FEATURE_TRACKER::OpticalFlowMethod::kFast;
    frontend_->feature_tracker()->options().kPatchRowHalfSize = options_.frontend.feature_tracker.half_row_size_of_patch;
    frontend_->feature_tracker()->options().kPatchColHalfSize = options_.frontend.feature_tracker.half_col_size_of_patch;
    frontend_->feature_tracker()->options().kMaxIteration = options_.frontend.feature_tracker.max_iterations;

    ReportInfo("[Vio] Visual frontend initialized.");
    return true;
}

}
