#include "vio.h"

#include "pinhole.h"

#include "optical_flow_basic_klt.h"

#include "feature_point_detector.h"
#include "feature_fast.h"

#include "slam_operations.h"
#include "log_report.h"

namespace VIO {

bool Vio::ConfigAllComponents(const VioOptions &options) {

    if (!ConfigComponentOfDataManager(options)) {
        ReportError("[Vio] Failed to config data manager.");
        return false;
    }

    if (!ConfigComponentOfDataLoader(options)) {
        ReportError("[Vio] Failed to config data loader.");
        return false;
    }

    if (!ConfigComponentOfFrontend(options)) {
        ReportError("[Vio] Failed to config visual frontend.");
        return false;
    }

    return true;
}

bool Vio::ConfigComponentOfDataManager(const VioOptions &options) {
    data_manager_ = std::make_unique<DataManager>();

    ReportInfo("[Vio] Data manager initialized.");
    return true;
}

bool Vio::ConfigComponentOfDataLoader(const VioOptions &options) {
    data_loader_ = std::make_unique<DataLoader>();

    ReportInfo("[Vio] Data loader initialized.");
    return true;
}

bool Vio::ConfigComponentOfFrontend(const VioOptions &options) {
    using CameraType = SENSOR_MODEL::Pinhole;
    using FeatureType = FEATURE_DETECTOR::FeaturePointDetector<FEATURE_DETECTOR::FastFeature>;
    using KltType = FEATURE_TRACKER::OpticalFlowBasicKlt;

    // Config visual frontend.
    frontend_ = std::make_unique<VisualFrontend>(options.frontend.image_rows, options.frontend.image_cols);
    frontend_->options().kEnableRecordBinaryLog = options.frontend.enable_recording_binlog;
    frontend_->options().kEnableVisualizeResult = options.frontend.enable_drawing_track_result;
    frontend_->options().kSelfSelectKeyframe = options.frontend.select_keyframe;
    frontend_->options().kMaxStoredFeaturePointsNumber = options.frontend.max_feature_number;
    frontend_->options().kMinDetectedFeaturePointsNumberInCurrentImage = options.frontend.min_feature_number;
    frontend_->Initialize(options.frontend.log_file_name);

    // Config camera model.
    frontend_->camera_model() = std::make_unique<CameraType>();
    frontend_->camera_model()->SetIntrinsicParameter(
        options.camera.fx, options.camera.fy, options.camera.cx, options.camera.cy);
    Vec5 distort_param;
    distort_param << options.camera.k1, options.camera.k2, options.camera.k3,
        options.camera.p1, options.camera.p2;
    frontend_->camera_model()->SetDistortionParameter(distort_param);

    // Config feature detector.
    frontend_->feature_detector() = std::make_unique<FeatureType>();
    frontend_->feature_detector()->options().kMinFeatureDistance = options.frontend.feature_detector.min_valid_feature_distance;
    frontend_->feature_detector()->options().kGridFilterRowDivideNumber = options.frontend.feature_detector.grid_filter_rows;
    frontend_->feature_detector()->options().kGridFilterColDivideNumber = options.frontend.feature_detector.grid_filter_cols;

    // Config optical flow tracker.
    frontend_->feature_tracker() = std::make_unique<KltType>();
    frontend_->feature_tracker()->options().kMethod = FEATURE_TRACKER::OpticalFlowMethod::kFast;
    frontend_->feature_tracker()->options().kPatchRowHalfSize = options.frontend.feature_tracker.half_row_size_of_patch;
    frontend_->feature_tracker()->options().kPatchColHalfSize = options.frontend.feature_tracker.half_col_size_of_patch;
    frontend_->feature_tracker()->options().kMaxIteration = options.frontend.feature_tracker.max_iterations;

    ReportInfo("[Vio] Visual frontend initialized.");
    return true;
}

}
