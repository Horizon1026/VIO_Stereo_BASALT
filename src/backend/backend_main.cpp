#include "backend.h"
#include "log_report.h"
#include "tick_tock.h"

namespace VIO {

bool Backend::Configuration(const std::string &log_file_name) {
    if (options_.kEnableRecordBinaryCurveLog) {
        if (!logger_.CreateLogFile(log_file_name)) {
            ReportError("[Backend] Failed to create log file.");
            options_.kEnableRecordBinaryCurveLog = false;
            return false;
        }

        RegisterLogPackages();
        logger_.PrepareForRecording();
    }

    return true;
}

bool Backend::RunOnce() {
    ReportInfo(MAGENTA "[Backend] Backend is triggerred to run once." RESET_COLOR);
    if (data_manager_ == nullptr) {
        ReportError("[Backend] Backend cannot link with data manager.");
        return false;
    }

    // Add newest frame_with_bias into visual_local_map.
    if (states_.is_initialized) {
        if (!AddNewestFrameWithBiasIntoLocalMap()) {
            ReportError("[Backend] Backend failed to add newest frame_with_bias into local map.");
            return false;
        }
        if (!TriangulizeAllVisualFeatures()) {
            ReportError("[Backend] Backend failed to triangulize features in local map.");
            return false;
        }
    }

    // If backend is not initialized, try to initialize.
    if (!states_.is_initialized) {
        TickTock timer;
        if (TryToInitialize()) {
            ShowTinyInformationOfVisualLocalMap();
            states_.is_initialized = true;
            ReportInfo(GREEN "[Backend] Backend succeed to initialize within " << timer.TockTickInMillisecond() << " ms." RESET_COLOR);
        } else {
            ResetToReintialize();
            ReportWarn("[Backend] Backend failed to initialize. All states will be reset for reinitialization.");
        }
    }

    // If backend is initialized.
    if (states_.is_initialized) {
        // Try to estimate states.
        TickTock timer;
        if (!TryToEstimate()) {
            ResetToReintialize();
            ReportWarn("[Backend] Backend failed to estimate.");
            return true;
        } else {
            ReportInfo(GREEN "[Backend] Backend succeed to estimate states within " << timer.TockTickInMillisecond() << " ms." RESET_COLOR);
        }

        if (options_.kEnableReportAllInformation) {
            // Show information of visual local map if neccessary.
            ShowTinyInformationOfVisualLocalMap();
            // Show all frames and features in local map.
            ShowLocalMapFramesAndFeatures();
            // Show all frames with bias.
            // ShowAllFramesWithBias();
        }

        // Decide marginalization type.
        if (data_manager_->visual_local_map()->frames().size() >= data_manager_->options().kMaxStoredKeyFrames) {
            if (data_manager_->frames_with_bias().front().visual_measure->is_current_keyframe) {
                states_.marginalize_type = BackendMarginalizeType::kMarginalizeOldestFrame;
            } else {
                states_.marginalize_type = BackendMarginalizeType::kMarginalizeSubnewFrame;
            }
        } else {
            states_.marginalize_type = BackendMarginalizeType::kNotMarginalize;
        }

        // Try to marginalize if necessary.
        if (!TryToMarginalize()) {
            ResetToReintialize();
            ReportWarn("[Backend] Backend failed to marginalize.");
            return true;
        } else {
            ReportInfo(GREEN "[Backend] Backend succeed to marginalize states within " << timer.TockTickInMillisecond() << " ms." RESET_COLOR);
        }
    }

    // Control the dimension of local map.
    RETURN_FALSE_IF(!ControlLocalMapDimension());

    // Update states and record log.
    UpdateBackendStates();
    RecordBackendStatesLog();

    return true;
}

void Backend::Reset() {
    // Clear stored states in data_manager.
    data_manager_->frames_with_bias().clear();
    data_manager_->visual_local_map()->Clear();

    // Reset status.
    states_.is_initialized = false;
}

void Backend::ResetToReintialize() {
    // Clear stored states in data_manager.
    while (data_manager_->frames_with_bias().size() >= data_manager_->options().kMaxStoredNewFrames) {
        data_manager_->frames_with_bias().pop_front();
    }
    data_manager_->visual_local_map()->Clear();

    // Reset status.
    states_.is_initialized = false;
}

}
