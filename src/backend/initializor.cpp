#include "backend.h"
#include "log_report.h"

namespace VIO {

bool Backend::TryToInitialize() {
    if (data_manager_->frames_with_bias().size() < data_manager_->options().kMaxStoredNewFrames) {
        ReportWarn("[Backend] Backend cannot initialize for lack of new frames.");
        return false;
    }

    // Convert the new frames into a covisible graph.
    if (!ConvertNewFramesToCovisibleGraphForInitialization()) {
        ReportError("[Backend] Backend failed to convert new frames to covisible graph.");
        return false;
    }

    // Estiamte gyro bias.
    // After this step, the following states are estimated:
    // bias_g, which is one-for-all.
    // q_wc (based on the first camera frame) of each frame.
    if (!EstimateGyroBiasForInitialization()) {
        ReportError("[Backend] Backend failed to estimate gyro bias.");
        return false;
    }

    return true;
}

bool Backend::ConvertNewFramesToCovisibleGraphForInitialization() {
    RETURN_FALSE_IF(data_manager_->visual_local_map() == nullptr);

    auto local_map_ptr = data_manager_->visual_local_map();
    for (const auto &frame : data_manager_->frames_with_bias()) {
        RETURN_FALSE_IF(frame.visual_measure == nullptr);
        local_map_ptr->AddNewFrameWithFeatures(frame.visual_measure->features_id,
                                               frame.visual_measure->observes_per_frame,
                                               frame.time_stamp_s);
    }
    RETURN_FALSE_IF(!local_map_ptr->SelfCheck());

    return true;
}


}
