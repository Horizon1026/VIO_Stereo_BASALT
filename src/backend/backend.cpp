#include "backend.h"
#include "log_report.h"

namespace VIO {

bool Backend::RunOnce() {
    if (data_manager_ == nullptr) {
        ReportError("[Backend] Backend cannot link with data manager.");
        return false;
    }

    // If backend is not initialized, try to initialize.
    if (!status_.is_initialized) {
        if (TryToInitialize()) {
            status_.is_initialized = true;
            ReportInfo(GREEN "[Backend] Backend succeed to initialize." RESET_COLOR);
        } else {
            ReportWarn("[Backend] Backend failed to initialize.");
            return false;
        }
    }

    return true;
}

bool Backend::TryToInitialize() {
    if (data_manager_->new_frames().size() < data_manager_->options().kMaxStoredNewFrames) {
        ReportWarn("[Backend] Backend cannot initialize for lack of new frames.");
        return false;
    }

    // TODO: How to initialize?
    // Convert the new frames into a covisible graph.
    std::unique_ptr<CovisibleGraphType> covisible_graph = std::make_unique<CovisibleGraphType>();
    for (const auto &frame : data_manager_->new_frames()) {
        RETURN_FALSE_IF(frame.visual_measure == nullptr);
        // covisible_graph->AddNewFrameWithFeatures(frame.visual_measure->features_id,
        //                                          frame.visual_measure->observes_per_frame,
        //                                          frame.time_stamp_s);
    }
    if (!covisible_graph->SelfCheck()) {
        ReportError("[Backend] Covisible graph of new frames failed to check itself.");
        return false;
    } else {
        ReportInfo("[Backend] Covisible graph of new frames succeed to check itself.");
    }

    // Conpute decomposed rotation of frames.

    // Estiamte gyro bias.

    return true;
}

}
