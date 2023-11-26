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

            // Debug.
            should_quit_ = true;
        } else {
            ResetToReintialize();
            ReportWarn("[Backend] Backend failed to initialize.");
            return true;
        }
    }

    return true;
}

void Backend::Reset() {
    // Clear stored states in data_manager.
    data_manager_->frames_with_bias().clear();
    data_manager_->visual_local_map()->Clear();

    // Reset status.
    status_.is_initialized = false;
}

void Backend::ResetToReintialize() {
    // Clear stored states in data_manager.
    while (data_manager_->frames_with_bias().size() >= data_manager_->options().kMaxStoredNewFrames) {
        data_manager_->frames_with_bias().pop_front();
    }
    data_manager_->visual_local_map()->Clear();

    // Reset status.
    status_.is_initialized = false;
}

}
