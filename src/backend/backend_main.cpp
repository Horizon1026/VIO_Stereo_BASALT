#include "backend.h"
#include "log_report.h"
#include "tick_tock.h"

namespace VIO {

bool Backend::RunOnce() {
    if (data_manager_ == nullptr) {
        ReportError("[Backend] Backend cannot link with data manager.");
        return false;
    }

    // If backend is not initialized, try to initialize.
    if (!status_.is_initialized) {
        TickTock timer;
        if (TryToInitialize()) {
            status_.is_initialized = true;
            ReportInfo(GREEN "[Backend] Backend succeed to initialize within " << timer.TockTickInMillisecond() << " ms." RESET_COLOR);
        } else {
            ResetToReintialize();
            ReportWarn("[Backend] Backend failed to initialize. All states will be reset for reinitialization.");
            return true;
        }
    }

    // If backend is initialized, try to estimate states.
    if (status_.is_initialized) {
        TickTock timer;
        if (!TryToEstimate()) {
            ResetToReintialize();
            ReportWarn("[Backend] Backend failed to estimate.");
            return true;
        } else {
            ReportInfo(GREEN "[Backend] Backend succeed to estimate states within " << timer.TockTickInMillisecond() << " ms." RESET_COLOR);
        }

        // Debug.
        should_quit_ = true;
        ShowLocalMapWithFrames();
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
