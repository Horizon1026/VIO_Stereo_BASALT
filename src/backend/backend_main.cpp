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

    // If backend is initialized.
    if (status_.is_initialized) {

        // Try to estimate states.
        TickTock timer;
        if (!TryToEstimate()) {
            ResetToReintialize();
            ReportWarn("[Backend] Backend failed to estimate.");
            return true;
        } else {
            ReportInfo(GREEN "[Backend] Backend succeed to estimate states within " << timer.TockTickInMillisecond() << " ms." RESET_COLOR);
        }

        // Try to marginalize if necessary.
        // TODO:

        // Control the size of local map.
        // TODO:
        if (data_manager_->visual_local_map()->frames().size() > data_manager_->options().kMaxStoredKeyframes) {
            const auto oldest_frame_id = data_manager_->visual_local_map()->frames().front().id();
            data_manager_->visual_local_map()->RemoveFrame(oldest_frame_id);

        }

        // Debug.
        should_quit_ = true;
    }

    // Control the dimension of local map.
    // TODO:
    if (data_manager_->frames_with_bias().size() >= data_manager_->options().kMaxStoredNewFrames) {
        data_manager_->frames_with_bias().pop_front();
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
