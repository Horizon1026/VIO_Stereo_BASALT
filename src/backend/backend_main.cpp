#include "backend.h"
#include "log_report.h"
#include "tick_tock.h"

namespace VIO {

bool Backend::RunOnce() {
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
            states_.is_initialized = true;
            ReportInfo(GREEN "[Backend] Backend succeed to initialize within " << timer.TockTickInMillisecond() << " ms." RESET_COLOR);
        } else {
            ResetToReintialize();
            ReportWarn("[Backend] Backend failed to initialize. All states will be reset for reinitialization.");
            return true;
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

        // Debug.
        ShowTinyInformationOfVisualLocalMap();

        // Decide marginalization type.
        if (data_manager_->visual_local_map()->frames().size() >= data_manager_->options().kMaxStoredKeyFrames) {
            // if (data_manager_->frames_with_bias().front().visual_measure->is_current_keyframe) {
                states_.marginalize_type = BackendMarginalizeType::kMarginalizeOldestFrame;
            // } else {
            //     states_.marginalize_type = BackendMarginalizeType::kMarginalizeSubnewFrame;
            // }
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
    switch (states_.marginalize_type) {
        case BackendMarginalizeType::kMarginalizeOldestFrame: {
            const auto oldest_frame_id = data_manager_->visual_local_map()->frames().front().id();
            data_manager_->visual_local_map()->RemoveFrame(oldest_frame_id);
            RETURN_FALSE_IF(!data_manager_->visual_local_map()->SelfCheck());
            break;
        }
        case BackendMarginalizeType::kMarginalizeSubnewFrame: {
            const auto subnew_frame_id = data_manager_->visual_local_map()->frames().back().id() -
                data_manager_->options().kMaxStoredNewFrames + 1;
            data_manager_->visual_local_map()->RemoveFrame(subnew_frame_id);
            RETURN_FALSE_IF(!data_manager_->visual_local_map()->SelfCheck());
            break;
        }
        default:
        case BackendMarginalizeType::kNotMarginalize: {

            break;
        }
    }
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
