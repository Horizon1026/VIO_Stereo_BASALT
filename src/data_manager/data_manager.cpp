#include "data_manager.h"

namespace VIO {

// Transform packed measurements to a new frame.
bool DataManager::ProcessMeasure(std::unique_ptr<PackedMeasurement> &new_packed_measure,
                                 std::unique_ptr<FrontendOutputData> &new_visual_measure) {
    if (new_packed_measure == nullptr || new_visual_measure == nullptr) {
        ReportError("[DataManager] Input new_packed_measure or new_visual_measure is nullptr.");
        return false;
    }

    frames_with_bias_.emplace_back(FrameWithBias{});
    FrameWithBias &frame_with_bias = frames_with_bias_.back();
    frame_with_bias.time_stamp_s = new_packed_measure->left_image->time_stamp_s;
    frame_with_bias.packed_measure = std::move(new_packed_measure);
    frame_with_bias.visual_measure = std::move(new_visual_measure);

    if (frames_with_bias_.size() > options_.kMaxStoredNewFrames) {
        frames_with_bias_.pop_front();
    }

    return true;
}

}
