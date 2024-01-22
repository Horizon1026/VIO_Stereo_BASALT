#include "data_manager.h"

namespace VIO {

namespace {
    constexpr uint32_t kDataManagerLocalMapLogIndex = 0;
}

void DataManager::Clear() {
    if (visual_local_map_ != nullptr) {
        visual_local_map_->Clear();
    }
    frames_with_bias_.clear();
    camera_extrinsics_.clear();
}

bool DataManager::Configuration(const std::string &log_file_name) {
    // Register packages for log file.
    if (options_.kEnableRecordBinaryCurveLog) {
        if (!logger_.CreateLogFile(log_file_name)) {
            ReportError("[DataManager] Failed to create log file.");
            options_.kEnableRecordBinaryCurveLog = false;
            return false;
        }

        RegisterLogPackages();
        logger_.PrepareForRecording();
    }

    return true;
}

void DataManager::RegisterLogPackages() {
    using namespace SLAM_DATA_LOG;

    std::unique_ptr<PackageInfo> package_ptr = std::make_unique<PackageInfo>();
    package_ptr->id = kDataManagerLocalMapLogIndex;
    package_ptr->name = "local map";
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kUint32, .name = "num_of_features"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kUint32, .name = "num_of_solved_features"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kUint32, .name = "num_of_marginalized_features"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kUint32, .name = "num_of_unsolved_features"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kUint32, .name = "num_of_features_observed_in_newest_keyframe"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kUint32, .name = "num_of_solved_features_observed_in_newest_keyframe"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kUint32, .name = "num_of_frames"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kUint32, .name = "num_of_keyframes"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kUint32, .name = "num_of_newframes"});
    if (!logger_.RegisterPackage(package_ptr)) {
        ReportError("[DataManager] Failed to register package for data manager log.");
    }
}

void DataManager::TriggerLogRecording(const float time_stamp_s) {
    RETURN_IF(visual_local_map_ == nullptr);

    RecordLocalMap(time_stamp_s);
    RecordCovisibleGraph(time_stamp_s);
}

void DataManager::RecordLocalMap(const float time_stamp_s) {
    log_package_local_map_ = DataManagerLocalMapLog{};
    log_package_local_map_.num_of_features = visual_local_map_->features().size();
    for (const auto &pair : visual_local_map_->features()) {
        const auto &feature = pair.second;
        switch (feature.status()) {
            case FeatureSolvedStatus::kUnsolved: {
                ++log_package_local_map_.num_of_unsolved_features;
                break;
            }
            case FeatureSolvedStatus::kSolved: {
                ++log_package_local_map_.num_of_solved_features;
                break;
            }
            case FeatureSolvedStatus::kMarginalized: {
                ++log_package_local_map_.num_of_marginalized_features;
                break;
            }
            default: break;
        }
    }

    const auto frame_ptr = visual_local_map_->frame(GetNewestKeyframeId());
    if (frame_ptr != nullptr) {
        for (const auto &pair : frame_ptr->features()) {
            const auto &feature_ptr = pair.second;
            ++log_package_local_map_.num_of_features_observed_in_newest_keyframe;
            if (feature_ptr->status() == FeatureSolvedStatus::kSolved) {
                ++log_package_local_map_.num_of_solved_features_observed_in_newest_keyframe;
            }
        }
    }

    log_package_local_map_.num_of_frames = visual_local_map_->frames().size();
    log_package_local_map_.num_of_newframes = frames_with_bias_.size();
    log_package_local_map_.num_of_keyframes = visual_local_map_->frames().empty() ? 0 :
        visual_local_map_->frames().size() - frames_with_bias_.size();

    logger_.RecordPackage(kDataManagerLocalMapLogIndex, reinterpret_cast<const char *>(&log_package_local_map_), time_stamp_s);
}

void DataManager::RecordCovisibleGraph(const float time_stamp_s) {

}

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

    return true;
}

// Get specified frame id.
uint32_t DataManager::GetNewestKeyframeId() {
    return visual_local_map_->frames().front().id() + options_.kMaxStoredKeyFrames - options_.kMaxStoredNewFrames - 1;
}

}
