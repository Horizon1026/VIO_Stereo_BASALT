#include "backend.h"
#include "log_report.h"
#include "tick_tock.h"

namespace VIO {

constexpr uint32_t kBackendStatesLogIndex = 1;

void Backend::RegisterLogPackages() {
    using namespace SLAM_DATA_LOG;

    std::unique_ptr<PackageInfo> package_ptr = std::make_unique<PackageInfo>();
    package_ptr->id = kBackendStatesLogIndex;
    package_ptr->name = "backend states";
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kUint8, .name = "is_initialized"});

    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "time_stamp_s"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "p_wi_x"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "p_wi_y"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "p_wi_z"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_w"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_x"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_y"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_z"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_pitch"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_roll"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_yaw"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "v_wi_x"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "v_wi_y"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "v_wi_z"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "bias_a_x"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "bias_a_y"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "bias_a_z"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "bias_g_x"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "bias_g_y"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "bias_g_z"});

    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kUint8, .name = "marginalize_type"});
    package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "prior_residual"});
    if (!logger_.RegisterPackage(package_ptr)) {
        ReportError("[Backend] Failed to register package for backend states log.");
    }
}

void Backend::RecordBackendStatesLog() {
    if (options().kEnableRecordBinaryCurveLog) {
        log_package_data_.is_initialized = states_.is_initialized;

        log_package_data_.time_stamp_s = states_.motion.time_stamp_s;

        log_package_data_.p_wi_x = states_.motion.p_wi.x();
        log_package_data_.p_wi_y = states_.motion.p_wi.y();
        log_package_data_.p_wi_z = states_.motion.p_wi.z();

        const Vec3 euler = Utility::QuaternionToEuler(states_.motion.q_wi);
        log_package_data_.q_wi_pitch = euler.x();
        log_package_data_.q_wi_roll = euler.y();
        log_package_data_.q_wi_yaw = euler.z();

        log_package_data_.q_wi_w = states_.motion.q_wi.w();
        log_package_data_.q_wi_x = states_.motion.q_wi.x();
        log_package_data_.q_wi_y = states_.motion.q_wi.y();
        log_package_data_.q_wi_z = states_.motion.q_wi.z();

        log_package_data_.v_wi_x = states_.motion.v_wi.x();
        log_package_data_.v_wi_y = states_.motion.v_wi.y();
        log_package_data_.v_wi_z = states_.motion.v_wi.z();

        log_package_data_.bias_a_x = states_.motion.ba.x();
        log_package_data_.bias_a_y = states_.motion.ba.y();
        log_package_data_.bias_a_z = states_.motion.ba.z();

        log_package_data_.bias_g_x = states_.motion.bg.x();
        log_package_data_.bias_g_y = states_.motion.bg.y();
        log_package_data_.bias_g_z = states_.motion.bg.z();

        log_package_data_.marginalize_type = static_cast<uint8_t>(states_.marginalize_type);
        log_package_data_.prior_residual = states_.prior.is_valid ? states_.prior.residual.squaredNorm() : 0.0f;

        // Record log.
        const float log_time_stamp_s = data_manager_->frames_with_bias().empty() ?
            0.0f : data_manager_->frames_with_bias().back().time_stamp_s;
        logger_.RecordPackage(kBackendStatesLogIndex, reinterpret_cast<const char *>(&log_package_data_), log_time_stamp_s);
    }
}

}