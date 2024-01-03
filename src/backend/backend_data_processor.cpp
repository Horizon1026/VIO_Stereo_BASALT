#include "backend.h"
#include "log_report.h"
#include "geometry_triangulation.h"

namespace VIO {

void Backend::RecomputeImuPreintegration() {
    // Compute imu preintegration.
    for (auto &frame : data_manager_->frames_with_bias()) {
        frame.imu_preint_block.Reset();

        frame.imu_preint_block.SetImuNoiseSigma(imu_model_->options().kAccelNoise,
                                                imu_model_->options().kGyroNoise,
                                                imu_model_->options().kAccelRandomWalk,
                                                imu_model_->options().kGyroRandomWalk);
        const int32_t max_idx = static_cast<int32_t>(frame.packed_measure->imus.size());
        for (int32_t i = 1; i < max_idx; ++i) {
            frame.imu_preint_block.Propagate(*frame.packed_measure->imus[i - 1], *frame.packed_measure->imus[i]);
        }
    }
}

bool Backend::TriangulizeAllVisualFeatures() {
    using namespace VISION_GEOMETRY;
    Triangulator solver;
    solver.options().kMethod = Triangulator::TriangulationMethod::kAnalytic;

    // Preallcate memory for temp variables.
    const int32_t max_capacity = data_manager_->options().kMaxStoredKeyFrames * data_manager_->camera_extrinsics().size();
    std::vector<Quat> q_wc_vec;
    std::vector<Vec3> p_wc_vec;
    std::vector<Vec2> norm_xy_vec;
    q_wc_vec.reserve(max_capacity);
    p_wc_vec.reserve(max_capacity);
    norm_xy_vec.reserve(max_capacity);

    // Iterate all feature in visual_local_map to triangulize.
    int32_t triangulize_num = 0;
    for (auto &pair : data_manager_->visual_local_map()->features()) {
        auto &feature = pair.second;
        q_wc_vec.clear();
        p_wc_vec.clear();
        norm_xy_vec.clear();

        // Extract all observations.
        const uint32_t max_observe_num = feature.observes().size();
        const uint32_t first_frame_id = feature.first_frame_id();
        const uint32_t final_frame_id = feature.final_frame_id();
        for (uint32_t id = 0; id < max_observe_num; ++id) {
            // Extract states of selected frame.
            const uint32_t frame_id = first_frame_id + id;
            RETURN_FALSE_IF(frame_id > final_frame_id);
            const auto frame_ptr = data_manager_->visual_local_map()->frame(frame_id);
            RETURN_FALSE_IF(frame_ptr == nullptr);
            const Quat q_wc = frame_ptr->q_wc();
            const Vec3 p_wc = frame_ptr->p_wc();

            // Add mono-view observations.
            const auto &obv = feature.observe(frame_id);
            RETURN_FALSE_IF(obv.empty());
            const Vec2 norm_xy = obv[0].rectified_norm_xy;
            q_wc_vec.emplace_back(q_wc);
            p_wc_vec.emplace_back(p_wc);
            norm_xy_vec.emplace_back(norm_xy);

            // Add multi-view observations.
            RETURN_FALSE_IF(data_manager_->camera_extrinsics().size() < obv.size());
            const Vec3 p_ic0 = data_manager_->camera_extrinsics()[0].p_ic;
            const Quat q_ic0 = data_manager_->camera_extrinsics()[0].q_ic;
            const Quat q_wi = q_wc * q_ic0.inverse();
            for (uint32_t i = 1; i < obv.size(); ++i) {
                const Vec3 p_ici = data_manager_->camera_extrinsics()[i].p_ic;
                const Quat q_ici = data_manager_->camera_extrinsics()[i].q_ic;
                // T_wci = T_wc0 * T_ic0.inv * T_ici.
                /*  [R_wci  t_wci] = [R_wc0  t_wc0] * [R_ic0.t  -R_ic0.t * t_ic0] * [R_ici  t_ici]
                    [  0      1  ]   [  0      1  ]   [   0              1      ]   [  0      1  ]
                                   = [R_wc0 * R_ic0.t  -R_wc0 * R_ic0.t * t_ic0 + t_wc0] * [R_ici  t_ici]
                                     [       0                        1                ]   [  0      1  ]
                                   = [R_wc0 * R_ic0.t * R_ici  R_wc0 * R_ic0.t * t_ici - R_wc0 * R_ic0.t * t_ic0 + t_wc0]
                                     [           0                                          1                           ] */
                const Quat q_wci = q_wi * q_ici;
                const Vec3 p_wci = q_wi * p_ici - q_wi * p_ic0 + p_wc;
                const Vec2 norm_xy_i = obv[i].rectified_norm_xy;
                q_wc_vec.emplace_back(q_wci);
                p_wc_vec.emplace_back(p_wci);
                norm_xy_vec.emplace_back(norm_xy_i);
            }
        }

        // Triangulize feature.
        Vec3 p_w = Vec3::Zero();
        if (solver.Triangulate(q_wc_vec, p_wc_vec, norm_xy_vec, p_w)) {
            feature.param() = p_w;
            feature.status() = FeatureSolvedStatus::kSolved;
            ++triangulize_num;
        } else {
            feature.status() = FeatureSolvedStatus::kUnsolved;
        }
    }

    // Report triangulization result.
    ReportInfo("[Backend] Backend triangulized " << triangulize_num << " / " <<
        data_manager_->visual_local_map()->features().size() << ".");
    return true;
}

bool Backend::AddNewestFrameWithBiasIntoLocalMap() {
    RETURN_TRUE_IF(data_manager_->frames_with_bias().empty() || data_manager_->visual_local_map()->frames().empty());

    auto &newest_frame_imu = data_manager_->frames_with_bias().back();
    if (newest_frame_imu.visual_measure == nullptr) {
        ReportError("[Backend] Error: data_manager_->frames_with_bias().back().visual_measure == nullptr.");
        return false;
    }

    // Preintegrate imu measurements.
    auto &sub_new_frame_with_imu = *std::prev(std::prev(data_manager_->frames_with_bias().end()));
    newest_frame_imu.imu_preint_block.Reset();
    newest_frame_imu.imu_preint_block.bias_gyro() = sub_new_frame_with_imu.imu_preint_block.bias_gyro();
    newest_frame_imu.imu_preint_block.bias_accel() = sub_new_frame_with_imu.imu_preint_block.bias_accel();
    newest_frame_imu.imu_preint_block.SetImuNoiseSigma(imu_model_->options().kAccelNoise,
                                                        imu_model_->options().kGyroNoise,
                                                        imu_model_->options().kAccelRandomWalk,
                                                        imu_model_->options().kGyroRandomWalk);
    const int32_t max_idx = static_cast<int32_t>(newest_frame_imu.packed_measure->imus.size());
    for (int32_t i = 1; i < max_idx; ++i) {
        newest_frame_imu.imu_preint_block.Propagate(*newest_frame_imu.packed_measure->imus[i - 1], *newest_frame_imu.packed_measure->imus[i]);
    }

    // Add new frame into local map.
    const auto &sub_new_frame = data_manager_->visual_local_map()->frames().back();
    const int32_t frame_id = data_manager_->visual_local_map()->frames().back().id() + 1;
    data_manager_->visual_local_map()->AddNewFrameWithFeatures(newest_frame_imu.visual_measure->features_id,
                                                                newest_frame_imu.visual_measure->observes_per_frame,
                                                                newest_frame_imu.time_stamp_s,
                                                                frame_id);
    auto &newest_frame = data_manager_->visual_local_map()->frames().back();


    Vec3 p_wi = Vec3::Zero();
    Quat q_wi = Quat::Identity();
    const Vec3 &p_ic = data_manager_->camera_extrinsics().front().p_ic;
    const Quat &q_ic = data_manager_->camera_extrinsics().front().q_ic;
    Utility::ComputeTransformTransformInverse(sub_new_frame.p_wc(), sub_new_frame.q_wc(), p_ic, q_ic, p_wi, q_wi);

    Vec3 new_p_wi = Vec3::Zero();
    Quat new_q_wi = Quat::Identity();
    Vec3 new_v_wi = Vec3::Zero();
    const float dt = newest_frame_imu.imu_preint_block.integrate_time_s();
    new_q_wi = q_wi * newest_frame_imu.imu_preint_block.q_ij();
    new_p_wi = q_wi * newest_frame_imu.imu_preint_block.p_ij() + p_wi +
        sub_new_frame.v_w() * dt - 0.5f * options_.kGravityInWordFrame * dt * dt;
    new_v_wi = q_wi * newest_frame_imu.imu_preint_block.v_ij() + sub_new_frame.v_w() -
        options_.kGravityInWordFrame * dt;

    Utility::ComputeTransformTransform(new_p_wi, new_q_wi, p_ic, q_ic, newest_frame.p_wc(), newest_frame.q_wc());
    newest_frame.v_w() = new_v_wi;

    return data_manager_->visual_local_map()->SelfCheck();
}

}
