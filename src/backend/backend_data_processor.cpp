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
    const int32_t max_capacity = data_manager_->options().kMaxStoredKeyframes * data_manager_->camera_extrinsics().size();
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
            CONTINUE_IF(data_manager_->camera_extrinsics().size() < obv.size());
            for (uint32_t i = 1; i < obv.size(); ++i) {
                const Vec3 p_ic0 = data_manager_->camera_extrinsics()[0].p_ic;
                const Quat q_ic0 = data_manager_->camera_extrinsics()[0].q_ic;
                const Vec3 p_ici = data_manager_->camera_extrinsics()[i].p_ic;
                const Quat q_ici = data_manager_->camera_extrinsics()[i].q_ic;
                // T_wci = T_wc0 * T_ic0.inv * T_ici.
            }
        }

        // Triangulize feature.
        Vec3 p_w = Vec3::Zero();
        if (solver.Triangulate(q_wc_vec, p_wc_vec, norm_xy_vec, p_w)) {
            feature.param().p_w = p_w;
            feature.param().is_solved = true;
            ++triangulize_num;
        }
    }

    // Report triangulization result.
    ReportInfo("[Backend] Backend triangulized " << triangulize_num << " / " <<
        data_manager_->visual_local_map()->features().size() << ".");
    return true;
}

}
