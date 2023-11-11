#include "backend.h"
#include "log_report.h"

namespace VIO {

bool Backend::SelectTwoFramesWithMaxParallax(CovisibleGraphType *local_map,
                                             const FeatureType &feature,
                                             int32_t &frame_id_l,
                                             int32_t &frame_id_r) {
    const int32_t num_of_observes = feature.observes().size();
    RETURN_FALSE_IF(feature.observes().size() < 2);

    // Iterate all pairs of frames, select the pair with max parallex angle.
    float max_parallex_angle = -1.0f;

    for (int32_t i = 0; i < num_of_observes - 1; ++i) {
        for (int32_t j = i + 1; j < num_of_observes; ++j) {
            // Extract observations in frame i/j.
            const auto &observe_i = feature.observes()[i];
            const auto &observe_j = feature.observes()[j];
            RETURN_FALSE_IF(observe_i.empty() || observe_j.empty());
            const Vec3 norm_xyz_i = Vec3(observe_i[0].rectified_norm_xy.x(), observe_i[0].rectified_norm_xy.y(), 1.0f);
            const Vec3 norm_xyz_j = Vec3(observe_j[0].rectified_norm_xy.x(), observe_j[0].rectified_norm_xy.y(), 1.0f);

            // Extract rotation of frame i/j.
            const Quat &q_wc_i = local_map->frame(i + feature.first_frame_id())->q_wc();
            const Quat &q_wc_j = local_map->frame(j + feature.first_frame_id())->q_wc();

            // Compute parallex angle.
            const Quat q_cjci = q_wc_j.inverse() * q_wc_i;
            const Vec3 angle_axis = norm_xyz_j.cross(q_cjci * norm_xyz_i);
            const float parallex_angle = angle_axis.norm();

            if (parallex_angle > max_parallex_angle) {
                // Update the pair of frames with max parallex angle.
                max_parallex_angle = parallex_angle;
                frame_id_l = i;
                frame_id_r = j;
            }
        }
    }

    ReportDebug("[Backend] frame " << frame_id_l << ", " << frame_id_r <<
        " have max parallex angle " << max_parallex_angle << " rad.");

    return true;
}

bool Backend::EstimateVelocityAndGravityForInitialization() {
    const int32_t num_of_frames = data_manager_->visual_local_map()->frames().size();
    const int32_t num_of_imu_block = num_of_frames - 1;
    RETURN_FALSE_IF(num_of_imu_block < 1);

    // Localize the left camera extrinsic.
    const Quat q_ic = data_manager_->camera_extrinsics().front().q_ic;

    // Iterate all feature in visual_local_map.
    for (const auto &pair : data_manager_->visual_local_map()->features()) {
        const auto &feature = pair.second;
        const auto &observes = feature.observes();

        int32_t frame_id_l = 0;
        int32_t frame_id_r = 0;
        RETURN_FALSE_IF(!SelectTwoFramesWithMaxParallax(data_manager_->visual_local_map(),
            feature, frame_id_l, frame_id_r));

        // TODO:
    }

    return true;
}

}
