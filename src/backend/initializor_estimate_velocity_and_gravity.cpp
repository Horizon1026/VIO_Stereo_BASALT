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
                frame_id_l = i + feature.first_frame_id();
                frame_id_r = j + feature.first_frame_id();
            }
        }
    }

    // ReportDebug("[Backend] frame " << frame_id_l << ", " << frame_id_r <<
    //     " have max parallex angle " << max_parallex_angle << " rad.");

    return true;
}

bool Backend::EstimateVelocityAndGravityForInitialization() {
    const int32_t num_of_frames = data_manager_->visual_local_map()->frames().size();
    const int32_t num_of_imu_block = num_of_frames - 1;
    RETURN_FALSE_IF(num_of_imu_block < 1);

    // Compute imu blocks based on the first frame.
    const auto start_iter = std::next(data_manager_->frames_with_bias().begin());
    RETURN_FALSE_IF(start_iter == data_manager_->frames_with_bias().end());
    auto end_iter = std::next(start_iter);

    std::vector<ImuPreintegrateBlock> imu_blocks;
    imu_blocks.emplace_back(start_iter->imu_preint_block);
    for (int32_t i = 1; i < num_of_imu_block; ++i) {
        ++end_iter;

        ImuPreintegrateBlock new_imu_block(imu_blocks.back());
        new_imu_block.ResetIntegratedStates();
        for (auto iter = start_iter; iter != end_iter; ++iter) {
            const int32_t max_idx = static_cast<int32_t>(iter->packed_measure->imus.size());
            for (int32_t j = 1; j < max_idx; ++j) {
                new_imu_block.Propagate(*iter->packed_measure->imus[j - 1], *iter->packed_measure->imus[j]);
            }
        }
        imu_blocks.emplace_back(new_imu_block);
    }
    for (const auto &imu_preint_block : imu_blocks) {
        imu_preint_block.SimpleInformation();
    }

    // Localize the left camera extrinsic.
    const Quat q_ic = data_manager_->camera_extrinsics().front().q_ic;

    // Iterate all feature in visual_local_map.
    for (const auto &pair : data_manager_->visual_local_map()->features()) {
        const auto &feature = pair.second;

        // Select two frames with max parallex angle.
        int32_t frame_id_l = 0;
        int32_t frame_id_r = 0;
        RETURN_FALSE_IF(!SelectTwoFramesWithMaxParallax(data_manager_->visual_local_map(),
            feature, frame_id_l, frame_id_r));

        // Extract frame l/r.
        const auto frame_ptr_l = data_manager_->visual_local_map()->frame(frame_id_l);
        const auto frame_ptr_r = data_manager_->visual_local_map()->frame(frame_id_r);
        if (frame_ptr_l == nullptr || frame_ptr_r == nullptr) {
            ReportError("[Backend] Backend failed to find frame " << frame_id_l << " and " << frame_id_r << ".");
            return false;
        }
        const Mat3 R_wcl = frame_ptr_l->q_wc().toRotationMatrix();
        const Mat3 R_wcr = frame_ptr_r->q_wc().toRotationMatrix();

        // Extract observations of frame l/r.
        const auto &obv_l = feature.observe(frame_id_l);
        const auto &obv_r = feature.observe(frame_id_r);
        if (obv_l.empty() || obv_r.empty()) {
            ReportError("[Backend] Backend failed to find observations of frame " << frame_id_l << " and " << frame_id_r << ", obv_l.size() is " <<
                obv_l.size() << ", obv_r.size() is " << obv_r.size() << ".");
            return false;
        }
        const Vec3 norm_xyz_l = Vec3(obv_l[0].rectified_norm_xy.x(), obv_l[0].rectified_norm_xy.y(), 1.0f);
        const Vec3 norm_xyz_r = Vec3(obv_r[0].rectified_norm_xy.x(), obv_r[0].rectified_norm_xy.y(), 1.0f);

        // Iterate all observations of this feature.
        const int32_t num_of_observes = feature.observes().size();
        for (int32_t i = 0; i < num_of_observes; ++i) {
            const int32_t frame_id_i = i + feature.first_frame_id();
            CONTINUE_IF(frame_id_i == frame_id_r);

            // Extract frame i.
            const auto frame_ptr_i = data_manager_->visual_local_map()->frame(frame_id_i);
            if (frame_ptr_i == nullptr) {
                ReportError("[Backend] Backend failed to find frame " << frame_id_i << ".");
                return false;
            }
            const Mat3 R_wci = frame_ptr_i->q_wc().toRotationMatrix();

            // Extract observations of frame i/l/r.
            const auto &obv_i = feature.observe(frame_id_i);
            if (obv_i.empty()) {
                ReportError("[Backend] Backend failed to find observation of frame " << frame_id_l << ", obv_i.size() is " << obv_i.size() << ".");
                return false;
            }
            const Vec3 norm_xyz_i = Vec3(obv_i[0].rectified_norm_xy.x(), obv_i[0].rectified_norm_xy.y(), 1.0f);

            // Compute matrice B, C, D.
            const Mat3 skew_norm_xyz_i = Utility::SkewSymmetricMatrix(norm_xyz_i);
            const Mat3 skew_norm_xyz_r = Utility::SkewSymmetricMatrix(norm_xyz_r);
            const Mat3 R_cicl = R_wci.transpose() * R_wcl;
            const Mat3 R_crcl = R_wcr.transpose() * R_wcl;
            const Vec3 a_lr_tmp_t = Utility::SkewSymmetricMatrix(R_crcl * norm_xyz_l) * norm_xyz_r;
            const Mat1x3 a_lr_t = a_lr_tmp_t.transpose() * skew_norm_xyz_r;
            const Vec3 theta_lr_vector = skew_norm_xyz_r * R_crcl * norm_xyz_l;
            const float theta_lr = theta_lr_vector.squaredNorm();
            const Mat3 R_ci = q_ic.toRotationMatrix().transpose();

            const Mat3 B = skew_norm_xyz_i * R_cicl * norm_xyz_l * a_lr_t * R_wcr.transpose();
            const Mat3 C = theta_lr * skew_norm_xyz_i * R_wci.transpose();
            const Mat3 D = - B - C;
            const Mat3 B_prime = B * R_ci;
            const Mat3 C_prime = C * R_ci;
            const Mat3 D_prime = D * R_ci;

            // TODO: Need to recompute imu preint block.
        }
    }

    return true;
}

}
