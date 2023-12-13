#include "backend.h"
#include "log_report.h"
#include "math_kinematics.h"
#include "visualizor.h"
#include "visualizor_3d.h"

using namespace SLAM_VISUALIZOR;

namespace VIO {

void Backend::ShowFeaturePairsBetweenTwoFrames(const uint32_t ref_frame_id,
                                               const uint32_t cur_frame_id,
                                               const bool use_rectify) {
    // Get covisible features only in left camera.
    std::vector<FeatureType *> covisible_features;
    if (!data_manager_->visual_local_map()->GetCovisibleFeatures(ref_frame_id, cur_frame_id, covisible_features)) {
        ReportError("[Backend] Failed to get covisible features between frame " << ref_frame_id << " and " << cur_frame_id << ".");
        return;
    }

    std::vector<Vec2> ref_pixel_uv;
    std::vector<Vec2> cur_pixel_uv;
    if (use_rectify) {
        Vec2 rectify_pixel_uv;
        for (const auto &feature_ptr : covisible_features) {
            visual_frontend_->camera_model()->LiftFromNormalizedPlaneToImagePlane(feature_ptr->observe(ref_frame_id)[0].rectified_norm_xy, rectify_pixel_uv);
            ref_pixel_uv.emplace_back(rectify_pixel_uv);
            visual_frontend_->camera_model()->LiftFromNormalizedPlaneToImagePlane(feature_ptr->observe(cur_frame_id)[0].rectified_norm_xy, rectify_pixel_uv);
            cur_pixel_uv.emplace_back(rectify_pixel_uv);
        }
    } else {
        for (const auto &feature_ptr : covisible_features) {
            ref_pixel_uv.emplace_back(feature_ptr->observe(ref_frame_id)[0].raw_pixel_uv);
            cur_pixel_uv.emplace_back(feature_ptr->observe(cur_frame_id)[0].raw_pixel_uv);
        }
    }

    // Create gray image of ref image.
    uint32_t min_frames_idx = data_manager_->visual_local_map()->frames().front().id();
    auto ref_frame_iter = data_manager_->frames_with_bias().begin();
    while (min_frames_idx < ref_frame_id) {
        ++ref_frame_iter;
        ++min_frames_idx;
    }
    const GrayImage ref_image(ref_frame_iter->packed_measure->left_image->image);

    // Create gray image of cur image.
    min_frames_idx = data_manager_->visual_local_map()->frames().front().id();
    auto cur_frame_iter = data_manager_->frames_with_bias().begin();
    while (min_frames_idx < cur_frame_id) {
        ++cur_frame_iter;
        ++min_frames_idx;
    }
    const GrayImage cur_image(cur_frame_iter->packed_measure->left_image->image);

    const std::vector<uint8_t> tracked_status(ref_pixel_uv.size(), 1);
    if (use_rectify) {
        // Correct distorted image.
        MatImg ref_mat_image, cur_mat_image;
        ref_mat_image.resize(ref_image.rows(), ref_image.cols());
        cur_mat_image.resize(ref_image.rows(), ref_image.cols());
        GrayImage ref_rectify_image(ref_mat_image);
        GrayImage cur_rectify_image(cur_mat_image);
        visual_frontend_->camera_model()->CorrectDistortedImage(ref_image, ref_rectify_image);
        visual_frontend_->camera_model()->CorrectDistortedImage(cur_image, cur_rectify_image);

        // Draw tracking results.
        Visualizor::ShowImageWithTrackedFeatures("ref and cur rectify image", ref_rectify_image, cur_rectify_image,
            ref_pixel_uv, cur_pixel_uv, tracked_status);
    } else {
        // Draw tracking results.
        Visualizor::ShowImageWithTrackedFeatures("ref and cur raw image", ref_image, cur_image,
            ref_pixel_uv, cur_pixel_uv, tracked_status);
    }

    Visualizor::WaitKey(0);
}

void Backend::ShowLocalMapWithFrames(const int32_t delay_ms) {
    Visualizor3D::Clear();

    // Add word frame.
    Visualizor3D::poses().emplace_back(PoseType{
        .p_wb = Vec3::Zero(),
        .q_wb = Quat::Identity(),
        .scale = 1.0f,
    });

    // Add all features in locap map.
    for (const auto &pair : data_manager_->visual_local_map()->features()) {
        const auto &feature = pair.second;
        CONTINUE_IF(!feature.param().is_solved);
        Visualizor3D::points().emplace_back(PointType{
            .p_w = feature.param().p_w,
            .color = RgbPixel{.r = 0, .g = 255, .b = 255},
            .radius = 2,
        });
    }

    // Add all frames in locap map.
    Vec3 p_wi = Vec3::Zero();
    Quat q_wi = Quat::Identity();
    Vec3 p_wc = Vec3::Zero();
    Quat q_wc = Quat::Identity();
    for (const auto &frame : data_manager_->visual_local_map()->frames()) {
        // Add imu frame in local map.
        Utility::ComputeTransformTransformInverse(frame.p_wc(), frame.q_wc(),
            data_manager_->camera_extrinsics().front().p_ic,
            data_manager_->camera_extrinsics().front().q_ic, p_wi, q_wi);
        Visualizor3D::poses().emplace_back(PoseType{ .p_wb = p_wi, .q_wb = q_wi, .scale = 0.1f });

        // Add all camera frames in local map.
        for (const auto &extrinsic : data_manager_->camera_extrinsics()) {
            Utility::ComputeTransformTransform(p_wi, q_wi, extrinsic.p_ic, extrinsic.q_ic, p_wc, q_wc);
            Visualizor3D::poses().emplace_back(PoseType{ .p_wb = p_wc, .q_wb = q_wc, .scale = 0.05f });
        }
    }

    while (!Visualizor3D::ShouldQuit()) {
        Visualizor3D::Refresh("Visualizor 3D", delay_ms);
    }
}

}
