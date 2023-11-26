#include "backend.h"
#include "log_report.h"
#include "visualizor.h"

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

}
