#include "backend.h"
#include "log_report.h"
#include "slam_memory.h"
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

void Backend::ShowMatrixImage(const std::string &title, const TMat<DorF> &matrix) {
    const uint32_t scale = 3;
    uint8_t *buf = (uint8_t *)malloc(matrix.rows() * matrix.cols() * scale * scale * sizeof(uint8_t));
    GrayImage image_matrix(buf, matrix.rows() * scale, matrix.cols() * scale, true);
    Visualizor::ConvertMatrixToImage<DorF>(matrix, image_matrix, 100.0f, scale);
    Visualizor::ShowImage(title, image_matrix);
    Visualizor::WaitKey(1);
}

void Backend::ShowLocalMapFramesAndFeatures() {
    // Camera name of each camera is camera_name[camera_id].
    std::vector<std::string> camera_name = {"left", "right"};

    for (auto &frame : data_manager_->visual_local_map()->frames()) {
        // If stereo, camera id can be 0 and 1.
        for (uint32_t camera_id = 0; camera_id < frame.raw_images().size(); ++camera_id) {
            // Convert gray image to rgb image.
            GrayImage gray_image(frame.raw_images()[camera_id]);
            RgbImage rgb_image;
            uint8_t *rgb_buf = (uint8_t *)SlamMemory::Malloc(gray_image.rows() * gray_image.cols() * 3 * sizeof(uint8_t));
            rgb_image.SetImage(rgb_buf, gray_image.rows(), gray_image.cols(), true);
            Visualizor::ConvertUint8ToRgb(gray_image.data(), rgb_image.data(), gray_image.rows() * gray_image.cols());

            // Draw all observed features in this frame and this camera image.
            for (auto &pair : frame.features()) {
                auto &feature = pair.second;
                auto &observe = feature->observe(frame.id());
                if (observe.size() > camera_id) {
                    // Draw feature in rgb image.
                    const Vec2 pixel_uv = observe[camera_id].raw_pixel_uv;
                    RgbPixel pixel_color = RgbPixel{.r = 255, .g = 255, .b = 0};
                    switch (feature->status()) {
                        case FeatureSolvedStatus::kSolved:
                            pixel_color = RgbPixel{.r = 0, .g = 255, .b = 0};
                            break;
                        case FeatureSolvedStatus::kMarginalized:
                            pixel_color = RgbPixel{.r = 0, .g = 0, .b = 255};
                            break;
                        default:
                        case FeatureSolvedStatus::kUnsolved:
                            pixel_color = RgbPixel{.r = 255, .g = 0, .b = 0};
                            break;
                    }

                    Visualizor::DrawSolidCircle(rgb_image, pixel_uv.x(), pixel_uv.y(), 3, pixel_color);
                    Visualizor::DrawString(rgb_image, std::to_string(feature->id()), pixel_uv.x(), pixel_uv.y(), pixel_color);
                }
            }
            Visualizor::ShowImage(std::string("frame ") + std::to_string(frame.id()) + std::string(" ") + camera_name[camera_id] +
                std::string(" at ") + std::to_string(frame.time_stamp_s()) + std::string("s"), rgb_image);
        }
    }
    Visualizor::WaitKey(1);
}

void Backend::ShowAllFramesWithBias() {
    // Camera name of each camera is camera_name[camera_id].
    std::vector<std::string> camera_name = {"left", "right"};

    for (auto &frame_with_bias : data_manager_->frames_with_bias()) {
        CONTINUE_IF(frame_with_bias.packed_measure == nullptr || frame_with_bias.visual_measure == nullptr);

        if (frame_with_bias.packed_measure->left_image != nullptr) {
            // Convert gray image to rgb image.
            GrayImage gray_image(frame_with_bias.packed_measure->left_image->image);
            RgbImage rgb_image;
            uint8_t *rgb_buf = (uint8_t *)SlamMemory::Malloc(gray_image.rows() * gray_image.cols() * 3 * sizeof(uint8_t));
            rgb_image.SetImage(rgb_buf, gray_image.rows(), gray_image.cols(), true);
            Visualizor::ConvertUint8ToRgb(gray_image.data(), rgb_image.data(), gray_image.rows() * gray_image.cols());

            // Draw all observed features in this frame and this camera image.
            for (uint32_t i = 0; i < frame_with_bias.visual_measure->features_id.size(); ++i) {
                const Vec2 pixel_uv = frame_with_bias.visual_measure->observes_per_frame[i][0].raw_pixel_uv;
                const RgbPixel pixel_color = RgbPixel{.r = 0, .g = 255, .b = 255};
                Visualizor::DrawSolidCircle(rgb_image, pixel_uv.x(), pixel_uv.y(), 3, pixel_color);
                Visualizor::DrawString(rgb_image, std::to_string(frame_with_bias.visual_measure->features_id[i]),
                    pixel_uv.x(), pixel_uv.y(), pixel_color);
            }

            // Draw image to show.
            Visualizor::ShowImage(std::string("frame with bias of ") + camera_name[0] + std::string(" at ") +
                std::to_string(frame_with_bias.time_stamp_s) + std::string("s"), rgb_image);
        }

        if (frame_with_bias.packed_measure->right_image != nullptr) {
            // Convert gray image to rgb image.
            GrayImage gray_image(frame_with_bias.packed_measure->right_image->image);
            RgbImage rgb_image;
            uint8_t *rgb_buf = (uint8_t *)SlamMemory::Malloc(gray_image.rows() * gray_image.cols() * 3 * sizeof(uint8_t));
            rgb_image.SetImage(rgb_buf, gray_image.rows(), gray_image.cols(), true);
            Visualizor::ConvertUint8ToRgb(gray_image.data(), rgb_image.data(), gray_image.rows() * gray_image.cols());

            // Draw all observed features in this frame and this camera image.
            for (uint32_t i = 0; i < frame_with_bias.visual_measure->features_id.size(); ++i) {
                CONTINUE_IF(frame_with_bias.visual_measure->observes_per_frame[i].size() < 2);
                const Vec2 pixel_uv = frame_with_bias.visual_measure->observes_per_frame[i][1].raw_pixel_uv;
                const RgbPixel pixel_color = RgbPixel{.r = 0, .g = 255, .b = 255};
                Visualizor::DrawSolidCircle(rgb_image, pixel_uv.x(), pixel_uv.y(), 3, pixel_color);
                Visualizor::DrawString(rgb_image, std::to_string(frame_with_bias.visual_measure->features_id[i]),
                    pixel_uv.x(), pixel_uv.y(), pixel_color);
            }

            // Draw image to show.
            Visualizor::ShowImage(std::string("frame with bias of ") + camera_name[1] + std::string(" at ") +
                std::to_string(frame_with_bias.time_stamp_s) + std::string("s"), rgb_image);
        }
    }

    Visualizor::WaitKey(1);
}

void Backend::ShowLocalMapInWorldFrame(const int32_t delay_ms) {
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
        CONTINUE_IF(feature.status() == FeatureSolvedStatus::kUnsolved);
        const RgbPixel color = feature.status() == FeatureSolvedStatus::kSolved ?
            RgbPixel{.r = 0, .g = 255, .b = 255} : RgbPixel{.r = 0, .g = 255, .b = 0};
        Visualizor3D::points().emplace_back(PointType{
            .p_w = feature.param(),
            .color = color,
            .radius = 2,
        });
    }

    // Add all frames in locap map.
    bool is_p_wi0_valid = false;
    Vec3 p_wi0 = Vec3::Zero();
    Vec3 p_wi = Vec3::Zero();
    Quat q_wi = Quat::Identity();
    Vec3 p_wc = Vec3::Zero();
    Quat q_wc = Quat::Identity();
    for (const auto &frame : data_manager_->visual_local_map()->frames()) {
        // Add imu frame in local map.
        Utility::ComputeTransformTransformInverse(frame.p_wc(), frame.q_wc(),
            data_manager_->camera_extrinsics().front().p_ic,
            data_manager_->camera_extrinsics().front().q_ic, p_wi, q_wi);
        Visualizor3D::poses().emplace_back(PoseType{ .p_wb = p_wi, .q_wb = q_wi, .scale = 0.02f });

        // Link relative imu pose.
        if (is_p_wi0_valid) {
            Visualizor3D::lines().emplace_back(LineType{ .p_w_i = p_wi0, .p_w_j = p_wi, .color = RgbPixel{.r = 255, .g = 255, .b = 255} });
        }
        p_wi0 = p_wi;
        is_p_wi0_valid = true;

        // Add all camera frames in local map.
        for (const auto &extrinsic : data_manager_->camera_extrinsics()) {
            Utility::ComputeTransformTransform(p_wi, q_wi, extrinsic.p_ic, extrinsic.q_ic, p_wc, q_wc);
            Visualizor3D::poses().emplace_back(PoseType{ .p_wb = p_wc, .q_wb = q_wc, .scale = 0.01f });
        }
    }

    while (!Visualizor3D::ShouldQuit()) {
        Visualizor3D::Refresh("Visualizor 3D", delay_ms);
        BREAK_IF(delay_ms < 1);
    }
}

void Backend::ShowSimpleInformationOfVisualLocalMap() {
    for (const auto &frame : data_manager_->frames_with_bias()) {
        ReportInfo(" - Frame with bias timestamp_s is " << frame.time_stamp_s);
        frame.imu_preint_block.SimpleInformation();
    }
    for (const auto &frame : data_manager_->visual_local_map()->frames()) {
        frame.SimpleInformation();
    }
}

void Backend::ShowTinyInformationOfVisualLocalMap() {
    ReportInfo("[Backend] Visual local map:");
    for (const auto &frame : data_manager_->visual_local_map()->frames()) {
        ReportInfo(" - frame " << frame.id() << " at " << frame.time_stamp_s() << "s, " <<
            " q_wc " << LogQuat(frame.q_wc()) << ", p_wc " << LogVec(frame.p_wc()) <<
            ", v_w " << LogVec(frame.v_w()));
    }
    for (const auto &frame : data_manager_->frames_with_bias()) {
        ReportInfo(" - frame with bias at " << frame.time_stamp_s << "s, " <<
            "bias a " << LogVec(frame.imu_preint_block.bias_accel()) << ", bias g " <<
            LogVec(frame.imu_preint_block.bias_gyro()));
    }
}

}
