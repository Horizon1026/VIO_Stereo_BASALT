#include "backend.h"
#include "visual_edges.h"
#include "inertial_edges.h"
#include "visual_inertial_edges.h"

#include "solver_lm.h"
#include "solver_dogleg.h"

#include "log_report.h"
#include "tick_tock.h"
#include "math_kinematics.h"

namespace VIO {

bool Backend::TryToEstimate() {
    // Compute information matrix of visual observation.
    const TMat2<DorF> visual_info_matrix = GetVisualObserveInformationMatrix();

    // Generate vertices of states to be optimized.
    // [Vertices] Extrinsics of each camera.
    std::vector<std::unique_ptr<Vertex<DorF>>> all_cameras_p_ic;
    std::vector<std::unique_ptr<VertexQuat<DorF>>> all_cameras_q_ic;
    for (const auto &extrinsic : data_manager_->camera_extrinsics()) {
        all_cameras_p_ic.emplace_back(std::make_unique<Vertex<DorF>>(3, 3));
        all_cameras_p_ic.back()->param() = extrinsic.p_ic.cast<DorF>();
        all_cameras_p_ic.back()->name() = std::string("p_ic");
        all_cameras_p_ic.back()->SetFixed(true);
        all_cameras_q_ic.emplace_back(std::make_unique<VertexQuat<DorF>>(4, 3));
        all_cameras_q_ic.back()->param() << extrinsic.q_ic.w(), extrinsic.q_ic.x(), extrinsic.q_ic.y(), extrinsic.q_ic.z();
        all_cameras_q_ic.back()->name() = std::string("q_ic");
        all_cameras_q_ic.back()->SetFixed(true);
    }

    // [Vertices] Camera pose of each frame.
    std::vector<uint32_t> all_frames_id;
    std::vector<std::unique_ptr<Vertex<DorF>>> all_frames_p_wi;
    std::vector<std::unique_ptr<VertexQuat<DorF>>> all_frames_q_wi;
    for (const auto &frame : data_manager_->visual_local_map()->frames()) {
        all_frames_id.emplace_back(frame.id());

        Vec3 p_wi = Vec3::Zero();
        Quat q_wi = Quat::Identity();
        Utility::ComputeTransformTransformInverse(frame.p_wc(), frame.q_wc(), data_manager_->camera_extrinsics().front().p_ic,
            data_manager_->camera_extrinsics().front().q_ic, p_wi, q_wi);

        all_frames_p_wi.emplace_back(std::make_unique<Vertex<DorF>>(3, 3));
        all_frames_p_wi.back()->param() = p_wi.cast<DorF>();
        all_frames_p_wi.back()->name() = std::string("p_wi") + std::to_string(frame.id());
        all_frames_q_wi.emplace_back(std::make_unique<VertexQuat<DorF>>(4, 3));
        all_frames_q_wi.back()->param() << q_wi.w(), q_wi.x(), q_wi.y(), q_wi.z();
        all_frames_q_wi.back()->name() = std::string("q_wi") + std::to_string(frame.id());
    }
    all_frames_p_wi.front()->SetFixed(true);
    all_frames_q_wi.front()->SetFixed(true);

    // [Vertices] Inverse depth of each feature.
    // [Edges] Visual reprojection factor.
    std::vector<uint32_t> all_features_id;
    std::vector<std::unique_ptr<Vertex<DorF>>> all_features_invdep;
    std::vector<std::unique_ptr<Edge<DorF>>> all_visual_reproj_factors;
    for (const auto &pair : data_manager_->visual_local_map()->features()) {
        const auto &feature = pair.second;
        CONTINUE_IF(feature.observes().size() < 2);
        CONTINUE_IF(feature.status() == FeatureSolvedStatus::kMarginalized);

        // Determine the range of all observations of this feature.
        const uint32_t min_frame_id = feature.first_frame_id();
        const uint32_t max_frame_id = feature.final_frame_id();
        const uint32_t idx_offset = min_frame_id - data_manager_->visual_local_map()->frames().front().id() + 1;

        // Compute inverse depth by p_w of this feature.
        const auto &frame = data_manager_->visual_local_map()->frame(feature.first_frame_id());
        const Vec3 p_c = frame->q_wc().inverse() * (feature.param() - frame->p_wc());
        const float invdep = 1.0f / p_c.z();
        CONTINUE_IF(p_c.z() < kZero);

        // Add vertex of feature invdep.
        all_features_id.emplace_back(feature.id());
        all_features_invdep.emplace_back(std::make_unique<Vertex<DorF>>(1, 1));
        all_features_invdep.back()->param() = TVec1<DorF>(invdep);
        all_features_invdep.back()->name() = std::string("invdep ") + std::to_string(feature.id());

        // Add edges of visual reprojection factor, considering two cameras view one frame.
        const auto &obv_in_ref = feature.observe(min_frame_id);
        Vec4 observe_vector = Vec4::Zero();
        observe_vector.head<2>() = obv_in_ref[0].rectified_norm_xy;
        for (uint32_t i = 1; i < obv_in_ref.size(); ++i) {
            observe_vector.tail<2>() = obv_in_ref[i].rectified_norm_xy;

            // Add edge of visual reprojection factor, considering two camera view one frame.
            all_visual_reproj_factors.emplace_back(std::make_unique<EdgeFeatureInvdepToNormPlaneViaImuWithinOneFramesTwoCamera<DorF>>());
            auto &visual_reproj_factor = all_visual_reproj_factors.back();
            visual_reproj_factor->SetVertex(all_features_invdep.back().get(), 0);
            visual_reproj_factor->SetVertex(all_cameras_p_ic[0].get(), 1);
            visual_reproj_factor->SetVertex(all_cameras_q_ic[0].get(), 2);
            visual_reproj_factor->SetVertex(all_cameras_p_ic[i].get(), 3);
            visual_reproj_factor->SetVertex(all_cameras_q_ic[i].get(), 4);
            visual_reproj_factor->observation() = observe_vector.cast<DorF>();
            visual_reproj_factor->information() = visual_info_matrix;
            visual_reproj_factor->kernel() = std::make_unique<KernelHuber<DorF>>(static_cast<DorF>(0.1));
            visual_reproj_factor->name() = std::string("one frame two cameras");
            RETURN_FALSE_IF(!visual_reproj_factor->SelfCheck());
        }

        // In order to add other edges, iterate all observations of this feature.
        for (uint32_t idx = min_frame_id + 1; idx <= max_frame_id; ++idx) {
            const auto &obv_in_cur = feature.observe(idx);
            observe_vector.tail<2>() = obv_in_cur[0].rectified_norm_xy;

            // Add edges of visual reprojection factor, considering one camera views two frames.
            all_visual_reproj_factors.emplace_back(std::make_unique<EdgeFeatureInvdepToNormPlaneViaImuWithinTwoFramesOneCamera<DorF>>());
            auto &visual_reproj_factor = all_visual_reproj_factors.back();
            visual_reproj_factor->SetVertex(all_features_invdep.back().get(), 0);
            visual_reproj_factor->SetVertex(all_frames_p_wi[min_frame_id - idx_offset].get(), 1);
            visual_reproj_factor->SetVertex(all_frames_q_wi[min_frame_id - idx_offset].get(), 2);
            visual_reproj_factor->SetVertex(all_frames_p_wi[idx - idx_offset].get(), 3);
            visual_reproj_factor->SetVertex(all_frames_q_wi[idx - idx_offset].get(), 4);
            visual_reproj_factor->SetVertex(all_cameras_p_ic[0].get(), 5);
            visual_reproj_factor->SetVertex(all_cameras_q_ic[0].get(), 6);
            visual_reproj_factor->observation() = observe_vector.cast<DorF>();
            visual_reproj_factor->information() = visual_info_matrix;
            visual_reproj_factor->kernel() = std::make_unique<KernelHuber<DorF>>(static_cast<DorF>(0.1));
            visual_reproj_factor->name() = std::string("two frames one camera");
            RETURN_FALSE_IF(!visual_reproj_factor->SelfCheck());

            // Add edges of visual reprojection factor, considering two cameras view two frames.
            for (uint32_t i = 1; i < obv_in_cur.size(); ++i) {
                observe_vector.tail<2>() = obv_in_cur[i].rectified_norm_xy;

                all_visual_reproj_factors.emplace_back(std::make_unique<EdgeFeatureInvdepToNormPlaneViaImuWithinTwoFramesTwoCamera<DorF>>());
                auto &visual_reproj_factor = all_visual_reproj_factors.back();
                visual_reproj_factor->SetVertex(all_features_invdep.back().get(), 0);
                visual_reproj_factor->SetVertex(all_frames_p_wi[min_frame_id - idx_offset].get(), 1);
                visual_reproj_factor->SetVertex(all_frames_q_wi[min_frame_id - idx_offset].get(), 2);
                visual_reproj_factor->SetVertex(all_frames_p_wi[idx - idx_offset].get(), 3);
                visual_reproj_factor->SetVertex(all_frames_q_wi[idx - idx_offset].get(), 4);
                visual_reproj_factor->SetVertex(all_cameras_p_ic[0].get(), 5);
                visual_reproj_factor->SetVertex(all_cameras_q_ic[0].get(), 6);
                visual_reproj_factor->SetVertex(all_cameras_p_ic[i].get(), 7);
                visual_reproj_factor->SetVertex(all_cameras_q_ic[i].get(), 8);
                visual_reproj_factor->observation() = observe_vector.cast<DorF>();
                visual_reproj_factor->information() = visual_info_matrix;
                visual_reproj_factor->kernel() = std::make_unique<KernelHuber<DorF>>(static_cast<DorF>(0.1));
                visual_reproj_factor->name() = std::string("two frames two cameras");
                RETURN_FALSE_IF(!visual_reproj_factor->SelfCheck());
            }
        }
    }

    // [Vertices] Velocity of each new frame.
    std::vector<std::unique_ptr<Vertex<DorF>>> all_new_frames_v_wi;
    const uint32_t min_frames_idx = data_manager_->visual_local_map()->frames().front().id();
    const uint32_t max_frames_idx = data_manager_->visual_local_map()->frames().back().id();
    const uint32_t idx_offset = data_manager_->visual_local_map()->frames().size() - data_manager_->frames_with_bias().size();
    for (uint32_t frame_idx = min_frames_idx + idx_offset; frame_idx <= max_frames_idx; ++frame_idx) {
        all_new_frames_v_wi.emplace_back(std::make_unique<Vertex<DorF>>(3, 3));
        all_new_frames_v_wi.back()->param() = data_manager_->visual_local_map()->frame(frame_idx)->v_w().cast<DorF>();
        all_new_frames_v_wi.back()->name() = std::string("v_wi") + std::to_string(frame_idx);
    }

    // [Vertices] Bias_accel and bias_gyro of each new frame.
    std::vector<std::unique_ptr<Vertex<DorF>>> all_new_frames_ba;
    std::vector<std::unique_ptr<Vertex<DorF>>> all_new_frames_bg;
    for (const auto &frame : data_manager_->frames_with_bias()) {
        // Add vertex of bias_accel and bias_gyro.
        all_new_frames_ba.emplace_back(std::make_unique<Vertex<DorF>>(3, 3));
        all_new_frames_ba.back()->param() = frame.imu_preint_block.bias_accel().cast<DorF>();
        all_new_frames_ba.back()->name() = std::string("bias_a");
        all_new_frames_bg.emplace_back(std::make_unique<Vertex<DorF>>(3, 3));
        all_new_frames_bg.back()->param() = frame.imu_preint_block.bias_gyro().cast<DorF>();
        all_new_frames_bg.back()->name() = std::string("bias_g");
    }
    RETURN_FALSE_IF(all_new_frames_v_wi.size() != all_new_frames_ba.size());

    // [Edges] Inerial preintegration factor.
    uint32_t frame_idx = idx_offset;
    uint32_t new_frame_idx = 0;
    std::vector<std::unique_ptr<Edge<DorF>>> all_imu_factors;
    for (auto it = std::next(data_manager_->frames_with_bias().begin()); it != data_manager_->frames_with_bias().end(); ++it) {
        // The imu preintegration block combined with the oldest 'new frame with bias' is useless.
        // Add edges of imu preintegration.
        const auto &frame = *it;
        all_imu_factors.emplace_back(std::make_unique<EdgeImuPreintegrationBetweenRelativePose<DorF>>(
            frame.imu_preint_block, options_.kGravityInWordFrame));
        auto &imu_factor = all_imu_factors.back();
        imu_factor->SetVertex(all_frames_p_wi[frame_idx].get(), 0);
        imu_factor->SetVertex(all_frames_q_wi[frame_idx].get(), 1);
        imu_factor->SetVertex(all_new_frames_v_wi[new_frame_idx].get(), 2);
        imu_factor->SetVertex(all_new_frames_ba[new_frame_idx].get(), 3);
        imu_factor->SetVertex(all_new_frames_bg[new_frame_idx].get(), 4);
        imu_factor->SetVertex(all_frames_p_wi[frame_idx + 1].get(), 5);
        imu_factor->SetVertex(all_frames_q_wi[frame_idx + 1].get(), 6);
        imu_factor->SetVertex(all_new_frames_v_wi[new_frame_idx + 1].get(), 7);
        imu_factor->SetVertex(all_new_frames_ba[new_frame_idx + 1].get(), 8);
        imu_factor->SetVertex(all_new_frames_bg[new_frame_idx + 1].get(), 9);
        imu_factor->name() = std::string("imu factor");
        RETURN_FALSE_IF(!imu_factor->SelfCheck());

        ++frame_idx;
        ++new_frame_idx;
        BREAK_IF(frame_idx > max_frames_idx);
    }

    // Construct graph problem, add all vertices and edges.
    Graph<DorF> graph_optimization_problem;
    for (uint32_t i = 0; i < all_cameras_p_ic.size(); ++i) {
        graph_optimization_problem.AddVertex(all_cameras_p_ic[i].get());
        graph_optimization_problem.AddVertex(all_cameras_q_ic[i].get());
    }
    for (uint32_t i = 0; i < all_frames_p_wi.size(); ++i) {
        graph_optimization_problem.AddVertex(all_frames_p_wi[i].get());
        graph_optimization_problem.AddVertex(all_frames_q_wi[i].get());
        if (i >= idx_offset) {
            const uint32_t j = i - idx_offset;
            graph_optimization_problem.AddVertex(all_new_frames_v_wi[j].get());
            graph_optimization_problem.AddVertex(all_new_frames_ba[j].get());
            graph_optimization_problem.AddVertex(all_new_frames_bg[j].get());
        }
    }
    for (auto &vertex : all_features_invdep) {
        graph_optimization_problem.AddVertex(vertex.get(), false);
    }
    for (auto &edge : all_visual_reproj_factors) {
        graph_optimization_problem.AddEdge(edge.get());
    }
    for (auto &edge : all_imu_factors) {
        graph_optimization_problem.AddEdge(edge.get());
    }

    if (options_.kReportAllInformation) {
        ReportInfo(YELLOW "[Backend] Estimator adds " <<
            all_cameras_p_ic.size() << " all_cameras_p_ic, " <<
            all_cameras_q_ic.size() << " all_cameras_q_ic, " <<
            all_features_invdep.size() << " all_features_invdep, " <<
            all_frames_p_wi.size() << " all_frames_p_wi, " <<
            all_frames_q_wi.size() << " all_frames_q_wi, " <<
            all_new_frames_v_wi.size() << " all_new_frames_v_wi, " <<
            all_new_frames_ba.size() << " all_new_frames_ba, " <<
            all_new_frames_bg.size() << " all_new_frames_bg, and " <<

            all_visual_reproj_factors.size() << " all_visual_reproj_factors, " <<
            all_imu_factors.size() << " all_imu_factors." RESET_COLOR);
    }

    // Add prior information if valid.
    if (states_.prior.is_valid) {
        graph_optimization_problem.prior_hessian() = states_.prior.hessian;
        graph_optimization_problem.prior_bias() = states_.prior.bias;
        graph_optimization_problem.prior_jacobian_t_inv() = states_.prior.jacobian_t_inv;
        graph_optimization_problem.prior_residual() = states_.prior.residual;

        ReportInfo("[Backend] Before estimation, prior residual squared norm is " << graph_optimization_problem.prior_residual().squaredNorm());
    }

    // Construct solver to solve this problem.
    SolverLm<DorF> solver;
    solver.options().kEnableReportEachIteration = options_.kReportAllInformation;
    solver.problem() = &graph_optimization_problem;
    solver.Solve(states_.prior.is_valid);

    // Show all vertices and incremental function in optimization problem.
    if (options_.kReportAllInformation) {
        solver.problem()->VerticesInformation();
        ShowMatrixImage("solve hessian", solver.problem()->hessian());
    }

    // Update all camera extrinsics.
    for (uint32_t i = 0; i < all_cameras_p_ic.size(); ++i) {
        data_manager_->camera_extrinsics()[i].p_ic = all_cameras_p_ic[i]->param().cast<float>();
        data_manager_->camera_extrinsics()[i].q_ic.w() = all_cameras_q_ic[i]->param()(0);
        data_manager_->camera_extrinsics()[i].q_ic.x() = all_cameras_q_ic[i]->param()(1);
        data_manager_->camera_extrinsics()[i].q_ic.y() = all_cameras_q_ic[i]->param()(2);
        data_manager_->camera_extrinsics()[i].q_ic.z() = all_cameras_q_ic[i]->param()(3);
    }

    // Update all frame pose in local map.
    const Vec3 &p_ic = data_manager_->camera_extrinsics().front().p_ic;
    const Quat &q_ic = data_manager_->camera_extrinsics().front().q_ic;
    for (uint32_t i = 0; i < all_frames_p_wi.size(); ++i) {
        auto frame_ptr = data_manager_->visual_local_map()->frame(all_frames_id[i]);
        const Vec3 p_wi = all_frames_p_wi[i]->param().cast<float>();
        const Quat q_wi = Quat(all_frames_q_wi[i]->param()(0), all_frames_q_wi[i]->param()(1), all_frames_q_wi[i]->param()(2), all_frames_q_wi[i]->param()(3));
        Utility::ComputeTransformTransform(p_wi, q_wi, p_ic, q_ic, frame_ptr->p_wc(), frame_ptr->q_wc());

        if (i >= idx_offset) {
            const uint32_t j = i - idx_offset;
            frame_ptr->v_w() = all_new_frames_v_wi[j]->param().cast<float>();
        }
    }

    // Update all feature position in local map.
    for (uint32_t i = 0; i < all_features_invdep.size(); ++i) {
        auto feature_ptr = data_manager_->visual_local_map()->feature(all_features_id[i]);
        auto frame_ptr = data_manager_->visual_local_map()->frame(feature_ptr->first_frame_id());
        auto &observe = feature_ptr->observe(feature_ptr->first_frame_id());
        const Vec3 p_c = Vec3(observe[0].rectified_norm_xy.x(), observe[0].rectified_norm_xy.y(), 1) / all_features_invdep[i]->param()(0);
        feature_ptr->param() = frame_ptr->q_wc() * p_c + frame_ptr->p_wc();

        if (p_c.z() > options_.kMinValidFeatureDepthInMeter && p_c.z() < options_.kMaxValidFeatureDepthInMeter) {
            feature_ptr->status() = FeatureSolvedStatus::kSolved;
        } else {
            feature_ptr->status() = FeatureSolvedStatus::kUnsolved;
        }
    }

    // Update imu preintegration.
    uint32_t idx = 0;
    for (auto &frame : data_manager_->frames_with_bias()) {
        frame.imu_preint_block.Reset();
        frame.imu_preint_block.bias_accel() = all_new_frames_ba[idx]->param().cast<float>();
        frame.imu_preint_block.bias_gyro() = all_new_frames_bg[idx]->param().cast<float>();
        frame.imu_preint_block.SetImuNoiseSigma(imu_model_->options().kAccelNoise,
                                                imu_model_->options().kGyroNoise,
                                                imu_model_->options().kAccelRandomWalk,
                                                imu_model_->options().kGyroRandomWalk);
        ++idx;

        const int32_t max_idx = static_cast<int32_t>(frame.packed_measure->imus.size());
        for (int32_t i = 1; i < max_idx; ++i) {
            frame.imu_preint_block.Propagate(*frame.packed_measure->imus[i - 1], *frame.packed_measure->imus[i]);
        }
    }

    // Update prior information.
    if (states_.prior.is_valid) {
        states_.prior.hessian = solver.problem()->prior_hessian();
        states_.prior.bias = solver.problem()->prior_bias();
        states_.prior.jacobian_t_inv = solver.problem()->prior_jacobian_t_inv();
        states_.prior.residual = solver.problem()->prior_residual();

        ReportInfo("[Backend] After estimation, prior residual squared norm is " << solver.problem()->prior_residual().squaredNorm());
    }

    return true;
}

TMat2<DorF> Backend::GetVisualObserveInformationMatrix() {
    const auto &camera_model = visual_frontend_->camera_model();
    const DorF residual_in_pixel = 1.0;
    const TVec2<DorF> visual_observe_info_vec = TVec2<DorF>(camera_model->fx() * camera_model->fx(),
        camera_model->fy() * camera_model->fy()) / residual_in_pixel;
    return visual_observe_info_vec.asDiagonal();
}

}
