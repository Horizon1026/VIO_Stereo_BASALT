#include "backend.h"
#include "visual_edges.h"
#include "inertial_edges.h"
#include "visual_inertial_edges.h"

#include "log_report.h"
#include "tick_tock.h"
#include "math_kinematics.h"

namespace VIO {

bool Backend::TryToMarginalize() {
    switch (states_.marginalize_type) {
        case BackendMarginalizeType::kMarginalizeOldestFrame: {
            return MarginalizeOldestFrame();
            break;
        }
        case BackendMarginalizeType::kMarginalizeSubnewFrame: {
            return MarginalizeSubnewFrame();
            break;
        }
        default:
        case BackendMarginalizeType::kNotMarginalize: {
            ReportInfo("[Bakcend] Backend not marginalize any frame.");
            break;
        }
    }

    return true;
}

bool Backend::MarginalizeOldestFrame() {
    ReportInfo("[Bakcend] Backend try to marginalize oldest frame.");

    // Compute information matrix of visual observation.
    const TMat2<DorF> visual_info_matrix = GetVisualObserveInformationMatrix();

    // Generate vertices of states to be optimized.
    // [Vertices] Extrinsics of each camera.
    std::vector<std::unique_ptr<Vertex<DorF>>> all_cameras_p_ic;
    std::vector<std::unique_ptr<VertexQuat<DorF>>> all_cameras_q_ic;
    for (const auto &extrinsic : data_manager_->camera_extrinsics()) {
        all_cameras_p_ic.emplace_back(std::make_unique<Vertex<DorF>>(3, 3));
        all_cameras_p_ic.back()->param() = extrinsic.p_ic.cast<DorF>();
        // all_cameras_p_ic.back()->SetFixed(true);
        all_cameras_q_ic.emplace_back(std::make_unique<VertexQuat<DorF>>(4, 3));
        all_cameras_q_ic.back()->param() << extrinsic.q_ic.w(), extrinsic.q_ic.x(), extrinsic.q_ic.y(), extrinsic.q_ic.z();
        // all_cameras_q_ic.back()->SetFixed(true);
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
        all_frames_q_wi.emplace_back(std::make_unique<VertexQuat<DorF>>(4, 3));
        all_frames_q_wi.back()->param() << q_wi.w(), q_wi.x(), q_wi.y(), q_wi.z();
    }

    // [Vertices] Inverse depth of each feature.
    // [Edges] Visual reprojection factor.
    std::vector<uint32_t> all_features_id;
    std::vector<std::unique_ptr<Vertex<DorF>>> all_features_invdep;
    std::vector<std::unique_ptr<Edge<DorF>>> all_visual_reproj_factors;
    for (const auto &pair : data_manager_->visual_local_map()->features()) {
        const auto &feature = pair.second;
        CONTINUE_IF(feature.observes().size() < 2);
        CONTINUE_IF(feature.first_frame_id() != data_manager_->visual_local_map()->frames().front().id());

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
        all_new_frames_v_wi.back()->param() = data_manager_->visual_local_map()->frame(frame_idx)->v_wc().cast<DorF>();
    }

    // [Vertices] Bias_accel and bias_gyro of each new frame.
    std::vector<std::unique_ptr<Vertex<DorF>>> all_new_frames_ba;
    std::vector<std::unique_ptr<Vertex<DorF>>> all_new_frames_bg;
    for (const auto &frame : data_manager_->frames_with_bias()) {
        // Add vertex of bias_accel and bias_gyro.
        all_new_frames_ba.emplace_back(std::make_unique<Vertex<DorF>>(3, 3));
        all_new_frames_ba.back()->param() = frame.imu_preint_block.bias_accel().cast<DorF>();
        all_new_frames_bg.emplace_back(std::make_unique<Vertex<DorF>>(3, 3));
        all_new_frames_bg.back()->param() = frame.imu_preint_block.bias_gyro().cast<DorF>();
    }
    RETURN_FALSE_IF(all_new_frames_v_wi.size() != all_new_frames_ba.size());

    // [Edges] Inerial preintegration factor.
    const uint32_t frame_idx = idx_offset;
    const uint32_t new_frame_idx = 0;
    std::vector<std::unique_ptr<Edge<DorF>>> all_imu_factors;
    // The imu preintegration block combined with the oldest 'new frame with bias' is useless.
    // Add edges of imu preintegration.
    const auto &frame = *std::next(data_manager_->frames_with_bias().begin());
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
    RETURN_FALSE_IF(!imu_factor->SelfCheck());

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
    ReportDebug(RED "[Backend] Marginalizor adds " <<
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

    // Add prior information if valid.
    if (states_.prior.is_valid) {
        graph_optimization_problem.prior_hessian() = states_.prior.hessian;
        graph_optimization_problem.prior_bias() = states_.prior.bias;
        graph_optimization_problem.prior_jacobian_t_inv() = states_.prior.jacobian_t_inv;
        graph_optimization_problem.prior_residual() = states_.prior.residual;
    }

    // Set vertices to be marged.
    std::vector<Vertex<DorF> *> vertices_to_be_marged = {
        all_frames_p_wi.front().get(),
        all_frames_q_wi.front().get(),
        all_new_frames_v_wi.front().get(),
        all_new_frames_ba.front().get(),
        all_new_frames_bg.front().get(),
    };

    // Do marginalization.
    Marginalizor<DorF> marger;
    marger.problem() = &graph_optimization_problem;
    marger.options().kSortDirection = SortMargedVerticesDirection::kSortAtBack;
    states_.prior.is_valid = marger.Marginalize(vertices_to_be_marged, states_.prior.is_valid);

    // Store prior information.
    if (states_.prior.is_valid) {
        states_.prior.hessian = marger.problem()->prior_hessian();
        states_.prior.bias = marger.problem()->prior_bias();
        states_.prior.jacobian_t_inv = marger.problem()->prior_jacobian_t_inv();
        states_.prior.residual = marger.problem()->prior_residual();
    }

    // Debug.
    ReportDebug("[Backend] Marginalized prior residual squared norm is " << marger.problem()->prior_residual().squaredNorm());
    ShowMatrixImage("marg hessian", marger.problem()->hessian());
    ShowMatrixImage("reverse hessian", marger.reverse_hessian());
    ShowMatrixImage("prior", marger.problem()->prior_hessian());

    return true;
}

bool Backend::MarginalizeSubnewFrame() {
    ReportInfo("[Bakcend] Backend try to marginalize subnew frame.");

    return true;
}

}
