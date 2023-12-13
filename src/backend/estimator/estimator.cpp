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

using Scalar = double;

bool Backend::TryToEstimate() {
    // Generate vertices of states to be optimized.
    // [Vertices] Extrinsics of each camera.
    std::vector<std::unique_ptr<Vertex<Scalar>>> all_cameras_p_ic;
    std::vector<std::unique_ptr<VertexQuat<Scalar>>> all_cameras_q_ic;
    for (const auto &extrinsic : data_manager_->camera_extrinsics()) {
        all_cameras_p_ic.emplace_back(std::make_unique<Vertex<Scalar>>(3, 3));
        all_cameras_p_ic.back()->param() = extrinsic.p_ic.cast<Scalar>();
        all_cameras_q_ic.emplace_back(std::make_unique<VertexQuat<Scalar>>(4, 3));
        all_cameras_q_ic.back()->param() << extrinsic.q_ic.w(), extrinsic.q_ic.x(), extrinsic.q_ic.y(), extrinsic.q_ic.z();
    }

    // [Vertices] Camera pose of each frame.
    std::vector<uint32_t> all_frames_id;
    std::vector<std::unique_ptr<Vertex<Scalar>>> all_frames_p_wi;
    std::vector<std::unique_ptr<VertexQuat<Scalar>>> all_frames_q_wi;
    for (const auto &frame : data_manager_->visual_local_map()->frames()) {
        all_frames_id.emplace_back(frame.id());

        Vec3 p_wi = Vec3::Zero();
        Quat q_wi = Quat::Identity();
        Utility::ComputeTransformTransformInverse(frame.p_wc(), frame.q_wc(), data_manager_->camera_extrinsics().front().p_ic,
            data_manager_->camera_extrinsics().front().q_ic, p_wi, q_wi);

        all_frames_p_wi.emplace_back(std::make_unique<Vertex<Scalar>>(3, 3));
        all_frames_p_wi.back()->param() = p_wi.cast<Scalar>();
        all_frames_q_wi.emplace_back(std::make_unique<VertexQuat<Scalar>>(4, 3));
        all_frames_q_wi.back()->param() << q_wi.w(), q_wi.x(), q_wi.y(), q_wi.z();
    }

    // [Vertices] Inverse depth of each feature.
    // [Edges] Visual reprojection factor.
    std::vector<uint32_t> all_features_id;
    std::vector<std::unique_ptr<Vertex<Scalar>>> all_features_invdep;
    std::vector<std::unique_ptr<Edge<Scalar>>> all_visual_reproj_factors;
    for (const auto &pair : data_manager_->visual_local_map()->features()) {
        const auto &feature = pair.second;
        CONTINUE_IF(!feature.param().is_solved);

        // Compute inverse depth by p_w of this feature.
        const auto &frame = data_manager_->visual_local_map()->frame(feature.first_frame_id());
        const Vec3 p_c = frame->q_wc().inverse() * (feature.param().p_w - frame->p_wc());
        const float invdep = 1.0f / p_c.z();
        CONTINUE_IF(std::isinf(invdep) || std::isnan(invdep));

        // Add vertex of feature invdep.
        all_features_id.emplace_back(feature.id());
        all_features_invdep.emplace_back(std::make_unique<Vertex<Scalar>>(1, 1));
        all_features_invdep.back()->param() = TVec1<Scalar>(invdep);

        // Determine the range of all observations of this feature.
        const uint32_t min_frame_id = feature.first_frame_id();
        const uint32_t max_frame_id = feature.final_frame_id();
        const uint32_t idx_offset = min_frame_id - data_manager_->visual_local_map()->frames().front().id() + 1;

        // Add edge of visual reprojection factor, considering two cameras view one frame.
        const auto &obv_in_ref = feature.observe(min_frame_id);
        Vec4 observe_vector = Vec4::Zero();
        observe_vector.head<2>() = obv_in_ref[0].rectified_norm_xy;
        for (uint32_t i = 1; i < obv_in_ref.size(); ++i) {
            observe_vector.tail<2>() = obv_in_ref[i].rectified_norm_xy;

            // Add edge of visual reprojection factor, considering two camera view one frame.
            all_visual_reproj_factors.emplace_back(std::make_unique<EdgeFeatureInvdepToNormPlaneViaImuWithinOneFramesTwoCamera<Scalar>>());
            auto &visual_reproj_factor = all_visual_reproj_factors.back();
            visual_reproj_factor->SetVertex(all_features_invdep.back().get(), 0);
            visual_reproj_factor->SetVertex(all_cameras_p_ic[0].get(), 1);
            visual_reproj_factor->SetVertex(all_cameras_q_ic[0].get(), 2);
            visual_reproj_factor->SetVertex(all_cameras_p_ic[i].get(), 3);
            visual_reproj_factor->SetVertex(all_cameras_q_ic[i].get(), 4);
            visual_reproj_factor->observation() = observe_vector.cast<Scalar>();
            visual_reproj_factor->kernel() = std::make_unique<KernelHuber<Scalar>>(static_cast<Scalar>(1.0));
            RETURN_FALSE_IF(!visual_reproj_factor->SelfCheck());
        }

        // Iterate all observations of this feature.
        for (uint32_t idx = min_frame_id + 1; idx <= max_frame_id; ++idx) {
            const auto &obv_in_cur = feature.observe(idx);
            observe_vector.tail<2>() = obv_in_cur[0].rectified_norm_xy;

            // Add edge of visual reprojection factor, considering one camera views two frames.
            all_visual_reproj_factors.emplace_back(std::make_unique<EdgeFeatureInvdepToNormPlaneViaImuWithinTwoFramesOneCamera<Scalar>>());
            auto &visual_reproj_factor = all_visual_reproj_factors.back();
            visual_reproj_factor->SetVertex(all_features_invdep.back().get(), 0);
            visual_reproj_factor->SetVertex(all_frames_p_wi[min_frame_id - idx_offset].get(), 1);
            visual_reproj_factor->SetVertex(all_frames_q_wi[min_frame_id - idx_offset].get(), 2);
            visual_reproj_factor->SetVertex(all_frames_p_wi[idx - idx_offset].get(), 3);
            visual_reproj_factor->SetVertex(all_frames_q_wi[idx - idx_offset].get(), 4);
            visual_reproj_factor->SetVertex(all_cameras_p_ic[0].get(), 5);
            visual_reproj_factor->SetVertex(all_cameras_q_ic[0].get(), 6);
            visual_reproj_factor->observation() = observe_vector.cast<Scalar>();
            visual_reproj_factor->kernel() = std::make_unique<KernelHuber<Scalar>>(static_cast<Scalar>(1.0));
            RETURN_FALSE_IF(!visual_reproj_factor->SelfCheck());

            // Add edge of visual reprojection factor, considering two cameras view two frames.
            for (uint32_t i = 1; i < obv_in_cur.size(); ++i) {
                observe_vector.tail<2>() = obv_in_cur[i].rectified_norm_xy;

                all_visual_reproj_factors.emplace_back(std::make_unique<EdgeFeatureInvdepToNormPlaneViaImuWithinTwoFramesTwoCamera<Scalar>>());
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
                visual_reproj_factor->observation() = observe_vector.cast<Scalar>();
                visual_reproj_factor->kernel() = std::make_unique<KernelHuber<Scalar>>(static_cast<Scalar>(1.0));
                RETURN_FALSE_IF(!visual_reproj_factor->SelfCheck());
            }
        }
    }

    // Construct graph problem, add all vertices and edges.
    Graph<Scalar> graph_optimization_problem;
    for (uint32_t i = 0; i < all_cameras_p_ic.size(); ++i) {
        graph_optimization_problem.AddVertex(all_cameras_p_ic[i].get());
        graph_optimization_problem.AddVertex(all_cameras_q_ic[i].get());
    }
    for (uint32_t i = 0; i < all_frames_p_wi.size(); ++i) {
        graph_optimization_problem.AddVertex(all_frames_p_wi[i].get());
        graph_optimization_problem.AddVertex(all_frames_q_wi[i].get());
    }
    for (auto &vertex : all_features_invdep) {
        graph_optimization_problem.AddVertex(vertex.get());
    }
    for (auto &edge : all_visual_reproj_factors) {
        graph_optimization_problem.AddEdge(edge.get());
    }

    // Construct solver to solve this problem.
    SolverLm<Scalar> solver;
    solver.problem() = &graph_optimization_problem;
    solver.Solve(false);

    // Update all camera extrinsics.
    for (uint32_t i = 0; i < all_cameras_p_ic.size(); ++i) {
        data_manager_->camera_extrinsics()[i].p_ic = all_cameras_p_ic[i]->param().cast<float>();
        data_manager_->camera_extrinsics()[i].q_ic.w() = all_cameras_q_ic[i]->param()(0);
        data_manager_->camera_extrinsics()[i].q_ic.x() = all_cameras_q_ic[i]->param()(1);
        data_manager_->camera_extrinsics()[i].q_ic.y() = all_cameras_q_ic[i]->param()(2);
        data_manager_->camera_extrinsics()[i].q_ic.z() = all_cameras_q_ic[i]->param()(3);
    }

    // Update all frame pose in local map.
    for (uint32_t i = 0; i < all_frames_p_wi.size(); ++i) {
        const Vec3 p_wi = all_frames_p_wi[i]->param().cast<float>();
        const Quat q_wi = Quat(all_frames_q_wi[i]->param()(0), all_frames_q_wi[i]->param()(1), all_frames_q_wi[i]->param()(2), all_frames_q_wi[i]->param()(3));

        auto frame_ptr = data_manager_->visual_local_map()->frame(all_frames_id[i]);
        const Vec3 &p_ic = data_manager_->camera_extrinsics().front().p_ic;
        const Quat &q_ic = data_manager_->camera_extrinsics().front().q_ic;
        Utility::ComputeTransformTransform(p_wi, q_wi, p_ic, q_ic, frame_ptr->p_wc(), frame_ptr->q_wc());
    }

    // Update all feature position in local map.
    for (uint32_t i = 0; i < all_features_invdep.size(); ++i) {
        auto feature_ptr = data_manager_->visual_local_map()->feature(all_features_id[i]);
        auto frame_ptr = data_manager_->visual_local_map()->frame(feature_ptr->first_frame_id());
        auto &observe = feature_ptr->observe(feature_ptr->first_frame_id());
        const Vec3 p_c = Vec3(observe[0].rectified_norm_xy.x(), observe[0].rectified_norm_xy.y(), 1) / all_features_invdep[i]->param()(0);
        feature_ptr->param().p_w = frame_ptr->q_wc() * p_c + frame_ptr->p_wc();
    }

    return true;
}

}
