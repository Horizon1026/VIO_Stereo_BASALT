#include "backend.h"
#include "visual_edges.h"
#include "inertial_edges.h"
#include "visual_inertial_edges.h"

#include "solver_lm.h"
#include "solver_dogleg.h"

#include "log_report.h"
#include "tick_tock.h"

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
    std::vector<std::unique_ptr<Vertex<Scalar>>> all_frames_p_wc;
    std::vector<std::unique_ptr<VertexQuat<Scalar>>> all_frames_q_wc;
    for (const auto &frame : data_manager_->visual_local_map()->frames()) {
        all_frames_id.emplace_back(frame.id());
        all_frames_p_wc.emplace_back(std::make_unique<Vertex<Scalar>>(3, 3));
        all_frames_p_wc.back()->param() = frame.p_wc().cast<Scalar>();
        all_frames_q_wc.emplace_back(std::make_unique<VertexQuat<Scalar>>(4, 3));
        all_frames_q_wc.back()->param() << frame.q_wc().w(), frame.q_wc().x(), frame.q_wc().y(), frame.q_wc().z();
    }

    // [Vertices] Inverse depth of each feature.
    // [Edges] Visual reprojection factor.
    std::vector<uint32_t> all_features_id;
    std::vector<std::unique_ptr<Vertex<Scalar>>> all_features_invdep;
    std::vector<std::unique_ptr<EdgeFeatureInvdepToNormPlaneViaImu<Scalar>>> all_visual_reproj_factors;
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

        // Iterate all observations of this feature.
        const uint32_t min_frame_id = feature.first_frame_id();
        const uint32_t max_frame_id = feature.final_frame_id();
        const uint32_t idx_offset = min_frame_id - data_manager_->visual_local_map()->frames().front().id() + 1;
        for (uint32_t idx = min_frame_id + 1; idx <= max_frame_id; ++idx) {
            Vec4 observe_vector = Vec4::Zero();
            observe_vector.head<2>() = feature.observe(min_frame_id)[0].rectified_norm_xy;
            observe_vector.tail<2>() = feature.observe(idx)[0].rectified_norm_xy;

            // Add edge of visual repeojection factor.
            all_visual_reproj_factors.emplace_back(std::make_unique<EdgeFeatureInvdepToNormPlaneViaImu<Scalar>>());
            auto &visual_reproj_factor = all_visual_reproj_factors.back();
            visual_reproj_factor->SetVertex(all_features_invdep.back().get(), 0);
            visual_reproj_factor->SetVertex(all_frames_p_wc[min_frame_id - idx_offset].get(), 1);
            visual_reproj_factor->SetVertex(all_frames_q_wc[min_frame_id - idx_offset].get(), 2);
            visual_reproj_factor->SetVertex(all_frames_p_wc[idx - idx_offset].get(), 3);
            visual_reproj_factor->SetVertex(all_frames_q_wc[idx - idx_offset].get(), 4);
            visual_reproj_factor->SetVertex(all_cameras_p_ic[0].get(), 5);
            visual_reproj_factor->SetVertex(all_cameras_q_ic[0].get(), 6);
            visual_reproj_factor->observation() = observe_vector.cast<Scalar>();
            visual_reproj_factor->kernel() = std::make_unique<KernelHuber<Scalar>>(static_cast<Scalar>(0.8));
            if (!visual_reproj_factor->SelfCheck()) {
                ReportError("[Backend] Visual reprojection factor error. 'min_frame_id - idx_offset' = " << min_frame_id - idx_offset <<
                    ", 'idx - idx_offset' = " << idx - idx_offset << ", min_frame_id = " << min_frame_id << ", max_frame_id = " <<
                    max_frame_id << ", idx_offset = " << idx_offset << ".");
                return false;
            }
        }
    }

    // Construct graph problem, add all vertices and edges.
    Graph<Scalar> graph_optimization_problem;
    for (uint32_t i = 0; i < all_cameras_p_ic.size(); ++i) {
        graph_optimization_problem.AddVertex(all_cameras_p_ic[i].get());
        graph_optimization_problem.AddVertex(all_cameras_q_ic[i].get());
    }
    for (uint32_t i = 0; i < all_frames_p_wc.size(); ++i) {
        graph_optimization_problem.AddVertex(all_frames_p_wc[i].get());
        graph_optimization_problem.AddVertex(all_frames_q_wc[i].get());
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
    for (uint32_t i = 0; i < all_frames_p_wc.size(); ++i) {
        auto frame_ptr = data_manager_->visual_local_map()->frame(all_frames_id[i]);
        frame_ptr->p_wc() = all_frames_p_wc[i]->param().cast<float>();
        frame_ptr->q_wc().w() = all_frames_q_wc[i]->param()(0);
        frame_ptr->q_wc().x() = all_frames_q_wc[i]->param()(1);
        frame_ptr->q_wc().y() = all_frames_q_wc[i]->param()(2);
        frame_ptr->q_wc().z() = all_frames_q_wc[i]->param()(3);
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
