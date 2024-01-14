#include "backend.h"
#include "general_edges.h"
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
    const uint32_t idx_offset = data_manager_->visual_local_map()->frames().size() - data_manager_->frames_with_bias().size();

    // Clear all vectors of vertices and edges.
    ClearBackendGraph();

    // Generate vertices of states to be optimized.
    // [Vertices] Extrinsics of each camera.
    // [Vertices] Camera pose of each frame.
    ConvertCameraPoseAndExtrinsicToVertices();

    // [Edges] Camera pose prior factor.
    // [Edges] Camera extrinsic prior factor.
    RETURN_FALSE_IF(!AddPriorFactorWhenNoPrior());

    // [Vertices] Inverse depth of each feature.
    // [Edges] Visual reprojection factor.
    RETURN_FALSE_IF(!ConvertFeatureInvdepAndAddVisualFactorForMarginalization());

    // [Vertices] Velocity of each new frame.
    ConvertImuMotionStatesToVertices();

    // [Edges] Inerial preintegration factor.
    RETURN_FALSE_IF(!AddImuPreintegrationFactorForMarginalization(idx_offset));

    // Construct graph problem, add all vertices and edges.
    // Add prior information if valid.
    Graph<DorF> graph_optimization_problem;
    ConstructGraphOptimizationProblem(idx_offset, graph_optimization_problem);
    if (states_.prior.is_valid) {
        ReportInfo("[Backend] Before marginalization, prior residual squared norm is " << graph_optimization_problem.prior_residual().squaredNorm());
    }

    // Set vertices to be marged.
    std::vector<Vertex<DorF> *> vertices_to_be_marged = {
        graph_.vertices.all_frames_p_wi.front().get(),
        graph_.vertices.all_frames_q_wi.front().get(),
        graph_.vertices.all_new_frames_v_wi.front().get(),
        graph_.vertices.all_new_frames_ba.front().get(),
        graph_.vertices.all_new_frames_bg.front().get(),
    };

    // Do marginalization.
    Marginalizor<DorF> marger;
    marger.problem() = &graph_optimization_problem;
    marger.options().kSortDirection = SortMargedVerticesDirection::kSortAtBack;
    states_.prior.is_valid = marger.Marginalize(vertices_to_be_marged, states_.prior.is_valid);
    if (options_.kEnableReportAllInformation) {
        marger.problem()->VerticesInformation();
    }

    // Store prior information.
    if (states_.prior.is_valid) {
        states_.prior.hessian = marger.problem()->prior_hessian();
        states_.prior.bias = marger.problem()->prior_bias();
        states_.prior.jacobian_t_inv = marger.problem()->prior_jacobian_t_inv();
        states_.prior.residual = marger.problem()->prior_residual();
    }

    // Report marginalization result.
    ReportInfo("[Backend] Marginalized prior residual size is " << states_.prior.residual.rows() <<
        ", squared norm is " << marger.problem()->prior_residual().squaredNorm() <<
        ", cost of problem is " << marger.cost_of_problem());
    if (options_.kEnableReportAllInformation) {
        ShowMatrixImage("marg hessian", marger.problem()->hessian());
        ShowMatrixImage("reverse hessian", marger.reverse_hessian());
        ShowMatrixImage("prior", marger.problem()->prior_hessian());
    }

    return true;
}

bool Backend::MarginalizeSubnewFrame() {
    ReportInfo("[Bakcend] Backend try to marginalize subnew frame.");

    // Discard relative prior information.
    const uint32_t min_size = 6 * (data_manager_->options().kMaxStoredKeyFrames - data_manager_->options().kMaxStoredNewFrames) +
        6 * data_manager_->camera_extrinsics().size();

    if (min_size == 0) {
        states_.prior.is_valid = false;
        return true;
    }
    RETURN_TRUE_IF_FALSE(states_.prior.is_valid);

    const uint32_t target_size = states_.prior.hessian.cols() > 15 ? std::max(static_cast<uint32_t>(states_.prior.hessian.cols() - 15), min_size) : 0;
    ReportInfo(RED "[Backend] Marginalizor discard prior information. [" << states_.prior.hessian.cols() << "]->[" << target_size << "]" RESET_COLOR);
    if (states_.prior.is_valid && target_size > 0) {
        Marginalizor<DorF> marger;
        // Prior information of frame to be discarded shoule be directly discarded.
        marger.DiscardPriorInformation(states_.prior.hessian, states_.prior.bias, min_size, 15);
        // marger.Marginalize(states_.prior.hessian, states_.prior.bias, min_size, 15);

        // Prior jacobian_t_inv and residual should be decomposed by hessian and bias.
        marger.DecomposeHessianAndBias(states_.prior.hessian, states_.prior.bias,
            states_.prior.jacobian, states_.prior.residual, states_.prior.jacobian_t_inv);
    }

    // Report marginalization result.
    ReportInfo("[Backend] Marginalized prior residual size is " << states_.prior.residual.rows() <<
        ", squared norm is " << states_.prior.residual.squaredNorm());
    if (options_.kEnableReportAllInformation) {
        ShowMatrixImage("prior", states_.prior.hessian);
    }

    return true;
}

}
