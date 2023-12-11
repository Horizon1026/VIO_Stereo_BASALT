#ifndef _VISUAL_EDGES_H_
#define _VISUAL_EDGES_H_

#include "datatype_basic.h"
#include "math_kinematics.h"

#include "vertex.h"
#include "vertex_quaternion.h"
#include "edge.h"
#include "kernel.h"
#include "kernel_huber.h"
#include "kernel_cauchy.h"
#include "kernel_tukey.h"

namespace VIO {

/* Class Edge reprojection. Project feature 3-dof position on visual norm plane. */
template <typename Scalar>
class EdgeFeaturePosToNormPlane : public Edge<Scalar> {
// Vertices are [feature, p_w]
//              [camera, p_wc]
//              [camera, q_wc]

public:
    EdgeFeaturePosToNormPlane() = delete;
    EdgeFeaturePosToNormPlane(int32_t residual_dim, int32_t vertex_num) : Edge<Scalar>(residual_dim, vertex_num) {}
    virtual ~EdgeFeaturePosToNormPlane() = default;

    // Compute residual and jacobians for each vertex. These operations should be defined by subclass.
    virtual void ComputeResidual() override {
        // Compute prediction.
        p_w = this->GetVertex(0)->param();
        p_wc = this->GetVertex(1)->param();
        const TVec4<Scalar> &parameter = this->GetVertex(2)->param();
        q_wc = TQuat<Scalar>(parameter(0), parameter(1), parameter(2), parameter(3));
        p_c = q_wc.inverse() * (p_w - p_wc);
        inv_depth = static_cast<Scalar>(1) / p_c.z();

        // Get observation.
        pixel_norm_xy = this->observation();

        // Compute residual.
        if (std::isinf(inv_depth) || std::isnan(inv_depth)) {
            this->residual().setZero(2);
        } else {
            this->residual() = (p_c.template head<2>() * inv_depth) - pixel_norm_xy;
        }
    }

    virtual void ComputeJacobians() override {
        TMat2x3<Scalar> jacobian_2d_3d = TMat2x3<Scalar>::Zero();
        if (!std::isinf(inv_depth) && !std::isnan(inv_depth)) {
            const Scalar inv_depth_2 = inv_depth * inv_depth;
            jacobian_2d_3d << inv_depth, 0, - p_c(0) * inv_depth_2,
                              0, inv_depth, - p_c(1) * inv_depth_2;
        }

        this->GetJacobian(0) = jacobian_2d_3d * (q_wc.inverse().matrix());
        this->GetJacobian(1) = - this->GetJacobian(0);
        this->GetJacobian(2) = jacobian_2d_3d * SLAM_UTILITY::Utility::Utility::SkewSymmetricMatrix(p_c);
    }

    // Use string to represent edge type.
    virtual std::string GetType() { return std::string("[Edge] Reporject p_w -> norm plane."); }

private:
    // Parameters will be calculated in ComputeResidual().
    // It should not be repeatedly calculated in ComputeJacobians().
    TVec3<Scalar> p_w = TVec3<Scalar>::Zero();
    TVec3<Scalar> p_wc = TVec3<Scalar>::Zero();
    TQuat<Scalar> q_wc = TQuat<Scalar>::Identity();
    TVec2<Scalar> pixel_norm_xy = TVec2<Scalar>::Zero();
    TVec3<Scalar> p_c = TVec3<Scalar>::Zero();
    Scalar inv_depth = 0;
};


/* Class Edge reprojection. Project feature 3-dof position on visual unit sphere. */
template <typename Scalar>
class EdgeFeaturePosToUnitSphere : public Edge<Scalar> {
// Vertices are [feature, p_w]
//              [camera, p_wc]
//              [camera, q_wc]

public:
    EdgeFeaturePosToUnitSphere() = delete;
    EdgeFeaturePosToUnitSphere(int32_t residual_dim, int32_t vertex_num) : Edge<Scalar>(residual_dim, vertex_num) {}
    virtual ~EdgeFeaturePosToUnitSphere() = default;

    // Compute residual and jacobians for each vertex. These operations should be defined by subclass.
    virtual void ComputeResidual() override {
        // Compute prediction.
        p_w = this->GetVertex(0)->param();
        p_wc = this->GetVertex(1)->param();
        const TVec4<Scalar> &parameter = this->GetVertex(2)->param();
        q_wc = TQuat<Scalar>(parameter(0), parameter(1), parameter(2), parameter(3));
        p_c = q_wc.inverse() * (p_w - p_wc);

        // Get observation.
        obv_norm_xy = this->observation();
        const TVec3<Scalar> obv_p_c = TVec3<Scalar>(obv_norm_xy.x(), obv_norm_xy.y(), 1.0f);

        // Compute residual.
        this->residual() = tangent_base_transpose * (p_c.normalized() - obv_p_c.normalized());
    }

    virtual void ComputeJacobians() override {
        const Scalar p_c_norm = p_c.norm();
        const Scalar p_c_norm3 = p_c_norm * p_c_norm * p_c_norm;
        TMat3<Scalar> jacobian_norm = TMat3<Scalar>::Zero();
        jacobian_norm << 1.0 / p_c_norm - p_c.x() * p_c.x() / p_c_norm3, - p_c.x() * p_c.y() / p_c_norm3,                - p_c.x() * p_c.z() / p_c_norm3,
                         - p_c.x() * p_c.y() / p_c_norm3,                1.0 / p_c_norm - p_c.y() * p_c.y() / p_c_norm3, - p_c.y() * p_c.z() / p_c_norm3,
                         - p_c.x() * p_c.z() / p_c_norm3,                - p_c.y() * p_c.z() / p_c_norm3,                1.0 / p_c_norm - p_c.z() * p_c.z() / p_c_norm3;

        TMat2x3<Scalar> jacobian_2d_3d = TMat2x3<Scalar>::Zero();
        jacobian_2d_3d = tangent_base_transpose * jacobian_norm;

        this->GetJacobian(0) = jacobian_2d_3d * (q_wc.inverse().matrix());
        this->GetJacobian(1) = - this->GetJacobian(0);
        this->GetJacobian(2) = jacobian_2d_3d * SLAM_UTILITY::Utility::SkewSymmetricMatrix(p_c);
    }

    // Use string to represent edge type.
    virtual std::string GetType() { return std::string("[Edge] Reporject p_w -> unit sphere."); }

    // Set tangent base.
    void SetTrangetBase(const TVec3<Scalar> &vec) {
        tangent_base_transpose = Utility::TangentBase(vec).transpose();
    }

private:
    // Parameters will be calculated in ComputeResidual().
    // It should not be repeatedly calculated in ComputeJacobians().
    TVec3<Scalar> p_w = TVec3<Scalar>::Zero();
    TVec3<Scalar> p_wc = TVec3<Scalar>::Zero();
    TQuat<Scalar> q_wc = TQuat<Scalar>::Identity();
    TVec2<Scalar> obv_norm_xy = TVec2<Scalar>::Zero();
    TVec3<Scalar> p_c = TVec3<Scalar>::Zero();
    TMat2x3<Scalar> tangent_base_transpose;
};

/* Class Edge reprojection. Project feature 1-dof invdep on visual norm plane. */
template <typename Scalar>
class EdgeFeatureInvdepToNormPlane : public Edge<Scalar> {
// Vertices are [feature, invdep]
//              [first camera, p_wc0]
//              [first camera, q_wc0]
//              [camera, p_wc]
//              [camera, q_wc]

public:
    EdgeFeatureInvdepToNormPlane() = delete;
    EdgeFeatureInvdepToNormPlane(int32_t residual_dim, int32_t vertex_num) : Edge<Scalar>(residual_dim, vertex_num) {}
    virtual ~EdgeFeatureInvdepToNormPlane() = default;

    // Compute residual and jacobians for each vertex. These operations should be defined by subclass.
    virtual void ComputeResidual() override {
        inv_depth0 = this->GetVertex(0)->param()(0);
        p_wc0 = this->GetVertex(1)->param();
        const TVec4<Scalar> &param_i = this->GetVertex(2)->param();
        q_wc0 = TQuat<Scalar>(param_i(0), param_i(1), param_i(2), param_i(3));
        p_wc = this->GetVertex(3)->param();
        const TVec4<Scalar> &param_j = this->GetVertex(4)->param();
        q_wc = TQuat<Scalar>(param_j(0), param_j(1), param_j(2), param_j(3));

        norm_xy0 = this->observation().block(0, 0, 2, 1);
        norm_xy = this->observation().block(2, 0, 2, 1);

        p_c0 = TVec3<Scalar>(norm_xy0(0), norm_xy0(1), static_cast<Scalar>(1)) / inv_depth0;
        p_w = q_wc0 * p_c0 + p_wc0;
        p_c = q_wc.inverse() * (p_w - p_wc);
        inv_depth = static_cast<Scalar>(1) / p_c.z();

        if (std::isinf(inv_depth) || std::isnan(inv_depth)) {
            this->residual().setZero(2);
        } else {
            this->residual() = (p_c.template head<2>() * inv_depth) - norm_xy;
        }
    }

    virtual void ComputeJacobians() override {
        TMat2x3<Scalar> jacobian_2d_3d = TMat2x3<Scalar>::Zero();
        if (!std::isinf(inv_depth) && !std::isnan(inv_depth)) {
            const Scalar inv_depth_2 = inv_depth * inv_depth;
            jacobian_2d_3d << inv_depth, 0, - p_c(0) * inv_depth_2,
                              0, inv_depth, - p_c(1) * inv_depth_2;
        }

        const TMat3<Scalar> R_cw = q_wc.toRotationMatrix().transpose();
        const TMat3<Scalar> R_cc0 = R_cw * q_wc0.matrix();

        const TMat3<Scalar> jacobian_cam0_q = - R_cc0 * Utility::SkewSymmetricMatrix(p_c0);
        const TMat3<Scalar> jacobian_cam0_p = R_cw;

        const TMat3<Scalar> jacobian_cam_q = Utility::SkewSymmetricMatrix(p_c);
        const TMat3<Scalar> jacobian_cam_p = - R_cw;

        const TVec3<Scalar> jacobian_invdep = - R_cc0 *
            TVec3<Scalar>(norm_xy0(0), norm_xy0(1), static_cast<Scalar>(1)) / (inv_depth0 * inv_depth0);

        this->GetJacobian(0) = jacobian_2d_3d * jacobian_invdep;
        this->GetJacobian(1) = jacobian_2d_3d * jacobian_cam0_p;
        this->GetJacobian(2) = jacobian_2d_3d * jacobian_cam0_q;
        this->GetJacobian(3) = jacobian_2d_3d * jacobian_cam_p;
        this->GetJacobian(4) = jacobian_2d_3d * jacobian_cam_q;
    }

    // Use string to represent edge type.
    virtual std::string GetType() { return std::string("Edge Reprojection"); }

private:
    // Parameters will be calculated in ComputeResidual().
    // It should not be repeatedly calculated in ComputeJacobians().
    TVec3<Scalar> p_wc0 = TVec3<Scalar>::Zero();
    TQuat<Scalar> q_wc0 = TQuat<Scalar>::Identity();
    TVec2<Scalar> norm_xy0 = TVec2<Scalar>::Zero();
    Scalar inv_depth0 = 0;
    TVec3<Scalar> p_c0 = TVec3<Scalar>::Zero();

    TVec3<Scalar> p_wc = TVec3<Scalar>::Zero();
    TQuat<Scalar> q_wc = TQuat<Scalar>::Identity();
    TVec2<Scalar> norm_xy = TVec2<Scalar>::Zero();
    Scalar inv_depth = 0;
    TVec3<Scalar> p_c = TVec3<Scalar>::Zero();

    TVec3<Scalar> p_w = TVec3<Scalar>::Zero();
};

}

#endif // end of _VISUAL_EDGES_H_
