#ifndef _VISUAL_INERTIAL_EDGES_H_
#define _VISUAL_INERTIAL_EDGES_H_

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

/* Class Edge reprojection. Project feature 1-dof invdep on visual norm plane via imu pose. */
template <typename Scalar>
class EdgeFeatureInvdepToNormPlaneViaImu : public Edge<Scalar> {
// Vertices are [feature, invdep]
//              [first imu pose, p_wi0]
//              [first imu pose, q_wi0]
//              [imu pose, p_wi]
//              [imu pose, q_wi]
//              [extrinsic, p_ic]
//              [extrinsic, q_ic]

public:
    EdgeFeatureInvdepToNormPlaneViaImu() = delete;
    EdgeFeatureInvdepToNormPlaneViaImu(int32_t residual_dim, int32_t vertex_num) : Edge<Scalar>(residual_dim, vertex_num) {}
    virtual ~EdgeFeatureInvdepToNormPlaneViaImu() = default;

    // Compute residual and jacobians for each vertex. These operations should be defined by subclass.
    virtual void ComputeResidual() override {
        // Extract states.
        inv_depth0 = this->GetVertex(0)->param()(0);
        p_wi0 = this->GetVertex(1)->param();
        const TVec4<Scalar> &param_i0 = this->GetVertex(2)->param();
        q_wi0 = TQuat<Scalar>(param_i0(0), param_i0(1), param_i0(2), param_i0(3));
        p_wi = this->GetVertex(3)->param();
        const TVec4<Scalar> &param_i = this->GetVertex(4)->param();
        q_wi = TQuat<Scalar>(param_i(0), param_i(1), param_i(2), param_i(3));
        p_ic = this->GetVertex(5)->param();
        const TVec4<Scalar> &param_ic = this->GetVertex(6)->param();
        q_ic = TQuat<Scalar>(param_ic(0), param_ic(1), param_ic(2), param_ic(3));

        // Extract observations.
        norm_xy0 = this->observation().block(0, 0, 2, 1);
        norm_xy = this->observation().block(2, 0, 2, 1);

        // Compute projection.
        p_c0 = TVec3<Scalar>(norm_xy0(0), norm_xy0(1), static_cast<Scalar>(1)) / inv_depth0;
        p_i0 = q_ic * p_c0 + p_ic;
        p_w = q_wi0 * p_i0 + p_wi0;
        p_i = q_wi.inverse() * (p_w - p_wi);
        p_c = q_ic.inverse() * (p_i - p_ic);
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

        const TMat3<Scalar> jacobian_cam0_p = q_ic.inverse() * q_wi.inverse();
        const TMat3<Scalar> jacobian_cam0_q = - q_ic.inverse() * q_wi.inverse() * q_wi0 * Utility::SkewSymmetricMatrix(p_i0);

        const TMat3<Scalar> jacobian_cam_p = - q_ic.inverse() * q_wi.inverse();
        const TMat3<Scalar> jacobian_cam_q = q_ic.inverse() * Utility::SkewSymmetricMatrix(p_i);

        const TVec3<Scalar> jacobian_invdep = - q_ic.inverse() * q_wi.inverse() * q_wi0 * q_ic *
            TVec3<Scalar>(norm_xy0.x(), norm_xy0.y(), static_cast<Scalar>(1)) / (inv_depth0 * inv_depth0);

        const TMat3<Scalar> temp = q_ic.inverse() * q_wi.inverse() * q_wi0 * q_ic;
        const TMat3<Scalar> jacobian_ex_p = q_ic.inverse() * (q_wi.inverse() * q_wi0 - TMat3<Scalar>::Identity());
        const TMat3<Scalar> jacobian_ex_q = - temp * Utility::SkewSymmetricMatrix(p_c0) + Utility::SkewSymmetricMatrix(temp * p_c0) +
                Utility::SkewSymmetricMatrix(q_ic.inverse() * (q_wi.inverse() * (q_wi0 * p_ic + p_wi0 - p_wi) - p_ic));

        this->GetJacobian(0) = jacobian_2d_3d * jacobian_invdep;
        this->GetJacobian(1) = jacobian_2d_3d * jacobian_cam0_p;
        this->GetJacobian(2) = jacobian_2d_3d * jacobian_cam0_q;
        this->GetJacobian(3) = jacobian_2d_3d * jacobian_cam_p;
        this->GetJacobian(4) = jacobian_2d_3d * jacobian_cam_q;
        this->GetJacobian(5) = jacobian_2d_3d * jacobian_ex_p;
        this->GetJacobian(6) = jacobian_2d_3d * jacobian_ex_q;
    }

    // Use string to represent edge type.
    virtual std::string GetType() { return std::string("Edge Reprojection"); }

private:
    // Parameters will be calculated in ComputeResidual().
    // It should not be repeatedly calculated in ComputeJacobians().
    TVec3<Scalar> p_wc0 = TVec3<Scalar>::Zero();
    TQuat<Scalar> q_wc0 = TQuat<Scalar>::Identity();
    TVec3<Scalar> p_wi0 = TVec3<Scalar>::Zero();
    TQuat<Scalar> q_wi0 = TQuat<Scalar>::Identity();
    TVec2<Scalar> norm_xy0 = TVec2<Scalar>::Zero();
    Scalar inv_depth0 = 0;
    TVec3<Scalar> p_i0 = TVec3<Scalar>::Zero();
    TVec3<Scalar> p_c0 = TVec3<Scalar>::Zero();

    TVec3<Scalar> p_wc = TVec3<Scalar>::Zero();
    TQuat<Scalar> q_wc = TQuat<Scalar>::Identity();
    TVec3<Scalar> p_wi = TVec3<Scalar>::Zero();
    TQuat<Scalar> q_wi = TQuat<Scalar>::Identity();
    TVec2<Scalar> norm_xy = TVec2<Scalar>::Zero();
    Scalar inv_depth = 0;
    TVec3<Scalar> p_i = TVec3<Scalar>::Zero();
    TVec3<Scalar> p_c = TVec3<Scalar>::Zero();

    TVec3<Scalar> p_w = TVec3<Scalar>::Zero();

    TVec3<Scalar> p_ic = TVec3<Scalar>::Zero();
    TQuat<Scalar> q_ic = TQuat<Scalar>::Identity();
};

}

#endif // end of _VISUAL_INERTIAL_EDGES_H_
