#ifndef _INERTIAL_EDGES_H_
#define _INERTIAL_EDGES_H_

#include "datatype_basic.h"
#include "math_kinematics.h"

#include "imu_state.h"
#include "imu_preintegrate.h"

#include "vertex.h"
#include "vertex_quaternion.h"
#include "edge.h"
#include "kernel.h"
#include "kernel_huber.h"
#include "kernel_cauchy.h"
#include "kernel_tukey.h"

namespace VIO {

/* Class Edge reprojection. Align imu preintegration between two imu pose. */
template <typename Scalar>
class EdgeImuPreintegrationBetweenRelativePose : public Edge<Scalar> {
// Vertices are [imu pose 0, p_wi0]
//              [imu pose 0, q_wi0]
//              [imu vel 0, v_wi0]
//              [imu bias_a 0, bias_a0]
//              [imu bias_g 0, bias_g0]
//              [imu pose 1, p_wi1]
//              [imu pose 1, q_wi1]
//              [imu vel 1, v_wi1]
//              [imu bias_a 1, bias_a1]
//              [imu bias_g 1, bias_g1]

public:
    EdgeImuPreintegrationBetweenRelativePose(const ImuPreintegrateBlock &imu_block,
                                             const Vec3 &gravity_w) : Edge<Scalar>(15, 10) {
        linear_point_.p_ij = imu_block.p_ij().cast<Scalar>();
        linear_point_.q_ij = imu_block.q_ij().cast<Scalar>();
        linear_point_.v_ij = imu_block.v_ij().cast<Scalar>();
        linear_point_.bias_a = imu_block.bias_accel().cast<Scalar>();
        linear_point_.bias_g = imu_block.bias_gyro().cast<Scalar>();
        imu_jacobians_.dp_dba = imu_block.dp_dba().cast<Scalar>();
        imu_jacobians_.dp_dbg = imu_block.dp_dbg().cast<Scalar>();
        imu_jacobians_.dr_dbg = imu_block.dr_dbg().cast<Scalar>();
        imu_jacobians_.dv_dba = imu_block.dv_dba().cast<Scalar>();
        imu_jacobians_.dv_dbg = imu_block.dv_dbg().cast<Scalar>();
        gravity_w_ = gravity_w.cast<Scalar>();
        integrate_time_s_ = static_cast<Scalar>(imu_block.integrate_time_s());
        this->information() = imu_block.covariance().inverse().cast<Scalar>();
    }
    virtual ~EdgeImuPreintegrationBetweenRelativePose() = default;

    // Compute residual and jacobians for each vertex. These operations should be defined by subclass.
    virtual void ComputeResidual() override {
        // Extract states.
        p_wi0_ = this->GetVertex(0)->param();
        const TVec4<Scalar> &param_i0 = this->GetVertex(1)->param();
        q_wi0_ = TQuat<Scalar>(param_i0(0), param_i0(1), param_i0(2), param_i0(3));
        v_wi0_ = this->GetVertex(2)->param();
        bias_a0_ = this->GetVertex(3)->param();
        bias_g0_ = this->GetVertex(4)->param();
        p_wi1_ = this->GetVertex(5)->param();
        const TVec4<Scalar> &param_i1 = this->GetVertex(6)->param();
        q_wi1_ = TQuat<Scalar>(param_i1(0), param_i1(1), param_i1(2), param_i1(3));
        v_wi1_ = this->GetVertex(7)->param();
        bias_a1_ = this->GetVertex(8)->param();
        bias_g1_ = this->GetVertex(9)->param();

        // Extract observations.
        const Scalar &dt = integrate_time_s_;
        CorrectObservation(bias_a0_, bias_g0_);

        // Compute residual.
        this->residual().setZero(15);
        this->residual().block<3, 1>(ImuIndex::kPosition, 0) = q_wi0_.inverse() * (p_wi1_ - p_wi0_ - v_wi0_ * dt + static_cast<Scalar>(0.5) * gravity_w_ * dt * dt) - p_ij_;
        this->residual().block<3, 1>(ImuIndex::kRotation, 0) = static_cast<Scalar>(2) * (q_ij_.inverse() * (q_wi0_.inverse() * q_wi1_)).vec();
        this->residual().block<3, 1>(ImuIndex::kVelocity, 0) = q_wi0_.inverse() * (v_wi1_ - v_wi0_ + gravity_w_ * dt) - v_ij_;
        this->residual().block<3, 1>(ImuIndex::kBiasAccel, 0) = bias_a1_ - bias_a0_;
        this->residual().block<3, 1>(ImuIndex::kBiasGyro, 0) = bias_g1_ - bias_g0_;
    }

    virtual void ComputeJacobians() override {
        // Compute dr d_state0.
        const TMat3<Scalar> dp_dp0 = - q_wi0_.inverse().toRotationMatrix();
        const TMat3<Scalar> dp_dq0 = SkewSymmetricMatrix(q_wb_i.inverse() * (0.5 * this->gravity_w * dt2 + p_wb_j - p_wb_i - v_wb_i * dt));
        const TMat3<Scalar> dq_dq0 = - (Qleft(q_wb_j.inverse() * q_wb_i) * Qright(delta_r)).template bottomRightCorner<3, 3>();
        const TMat3<Scalar> dv_dq0 = SkewSymmetricMatrix(q_wb_i.inverse() * (this->gravity_w * dt + v_wb_j - v_wb_i));

        const TMat3<Scalar> dp_dv0 = - R_wb_i.transpose() * dt;
        const TMat3<Scalar> dp_dba0 = - this->jacobians.dp_dba;
        const TMat3<Scalar> dp_dbg0 = - this->jacobians.dp_dbg;
        const TMat3<Scalar> dq_dbg0 = - Qleft(q_wb_j.inverse() * q_wb_i * delta_r).template bottomRightCorner<3, 3>() * this->jacobians.dr_dbg;
        const TMat3<Scalar> dv_dv0 = - R_wb_i.transpose();
        const TMat3<Scalar> dv_dba0 = - this->jacobians.dv_dba;
        const TMat3<Scalar> dv_dbg0 = - this->jacobians.dv_dbg;
        const TMat3<Scalar> dba_dba0 = - Matrix3<Scalar>::Identity();
        const TMat3<Scalar> dbg_dbg0 = - Matrix3<Scalar>::Identity();

        // Compute dr d_state1.
        const TMat3<Scalar> dp_dp1 = R_wb_i.transpose();
        const TMat3<Scalar> dq_dq1 = Qleft(delta_r.inverse() * q_wb_i.inverse() * q_wb_j).template bottomRightCorner<3, 3>();

        const TMat3<Scalar> dv_dv1 = R_wb_i.transpose();
        const TMat3<Scalar> dba_dba1 = Matrix3<Scalar>::Identity();
        const TMat3<Scalar> dbg_dbg1 = Matrix3<Scalar>::Identity();
    }

    void CorrectObservation(const TVec3<Scalar> &bias_a,
                            const TVec3<Scalar> &bias_g) {
        const TVec3<Scalar> dba = bias_a - linear_point_.bias_a;
        const TVec3<Scalar> dbg = bias_g - linear_point_.bias_g;
        p_ij_ = linear_point_.p_ij + imu_jacobians_.dp_dba * dba + imu_jacobians_.dp_dbg * dbg;
        q_ij_ = linear_point_.q_ij * Utility::DeltaQ(imu_jacobians_.dr_dbg * dbg);
        v_ij_ = linear_point_.v_ij + imu_jacobians_.dv_dba * dba + imu_jacobians_.dv_dbg * dbg;
    }

    // Use string to represent edge type.
    virtual std::string GetType() { return std::string("Edge Reprojection"); }

private:
    // Parameters will be calculated in ComputeResidual().
    // It should not be repeatedly calculated in ComputeJacobians().
    TVec3<Scalar> p_wi0_ = TVec3<Scalar>::Zero();
    TQuat<Scalar> q_wi0_ = TQuat<Scalar>::Identity();
    TVec3<Scalar> v_wi0_ = TVec3<Scalar>::Zero();
    TVec3<Scalar> bias_a0_ = TVec3<Scalar>::Zero();
    TVec3<Scalar> bias_g0_ = TVec3<Scalar>::Zero();

    TVec3<Scalar> p_wi1_ = TVec3<Scalar>::Zero();
    TQuat<Scalar> q_wi1_ = TQuat<Scalar>::Identity();
    TVec3<Scalar> v_wi1_ = TVec3<Scalar>::Zero();
    TVec3<Scalar> bias_a1_ = TVec3<Scalar>::Zero();
    TVec3<Scalar> bias_g1_ = TVec3<Scalar>::Zero();

    TVec3<Scalar> p_ij_ = TVec3<Scalar>::Zero();
    TQuat<Scalar> q_ij_ = TQuat<Scalar>::Identity();
    TVec3<Scalar> v_ij_ = TVec3<Scalar>::Zero();

    // Imu observations.
    struct LineralizedPoint {
        TVec3<Scalar> p_ij = TVec3<Scalar>::Zero();
        TQuat<Scalar> q_ij = TQuat<Scalar>::Identity();
        TVec3<Scalar> v_ij = TVec3<Scalar>::Zero();
        TVec3<Scalar> bias_a = TVec3<Scalar>::Zero();
        TVec3<Scalar> bias_g = TVec3<Scalar>::Zero();
    } linear_point_;
    struct ImuJacobians {
        TMat3<Scalar> dp_dbg = TMat3<Scalar>::Identity();
        TMat3<Scalar> dp_dba = TMat3<Scalar>::Identity();
        TMat3<Scalar> dr_dbg = TMat3<Scalar>::Identity();
        TMat3<Scalar> dv_dbg = TMat3<Scalar>::Identity();
        TMat3<Scalar> dv_dba = TMat3<Scalar>::Identity();
    } imu_jacobians_;
    TVec3<Scalar> gravity_w_ = TVec3<Scalar>(0, 0, 9.8);
    Scalar integrate_time_s_ = static_cast<Scalar>(0);

};

}

#endif // end of _INERTIAL_EDGES_H_
