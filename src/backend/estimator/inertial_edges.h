#ifndef _INERTIAL_EDGES_H_
#define _INERTIAL_EDGES_H_

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
    EdgeImuPreintegrationBetweenRelativePose() : Edge<Scalar>(15, 10) {}
    virtual ~EdgeImuPreintegrationBetweenRelativePose() = default;

    // Compute residual and jacobians for each vertex. These operations should be defined by subclass.
    virtual void ComputeResidual() override {
        // Extract states.
        p_wi0 = this->GetVertex(0)->param();
        const TVec4<Scalar> &param_i0 = this->GetVertex(1)->param();
        q_wi0 = TQuat<Scalar>(param_i0(0), param_i0(1), param_i0(2), param_i0(3));
        v_wi0 = this->GetVertex(2)->param();
        bias_a0 = this->GetVertex(3)->param();
        bias_g0 = this->GetVertex(4)->param();
        p_wi1 = this->GetVertex(5)->param();
        const TVec4<Scalar> &param_i1 = this->GetVertex(6)->param();
        q_wi1 = TQuat<Scalar>(param_i1(0), param_i1(1), param_i1(2), param_i1(3));
        v_wi1 = this->GetVertex(7)->param();
        bias_a1 = this->GetVertex(8)->param();
        bias_g1 = this->GetVertex(9)->param();

        // Extract observations.

    }

    virtual void ComputeJacobians() override {

    }

    // Use string to represent edge type.
    virtual std::string GetType() { return std::string("Edge Reprojection"); }

private:
    // Parameters will be calculated in ComputeResidual().
    // It should not be repeatedly calculated in ComputeJacobians().
    TVec3<Scalar> p_wi0 = TVec3<Scalar>::Zero();
    TQuat<Scalar> q_wi0 = TQuat<Scalar>::Identity();
    TVec3<Scalar> v_wi0 = TVec3<Scalar>::Zero();
    TVec3<Scalar> bias_a0 = TVec3<Scalar>::Zero();
    TVec3<Scalar> bias_g0 = TVec3<Scalar>::Zero();

    TVec3<Scalar> p_wi1 = TVec3<Scalar>::Zero();
    TQuat<Scalar> q_wi1 = TQuat<Scalar>::Identity();
    TVec3<Scalar> v_wi1 = TVec3<Scalar>::Zero();
    TVec3<Scalar> bias_a1 = TVec3<Scalar>::Zero();
    TVec3<Scalar> bias_g1 = TVec3<Scalar>::Zero();
};

}

#endif // end of _INERTIAL_EDGES_H_
