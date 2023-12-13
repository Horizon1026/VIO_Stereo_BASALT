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
// Vertices are [feature, invdep]
//              [extrinsic, p_ic0]
//              [extrinsic, q_ic0]
//              [extrinsic, p_ic]
//              [extrinsic, q_ic]

public:
    EdgeImuPreintegrationBetweenRelativePose() : Edge<Scalar>(15, 6) {}
    virtual ~EdgeImuPreintegrationBetweenRelativePose() = default;

    // Compute residual and jacobians for each vertex. These operations should be defined by subclass.
    virtual void ComputeResidual() override {

    }

    virtual void ComputeJacobians() override {

    }

    // Use string to represent edge type.
    virtual std::string GetType() { return std::string("Edge Reprojection"); }

private:
    // Parameters will be calculated in ComputeResidual().
    // It should not be repeatedly calculated in ComputeJacobians().

};

}

#endif // end of _INERTIAL_EDGES_H_
