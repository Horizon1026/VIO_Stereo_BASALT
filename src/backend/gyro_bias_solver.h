#ifndef _GYRO_BIAS_SOLVER_H_
#define _GYRO_BIAS_SOLVER_H_

#include "eigen_optimization_functor.h"
#include "eigen3/unsupported/Eigen/NonLinearOptimization"
#include "eigen3/unsupported/Eigen/NumericalDiff"

#include "relative_rotation.h"
#include "math_kinematics.h"

namespace VIO {

using namespace VISION_GEOMETRY;

/* Gyro Bias Solver Step Definition. */
struct GyroBiasSolveStep : OptimizationFunctor<float> {
    const SummationTerms &terms_;
    const Mat3 &dr_dbg_;

    GyroBiasSolveStep(const SummationTerms &terms, const Mat3 &dr_dbg) :
        OptimizationFunctor<float>(1, 3),
        terms_(terms), dr_dbg_(dr_dbg) {}

    int operator()(const Vec &x, Vec &fvec) const {
        const Vec3 bias_gyro = x;
        const Vec3 delta_rot = dr_dbg_ * bias_gyro;
        const Quat delta_q = Utility::ConvertAngleAxisToQuaternion(delta_rot);
        const Vec3 cayley = Utility::ConvertQuaternionToCayley(delta_q);

        Mat1x3 dlambda_dcayley = Mat1x3::Zero();
        fvec[0] = RelativeRotation::ComputeSmallestEigenValueAndJacobian(terms_, cayley, dlambda_dcayley);

        return 0;
    }
};

}

#endif // end of _GYRO_BIAS_SOLVER_H_
