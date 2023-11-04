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
        OptimizationFunctor<float>(3, 3),
        terms_(terms), dr_dbg_(dr_dbg) {}

    int operator()(const Vec &x, Vec &fvec) const {
        const Vec3 bias_gyro = x;
        const Vec3 delta_rot = dr_dbg_ * bias_gyro;
        const Quat q = Utility::Exponent(delta_rot);
        const Vec3 cayley = q.vec() / q.w();

        Mat1x3 dlambda_dcayley = Mat1x3::Zero();
        RelativeRotation::ComputeSmallestEigenValueAndJacobian(terms_, cayley, dlambda_dcayley);

        const float norm = delta_rot.norm();
        const float temp1 = std::tan(0.5f * norm) / norm;
        const float temp2 = std::cos(0.5f * norm);
        const float temp3 = 1.0f / (2.0f * norm * temp2 * temp2) - temp1 / norm;
        const float temp4 = delta_rot.squaredNorm() / norm;
        const Mat3 dcayley_dbg = (temp1 + temp3 * temp4) * dr_dbg_;

        const Mat1x3 dlambda_dbg = dlambda_dcayley * dcayley_dbg;
        fvec[0] = dlambda_dbg(0, 0);
        fvec[1] = dlambda_dbg(0, 1);
        fvec[2] = dlambda_dbg(0, 2);

        return 0;
    }
};

}

#endif // end of _GYRO_BIAS_SOLVER_H_
