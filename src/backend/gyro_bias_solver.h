#ifndef _GYRO_BIAS_SOLVER_H_
#define _GYRO_BIAS_SOLVER_H_

#include "eigen_optimization_functor.h"
#include "relative_rotation.h"

namespace VIO {

/* Gyro Bias Solver Step Definition. */
struct GyroBiasSolveStep : OptimizationFunctor<float> {
    const SummationTerms &terms_;
    const Mat3 &dr_dbg_;

    GyroBiasSolveStep(const SummationTerms &terms, const Mat3 &dr_dbg) :
        OptimizationFunctor<float>(3, 3),
        terms_(terms), dr_dbg_(dr_dbg) {}

    int operator()(const Vec &x, Vec &fvec) const {
        Vec3 cayley = x;
        Mat1x3 jacobian = Mat1x3::Zero();

        RelativeRotation::ComputeSmallestEigenValueAndJacobian(terms_, cayley, jacobian);

        fvec[0] = jacobian(0, 0);
        fvec[1] = jacobian(0, 1);
        fvec[2] = jacobian(0, 2);

        return 0;
    }
};

}

#endif // end of _GYRO_BIAS_SOLVER_H_
