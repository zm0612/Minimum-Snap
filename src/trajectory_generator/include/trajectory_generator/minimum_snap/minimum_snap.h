//
// Created by Zhang Zhimeng on 2021/11/29.
//

#ifndef TRAJECTORY_GENERATOR_MINIMUM_SNAP_H
#define TRAJECTORY_GENERATOR_MINIMUM_SNAP_H

#include "trajectory_generator/utility/type.h"
#include "trajectory_generator/utility/math_function.h"

#include <Eigen/Dense>

// refer to: 《Polynomial Trajectory Planning for Aggressive
//             Quadrotor Flight in Dense Indoor Environments》
class MinimumSnap {
public:
    MinimumSnap(unsigned int order, double max_vel, double max_accel);

    MinimumSnap() = delete;

    ~MinimumSnap() = default;

    VecXd AllocateTime(const MatXd &waypoint) const;

    MatXd SolveQPClosedForm(const VecXd &waypoints, const VecXd &vel,
                            const VecXd &acc, const VecXd &time) const;

    unsigned int GetPolyCoeffNum() const;

private:
    const unsigned int order_;// =2: accel | =3: jerk | =4: snap
    const double max_vel_;
    const double max_accel_;
};

#endif //TRAJECTORY_GENERATOR_MINIMUM_SNAP_H