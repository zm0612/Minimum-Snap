//
// Created by Zhang Zhimeng on 2021/11/29.
//

#include "trajectory_generator/minimum_snap/minimum_snap.h"

#include <glog/logging.h>

MinimumSnap::MinimumSnap(unsigned int order, double max_vel, double max_accel)
        : order_(order), max_vel_(max_vel), max_accel_(max_accel) {}

VecXd MinimumSnap::AllocateTime(const MatXd &waypoint) const {
    VecXd times = VecXd::Zero(waypoint.rows() - 1);
    const double t = max_vel_ / max_accel_;
    const double dist_threshold_1 = max_accel_ * t * t;

    double segment_t;
    for (unsigned int i = 1; i < waypoint.rows(); ++i) {
        double delta_dist = (waypoint.row(i) - waypoint.row(i - 1)).norm();
        if (delta_dist > dist_threshold_1) {
            segment_t = t * 2 + (delta_dist - dist_threshold_1) / max_vel_;
        } else {
            segment_t = std::sqrt(delta_dist / max_accel_);
        }

        times[i - 1] = segment_t;
    }

    return times;
}

unsigned int MinimumSnap::GetPolyCoeffNum() const {
    return 2u * order_;
}

MatXd MinimumSnap::SolveQPClosedForm(const VecXd &waypoints_pos, const VecXd &waypoints_vel,
                                     const VecXd &waypoints_acc, const VecXd &segments_time) const {
    CHECK_EQ(waypoints_pos.size(), waypoints_vel.size());
    CHECK_EQ(waypoints_vel.size(), waypoints_acc.size());
    CHECK_EQ(waypoints_pos.size(), segments_time.size() + 1u);

    const unsigned int poly_order = 2 * order_ - 1;
    const unsigned int num_poly_coeff = poly_order + 1;
    const unsigned int num_segments = segments_time.size();

    const unsigned int num_all_poly_coeff = num_poly_coeff * num_segments;

    const unsigned int A_block_rows = 2u * order_;
    const unsigned int A_block_cols = num_poly_coeff;
    MatXd A = MatXd::Zero(num_segments * A_block_rows, num_segments * A_block_cols);

    for (unsigned int i = 0; i < num_segments; ++i) {
        unsigned int row = i * A_block_rows;
        unsigned int col = i * A_block_cols;

        MatXd sub_A = MatXd::Zero(A_block_rows, A_block_cols);

        for (unsigned int j = 0; j < order_; ++j) {
            for (unsigned int k = 0; k < num_poly_coeff; ++k) {
                if (k < j) {
                    continue;
                }

                sub_A(j, num_poly_coeff - 1 - k) = Factorial(k) / Factorial(k - j) * std::pow(0, k - j);
                sub_A(j + order_, num_poly_coeff - 1 - k) =
                        Factorial(k) / Factorial(k - j) * pow(segments_time[i], k - j);
            }
        }

        A.block(row, col, A_block_rows, A_block_cols) = sub_A;
    }

    const unsigned int num_valid_variables = (num_segments + 1) * order_;
    const unsigned int num_fixed_variables = 2u * order_ + (num_segments - 1);

    MatXd C_T = MatXd::Zero(num_all_poly_coeff, num_valid_variables);
    for (unsigned int i = 0; i < num_all_poly_coeff; ++i) {
        if (i < order_) {
            C_T(i, i) = 1.0;
            continue;
        }

        if (i >= num_all_poly_coeff - order_) {
            const unsigned int delta_index = i - (num_all_poly_coeff - order_);
            C_T(i, num_fixed_variables - order_ + delta_index) = 1.0;
            continue;
        }

        if ((i % order_ == 0u) && (i / order_ % 2u == 1u)) {
            const unsigned int index = i / (2u * order_) + order_;
            C_T(i, index) = 1.0;
            continue;
        }

        if ((i % order_ == 0u) && (i / order_ % 2u == 0u)) {
            const unsigned int index = i / (2u * order_) + order_ - 1u;
            C_T(i, index) = 1.0;
            continue;
        }

        if ((i % order_ != 0u) && (i / order_ % 2u == 1u)) {
            const unsigned int temp_index_0 = i / (2 * order_) * (2 * order_) + order_;
            const unsigned int temp_index_1 = i / (2 * order_) * (order_ - 1) + i - temp_index_0 - 1;
            C_T(i, num_fixed_variables + temp_index_1) = 1.0;
            continue;
        }

        if ((i % order_ != 0u) && (i / order_ % 2u == 0u)) {
            const unsigned int temp_index_0 = (i - order_) / (2 * order_) * (2 * order_) + order_;
            const unsigned int temp_index_1 =
                    (i - order_) / (2 * order_) * (order_ - 1) + (i - order_) - temp_index_0 - 1;
            C_T(i, num_fixed_variables + temp_index_1) = 1.0;
            continue;
        }
    }

    MatXd Q = MatXd::Zero(num_all_poly_coeff, num_all_poly_coeff);
    for (unsigned int k = 0u; k < num_segments; ++k) {
        MatXd sub_Q = MatXd::Zero(num_poly_coeff, num_poly_coeff);
        for (unsigned int i = 0u; i <= poly_order; ++i) {
            for (unsigned int l = 0u; l <= poly_order; ++l) {
                if (num_poly_coeff - i <= order_ || num_poly_coeff - l <= order_) {
                    continue;
                }

                sub_Q(i, l) = (Factorial(poly_order - i) / Factorial(poly_order - order_ - i)) *
                              (Factorial(poly_order - l) / Factorial(poly_order - order_ - l)) /
                              (poly_order - i + poly_order - l - (2 * order_ - 1)) *
                              std::pow(segments_time[k], poly_order - i + poly_order - l - (2 * order_ - 1));
            }
        }

        const unsigned int row = k * num_poly_coeff;
        Q.block(row, row, num_poly_coeff, num_poly_coeff) = sub_Q;
    }

    MatXd R = C_T.transpose() * A.transpose().inverse() * Q * A.inverse() * C_T;

    VecXd d_selected = VecXd::Zero(num_valid_variables);
    for (unsigned int i = 0; i < num_all_poly_coeff; ++i) {
        if (i == 0u) {
            d_selected[i] = waypoints_pos[0];
            continue;
        }

        if (i == 1u && order_ >= 2u) {
            d_selected[i] = waypoints_vel[0];
            continue;
        }

        if (i == 2u && order_ >= 3u) {
            d_selected[i] = waypoints_acc[0];
            continue;
        }

        if (i == num_all_poly_coeff - order_ + 2u && order_ >= 3u) {
            d_selected(num_fixed_variables - order_ + 2) = waypoints_acc[1];
            continue;
        }

        if (i == num_all_poly_coeff - order_ + 1u && order_ >= 2u) {
            d_selected(num_fixed_variables - order_ + 1) = waypoints_vel[1];
            continue;
        }

        if (i == num_all_poly_coeff - order_) {
            d_selected(num_fixed_variables - order_) = waypoints_pos[num_segments];
            continue;
        }

        if ((i % order_ == 0u) && (i / order_ % 2u == 0u)) {
            const unsigned int index = i / (2 * order_) + order_ - 1;
            d_selected(index) = waypoints_pos[i / (2 * order_)];
            continue;
        }
    }

    MatXd R_PP = R.block(num_fixed_variables, num_fixed_variables,
                         num_valid_variables - num_fixed_variables,
                         num_valid_variables - num_fixed_variables);

    VecXd d_F = d_selected.head(num_fixed_variables);
    MatXd R_FP = R.block(0, num_fixed_variables, num_fixed_variables,
                         num_valid_variables - num_fixed_variables);

    MatXd d_optimal = -R_PP.inverse() * R_FP.transpose() * d_F;

    d_selected.tail(num_valid_variables - num_fixed_variables) = d_optimal;
    VecXd d = C_T * d_selected;

    VecXd P = A.inverse() * d;

    MatXd poly_coeff_mat = MatXd::Zero(num_segments, num_poly_coeff);
    for (unsigned int i = 0; i < num_segments; ++i) {
        poly_coeff_mat.block(i, 0, 1, num_poly_coeff) =
                P.block(num_poly_coeff * i, 0, num_poly_coeff, 1).transpose();
    }

    return poly_coeff_mat;
}