//
// Created by meng on 2020/9/9.
//
#include "trajectory_generator.h"
#include <iostream>

using namespace std;
using namespace Eigen;

/*!
 * 计算x的阶乘
 * @param x
 * @return x!
 */
int TrajectoryGeneratorTool::Factorial(int x) {
    int fac = 1;
    for (int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

/*!
 * 通过闭式求解QP，得到每段拟合轨迹的多项式系数
 * @param order 导数阶数。例如最小化jerk，则需要求解三次导数，则 d_order=3
 * @param Path 航迹点的空间坐标(3D)
 * @param Vel 航迹点对应的速度(中间点速度是待求的未知量)
 * @param Acc 航迹点对应的加速度(中间点加速度是待求的未知量)
 * @param Time 每段轨迹对应的时间周期
 * @return 轨迹x,y,z三个方向上的多项式系数
 *
 * 返回矩阵(PolyCoeff)的数据格式：每一行是一段轨迹，第一列是x方向上的多项式次数，越左次数越高
 * 第一段轨迹三个方向上的系数 | px_i px_(i-1) px_(i-2) ... px_1 px_0 | y ... | z ... |
 * 第二段轨迹三个方向上的系数 | px_i px_(i-1) px_(i-2) ... px_1 px_0 | y ... | z ... |
 *                          ........
 *
 * 注意：给定起始点和终点的速度加速度，更高阶的导数设置为0
 */
Eigen::MatrixXd TrajectoryGeneratorTool::SolveQPClosedForm(
        int order,
        const Eigen::MatrixXd &Path,
        const Eigen::MatrixXd &Vel,
        const Eigen::MatrixXd &Acc,
        const Eigen::VectorXd &Time) {

    const int p_order = 2 * order - 1;//多项式的最高次数 p^(p_order)t^(p_order) + ...
    const int p_num1d = p_order + 1;//每一段轨迹的变量个数，对于五阶多项式为：p5, p4, ... p0

    const int number_segments = Time.size();
    //每一段都有x,y,z三个方向，每一段多项式的系数的个数有3*p_num1d
    MatrixXd PolyCoeff = MatrixXd::Zero(number_segments, 3 * p_num1d);
    //整条轨迹在ｘ,y,z方向上共多少个未知系数
    const int number_coefficients = p_num1d * number_segments;
    VectorXd Px(number_coefficients), Py(number_coefficients), Pz(number_coefficients);

    const int M_block_rows = order * 2;
    const int M_block_cols = p_num1d;
    //M：转换矩阵，将系数向量转换为方程的微分量
    MatrixXd M = MatrixXd::Zero(number_segments * M_block_rows, number_segments * M_block_cols);
    for (int i = 0; i < number_segments; ++i) {
        int row = i * M_block_rows, col = i * M_block_cols;
        MatrixXd sub_M = MatrixXd::Zero(M_block_rows, M_block_cols);

        for (int j = 0; j < order; ++j) {
            for (int k = 0; k < p_num1d; ++k) {
                if (k < j)
                    continue;

                sub_M(j, p_num1d - 1 - k) = Factorial(k) / Factorial(k - j) * pow(0, k - j);
                sub_M(j + order, p_num1d - 1 - k) = Factorial(k) / Factorial(k - j) * pow(Time(i), k - j);
            }
        }

        M.block(row, col, M_block_rows, M_block_cols) = sub_M;
    }

    //构造选择矩阵C的过程非常复杂，但是只要多花点时间探索一些规律，举几个例子，应该是能写出来的!!
    const int number_valid_variables = (number_segments + 1) * order;
    const int number_fixed_variables = 2 * order + (number_segments - 1);
    //C_T：选择矩阵，用于分离未知量和已知量
    MatrixXd C_T = MatrixXd::Zero(number_coefficients, number_valid_variables);
    for (int i = 0; i < number_coefficients; ++i) {
        if (i < order) {
            C_T(i, i) = 1;
            continue;
        }

        if (i >= number_coefficients - order) {
            const int delta_index = i - (number_coefficients - order);
            C_T(i, number_fixed_variables - order + delta_index) = 1;
            continue;
        }

        if ((i % order == 0) && (i / order % 2 == 1)) {
            const int index = i / (2 * order) + order;
            C_T(i, index) = 1;
            continue;
        }

        if ((i % order == 0) && (i / order % 2 == 0)) {
            const int index = i / (2 * order) + order - 1;
            C_T(i, index) = 1;
            continue;
        }

        if ((i % order != 0) && (i / order % 2 == 1)) {
            const int temp_index_0 = i / (2 * order) * (2 * order) + order;
            const int temp_index_1 = i / (2 * order) * (order - 1) + i - temp_index_0 - 1;
            C_T(i, number_fixed_variables + temp_index_1) = 1;
            continue;
        }

        if ((i % order != 0) && (i / order % 2 == 0)) {
            const int temp_index_0 = (i - order) / (2 * order) * (2 * order) + order;
            const int temp_index_1 = (i - order) / (2 * order) * (order - 1) + (i - order) - temp_index_0 - 1;
            C_T(i, number_fixed_variables + temp_index_1) = 1;
            continue;
        }
    }

    // Q：二项式的系数矩阵
    MatrixXd Q = MatrixXd::Zero(number_coefficients, number_coefficients);
    for (int k = 0; k < number_segments; ++k) {
        MatrixXd sub_Q = MatrixXd::Zero(p_num1d, p_num1d);
        for (int i = 0; i <= p_order; ++i) {
            for (int l = 0; l <= p_order; ++l) {
                if (p_num1d - i <= order || p_num1d - l <= order)
                    continue;

                sub_Q(i, l) = (Factorial(p_order - i) / Factorial(p_order - order - i)) *
                              (Factorial(p_order - l) / Factorial(p_order - order - l)) /
                              (p_order - i + p_order - l - (2 * order - 1)) *
                              pow(Time(k), p_order - i + p_order - l - (2 * order - 1));
            }
        }

        const int row = k * p_num1d;
        Q.block(row, row, p_num1d, p_num1d) = sub_Q;
    }

    MatrixXd R = C_T.transpose() * M.transpose().inverse() * Q * M.inverse() * C_T;

    for (int axis = 0; axis < 3; ++axis) {
        VectorXd d_selected = VectorXd::Zero(number_valid_variables);
        for (int i = 0; i < number_coefficients; ++i) {
            if (i == 0) {
                d_selected(i) = Path(0, axis);
                continue;
            }

            if (i == 1 && order >= 2) {
                d_selected(i) = Vel(0, axis);
                continue;
            }

            if (i == 2 && order >= 3) {
                d_selected(i) = Acc(0, axis);
                continue;
            }

            if (i == number_coefficients - order + 2 && order >= 3) {
                d_selected(number_fixed_variables - order + 2) = Acc(1, axis);
                continue;
            }

            if (i == number_coefficients - order + 1 && order >= 2) {
                d_selected(number_fixed_variables - order + 1) = Vel(1, axis);
                continue;
            }

            if (i == number_coefficients - order) {
                d_selected(number_fixed_variables - order) = Path(number_segments, axis);
                continue;
            }

            if ((i % order == 0) && (i / order % 2 == 0)) {
                const int index = i / (2 * order) + order - 1;
                d_selected(index) = Path(i / (2 * order), axis);
                continue;
            }
        }

        MatrixXd R_PP = R.block(number_fixed_variables, number_fixed_variables,
                                number_valid_variables - number_fixed_variables,
                                number_valid_variables - number_fixed_variables);
        VectorXd d_F = d_selected.head(number_fixed_variables);
        MatrixXd R_FP = R.block(0, number_fixed_variables, number_fixed_variables,
                                number_valid_variables - number_fixed_variables);

        MatrixXd d_optimal = -R_PP.inverse() * R_FP.transpose() * d_F;

        d_selected.tail(number_valid_variables - number_fixed_variables) = d_optimal;
        VectorXd d = C_T * d_selected;

        if (axis == 0)
            Px = M.inverse() * d;

        if (axis == 1)
            Py = M.inverse() * d;

        if (axis == 2)
            Pz = M.inverse() * d;
    }

    for (int i = 0; i < number_segments; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (j == 0) {
                PolyCoeff.block(i, j * p_num1d, 1, p_num1d) =
                        Px.block(p_num1d * i, 0, p_num1d, 1).transpose();
                continue;
            }

            if (j == 1) {
                PolyCoeff.block(i, j * p_num1d, 1, p_num1d) =
                        Py.block(p_num1d * i, 0, p_num1d, 1).transpose();
                continue;
            }

            if (j == 2) {
                PolyCoeff.block(i, j * p_num1d, 1, p_num1d) =
                        Pz.block(p_num1d * i, 0, p_num1d, 1).transpose();
                continue;
            }
        }
    }

    return PolyCoeff;
    /// 该矩阵的实现其数学原理并不难，但是矩阵构造的细节实在是复杂啊
}
