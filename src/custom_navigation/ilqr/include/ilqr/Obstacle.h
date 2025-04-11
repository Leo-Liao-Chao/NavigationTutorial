#pragma once

#define EIGEN_USE_MKL_ALL
#include "mkl.h"

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "Parameters.h"
#include "struct_x_vx_mx_vmx.h"

class Obstacle
{
public:
    Obstacle(const Parameters &p);
    Obstacle(const Parameters &p, const Eigen::MatrixXd &dimension, const Eigen::MatrixXd &relative_pos_array);

    struct_x_vx_mx_vmx barrier_function(const double &q1, const double &q2, const double &c, const Eigen::Vector4d &c_dot);
    struct_x_vx_mx_vmx get_obstalce_cost(const int &i, const Eigen::VectorXd &ego_state);

private:
    Parameters p;
    Eigen::MatrixXd dimension;          // obstalce size
    Eigen::MatrixXd relative_pos_array; // 4*n
};