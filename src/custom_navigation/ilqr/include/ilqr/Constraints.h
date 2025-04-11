#pragma once

#define EIGEN_USE_MKL_ALL
#include "mkl.h"

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>
#include <vector>
#include "Parameters.h"
#include "Obstacle.h"
#include "struct_x_vx_mx_vmx.h"

class Constraints
{
public:
    Constraints(const Parameters &param);

    // find_closest_point
    Eigen::Vector2d find_closest_point(const Eigen::Vector4d &state, const Eigen::VectorXd &coeffs, const Eigen::VectorXd &x_local_plan);
    // barrier function
    struct_x_vx_mx_vmx barrier_function(const double &q1, const double &q2, const double &c, const Eigen::Vector2d &c_dot);
    // get_control_cost_derivate
    struct_x_vx_mx_vmx get_control_cost(const Eigen::MatrixXd &state, const Eigen::MatrixXd &control);
    // get_state_cost_derivate
    struct_x_vx_mx_vmx get_state_cost(const Eigen::MatrixXd &state, const Eigen::VectorXd &poly_coeffs, const Eigen::VectorXd &x_local_plan);
    // get_state_control_cost_derivate
    Eigen::Tensor<double,3> get_state_control_cost_dd();

    // get_state_cost_derivate Obstacle
    void set_obstalces(const std::vector<Obstacle> &obstacles);
    void clear_obstacle();

    // get J
    double get_J(const Eigen::MatrixXd &state, const Eigen::MatrixXd &control, const Eigen::VectorXd &poly_coeffs, const Eigen::VectorXd &x_local_plan);

private:
    Parameters param;
    Eigen::Matrix2d control_cost;
    Eigen::Matrix4d state_cost;

    std::vector<Obstacle> obstacles;
};
