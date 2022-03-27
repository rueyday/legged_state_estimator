/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   LieGroup.h
 *  @author Ross Hartley
 *  @brief  Header file for various Lie Group functions 
 *  @date   September 25, 2018
 **/

#ifndef INEKF_LIEGROUP_H
#define INEKF_LIEGROUP_H 
#include <Eigen/Dense>
#include <iostream>
#include <cmath>

namespace inekf {

long int factorial(const int n);
Eigen::Matrix3d skew(const Eigen::Vector3d& v);
Eigen::Matrix3d Gamma_SO3(const Eigen::Vector3d& w, const int n,
                          const double exp_map_tol=1.0e-10);
Eigen::Matrix3d Exp_SO3(const Eigen::Vector3d& w);
Eigen::Matrix3d LeftJacobian_SO3(const Eigen::Vector3d& w);
Eigen::Matrix3d RightJacobian_SO3(const Eigen::Vector3d& w);
Eigen::MatrixXd Exp_SEK3(const Eigen::VectorXd& v, 
                         const double exp_map_tol=1.0e-10);
Eigen::MatrixXd Adjoint_SEK3(const Eigen::MatrixXd& X);

} // end inekf namespace
#endif 
