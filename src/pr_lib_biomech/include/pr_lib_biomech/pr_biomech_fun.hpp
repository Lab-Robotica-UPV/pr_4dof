#ifndef PR_LIB_BIOMECH__BIOMECH_FUN_HPP_
#define PR_LIB_BIOMECH__BIOMECH_FUN_HPP_

#include "eigen3/Eigen/Dense"
#include <iostream>
#include <vector>
#include "pr_data_structures.hpp"

using namespace Eigen;

void Denavit(Matrix<double, 5, 1> a, Matrix<double, 5, 1> alpha, Matrix<double, 6, 1> theta, Matrix<double, 6, 1> d, int dof, Vector3d ro1o2, Matrix<double, 3, Dynamic>& i_1_r_Oi_1_Oi, std::vector<std::vector<Eigen::Vector3d>>& i_r_Oi_Oj, std::vector<Eigen::Matrix3d>& i_1_R_i, std::vector<std::vector<Eigen::Matrix3d>>& i_R_j);
Matrix<double, 3, 5> jacobian(const PRDataStructures::R_struct& R, const Matrix<double, 3, Dynamic>& dd_r_X_ddqi, const Vector3d& InsPt, const std::vector<std::vector<Eigen::Matrix3d>>& i_R_j);
Matrix<double, 3, 5> jacobian(const PRDataStructures::R_struct& R, const Matrix<double, 3, Dynamic>& dd_r_X_ddqi, const Vector3d& InsPt, const std::vector<std::vector<Eigen::Matrix3d>>& i_R_j, std::string location, const PRDataStructures::G_r_G_struct& G_r_G);

#endif