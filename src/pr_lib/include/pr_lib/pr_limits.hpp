#ifndef PR_LIB__LIMITS_HPP_
#define PR_LIB__LIMITS_HPP_

#include <vector>
#include <array>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"


namespace PRLimits {

    Eigen::Matrix<double,4,2> LimActuators(bool robot_5p = false);

    Eigen::Vector4d LimAngles(bool robot_5p = false);

    Eigen::Matrix<int,7,1> VerFactPos(const std::array<double, 4> &X, const std::vector<double> &RParam, bool robot_5p = false);

    Eigen::Vector4d AngPatas(const double &theta, const Eigen::Matrix<double,4,3> &q);
}


#endif