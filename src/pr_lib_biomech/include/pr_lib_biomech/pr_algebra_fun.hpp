#ifndef PR_LIB_BIOMECH__ALGEBRA_FUN_HPP_
#define PR_LIB_BIOMECH__ALGEBRA_FUN_HPP_

#include "eigen3/Eigen/Dense"
#include <iostream>

using namespace Eigen;

#define PI 3.14159265

// Unit vector
VectorXd unit(VectorXd v);

Matrix3d GetRotMat(Vector3d P1, Vector3d P2, Vector3d P3, int P2P3axis, int NormToPlaneAxis, int ThrdAxis);
/*
This function calculates the rotation matrix from 3 points.The first
axis is defined from P2 to P3, the second from the direction normal to
the plane that contains the three points.And the third from the right
hand rule.
*/

int Sol4BarGivenTheta3(double Theta3, double L1, double L2, double L3, double L4, double& theta2, double& theta3);

Vector2i FindLoc(double qcurr, VectorXd qmat);
// Returns a vector of the positions of qmat for which qcurr falls between
// Example: qmat = [3, 4, 5, 6], qcurr = 4.3 -> qLoc = [1, 2] (because qcurr is between 4 (index 1) and 5 (index 2), using C++ indexing)

VectorXd InterCoef(const double &x,const Vector2d &X,const Matrix<double, Dynamic, 2> &Y);

Matrix3d RotMat(double theta, char axis);

// Sine in degrees
double sind(double n);

// Cosine in degrees
double cosd(double n);

#endif
