#include "pr_lib_biomech/pr_algebra_fun.hpp"

// Unit vector
VectorXd unit(VectorXd v) {
	return v / v.norm();
}

Matrix3d GetRotMat(Vector3d P1, Vector3d P2, Vector3d P3, int P2P3axis, int NormToPlaneAxis, int ThrdAxis) {
	/*
	This function calculates the rotation matrix from 3 points.The first
	axis is defined from P2 to P3, the second from the direction normal to
	the plane that contains the three points.And the third from the right
	hand rule.
					P1
				   /  \
		          /    \
		         /      \
		        /        \      NormToPlaneAxis is calculated from : unit(cross2(P2 - P1, P3 - P1))
		       /          \
		      /            \
		     /              \
		P2-> > -P2P3axis-> > P3
		*/
	// The index of the columns are subtracted 1 to match the difference between Matlab (starts from 1) and C++ (starts from 0)
	// This way the function is used in a Matlab-indexing way
	Matrix3d R = Matrix3d::Zero();
	R.col(P2P3axis-1) = unit(P3 - P2);
	if (NormToPlaneAxis < 0) {
		NormToPlaneAxis = -NormToPlaneAxis;
		R.col(NormToPlaneAxis - 1) = -unit((P2 - P1).cross(P3 - P1));
	}
	else {
		R.col(NormToPlaneAxis - 1) = unit((P2 - P1).cross(P3 - P1));
	}

	if (NormToPlaneAxis == (P2P3axis + 1) || (P2P3axis == 3 && NormToPlaneAxis == 1)) {
		R.col(ThrdAxis-1) = R.col(P2P3axis-1).cross(R.col(NormToPlaneAxis-1));
	}
	else {
		R.col(ThrdAxis-1) = -R.col(P2P3axis-1).cross(R.col(NormToPlaneAxis-1));
	}

	return R;
}

int Sol4BarGivenTheta3(double Theta3, double L1, double L2, double L3, double L4, double &Theta2, double &Theta4) {
	int Ifail = 0;
	double A = (L1 - L3 * cos(Theta3)) / L4;
	double B = -L3 * sin(Theta3) / L4;
	double D2 = pow((L2 / L4),2);
	double C = (D2 - 1 - pow(A, 2) - pow(B, 2)) / 2;
	if (pow(A, 2) + pow(B, 2) - pow(C, 2) >= 0) {
		Theta4 = atan2(B, A) + atan2(sqrt(pow(A, 2) + pow(B, 2) - pow(C, 2)), C);
		Theta2 = atan2(B + sin(Theta4), A + cos(Theta4));
		Ifail = 0;
	}
	else {
		Theta4 = atan(1) * 4; // Pi number
		Theta2 = 0;
		std::cout << "Error in solving the inverse solution of the 4 bar mechanism" << std::endl;
		Ifail = 1;
	}
	return Ifail;
}

Vector2i FindLoc(double qcurr, VectorXd qmat) {
	// Returns a vector of the positions of qmat for which qcurr falls between
	// Example: qmat = [3, 4, 5, 6], qcurr = 4.3 -> qLoc = [1, 2] (because qcurr is between 4 (index 1) and 5 (index 2), using C++ indexing)
	Vector2i qLoc;

	int npts = qmat.size();

	double qstep = qmat(1) - qmat(0);
	qLoc(0) = static_cast<int>(floor((qcurr - qmat(0)) / qstep) + 1);

	if (qLoc(0) < (npts-1) && qLoc(0) >= 0) {
		qLoc(1) = qLoc(0) + 1;
	}

	else if(qLoc(0) >= (npts-1)) {
		qLoc << npts - 2, npts - 1;
	}

	else {
		qLoc << 0, 1;
	}

	return qLoc;

}

VectorXd InterCoef(const double &x,const Vector2d &X,const Matrix<double, Dynamic, 2> &Y) {
	/*
	Funcion para calcular la interpolaci�n lineal
	x representa el valor intermedio del eje x
	X representa los dos valores mas pr�ximos en el eje x
	Y representa todos los valores mas pr�ximos en el eje y a interpolar
	*/
	double x1 = X(0); 
	double x2 = X(1);

	VectorXd y1 = Y.col(0);
	VectorXd y2 = Y.col(1);

	VectorXd y = y1 + (y2 - y1) * (x - x1) / (x2 - x1);
	return y;

}

Matrix3d RotMat(double theta, char axis)
{
	Eigen::Matrix3d R;

	axis = ::tolower(axis);
	switch (axis) {
	case 'x':
	{
		R(0, 0) = 1;
		R(0, 1) = 0;
		R(0, 2) = 0;

		R(1, 0) = 0;
		R(1, 1) = cos(theta);
		R(1, 2) = -1 * sin(theta);

		R(2, 0) = 0;
		R(2, 1) = sin(theta);
		R(2, 2) = cos(theta);
		break;

	}
	case 'y':
	{
		R(0, 0) = cos(theta);
		R(0, 1) = 0;
		R(0, 2) = sin(theta);

		R(1, 0) = 0;
		R(1, 1) = 1;
		R(1, 2) = 0;

		R(2, 0) = -1 * sin(theta);
		R(2, 1) = 0;
		R(2, 2) = cos(theta);
		break;
	}
	case 'z':
	{
		R(0, 0) = cos(theta);
		R(0, 1) = -1 * sin(theta);
		R(0, 2) = 0;

		R(1, 0) = sin(theta);
		R(1, 1) = cos(theta);
		R(1, 2) = 0;

		R(2, 0) = 0;
		R(2, 1) = 0;
		R(2, 2) = 1;
		break;
	}
	default:
		std::cout << "Invalid argument" << std::endl;
	}
	return R;
}

// Sine in degrees
double sind(double n) {
	return sin(n * PI / 180.0);
}

// Cosine in degrees
double cosd(double n) {
	return cos(n * PI / 180.0);
}