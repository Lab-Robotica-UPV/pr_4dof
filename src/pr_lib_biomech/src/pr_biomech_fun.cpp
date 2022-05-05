#include "pr_lib_biomech/pr_biomech_fun.hpp"

/*----AUXILIAR FUNCTIONS----*/
void Denavit(Matrix<double, 5, 1> a, Matrix<double, 5, 1> alpha, Matrix<double, 6, 1> theta, Matrix<double, 6, 1> d, int dof, Vector3d ro1o2, Matrix<double, 3, Dynamic>& i_1_r_Oi_1_Oi, std::vector<std::vector<Eigen::Vector3d>>& i_r_Oi_Oj, std::vector<Eigen::Matrix3d>& i_1_R_i, std::vector<std::vector<Eigen::Matrix3d>>& i_R_j)
{
	i_1_R_i.resize(dof + 1);
	i_1_r_Oi_1_Oi.resize(3, dof + 1);
	i_1_r_Oi_1_Oi.fill(0);
	for (int i = 0; i < dof + 1; i++)
	{
		i_1_R_i[i].setZero(3, 3);
	}

	for (int i = 1; i < dof + 1; i++)
	{
		i_1_R_i[i] << cos(theta(i)), -1 * sin(theta(i)), 0,
			cos(alpha(i - 1))* sin(theta(i)), cos(alpha(i - 1))* cos(theta(i)), -sin(alpha(i - 1)),
			sin(alpha(i - 1))* sin(theta(i)), sin(alpha(i - 1))* cos(theta(i)), cos(alpha(i - 1));

		i_1_r_Oi_1_Oi.col(i) << a(i - 1),
			-sin(alpha(i - 1)) * d(i),
			cos(alpha(i - 1))* d(i);

	}

	i_1_r_Oi_1_Oi.col(1) = ro1o2;

	i_R_j.resize(dof + 1);
	for (int i = 0; i < dof + 1; i++)
	{
		i_R_j[i].resize(dof + 1);
		for (int j = 0; j < dof + 1; j++)
		{
			i_R_j[i][j].setZero(3, 3);
		}

	}

	i_r_Oi_Oj.resize(dof);
	for (int i = 0; i < dof; i++)
	{
		i_r_Oi_Oj[i].resize(dof + 1);
		for (int j = 0; j < dof + 1; j++)
		{
			i_r_Oi_Oj[i][j].setZero(3);
		}

	}


	for (int i = 0; i < dof; i++)
	{
		i_R_j[i][i + 1] = i_1_R_i[i + 1];
		i_r_Oi_Oj[i][i + 1] = i_1_r_Oi_1_Oi.col(i + 1);
		for (int j = i + 2; j < dof + 1; j++)
		{
			i_R_j[i][j] = i_R_j[i][j - 1] * i_1_R_i[j];
			i_r_Oi_Oj[i][j] = i_r_Oi_Oj[i][j - 1] + i_R_j[i][j - 1] * i_1_r_Oi_1_Oi.col(j);
		}
	}

	for (int i = 1; i < dof + 1; i++)
	{
		i_R_j[i][i].setIdentity(3, 3);
		for (int j = 0; j < i - 1; j++)
		{
			i_R_j[i][j] = i_R_j[j][i].transpose();
		}
	}
}

Matrix<double, 3, 5> jacobian(const PRDataStructures::R_struct& R, const Matrix<double, 3, Dynamic>& dd_r_X_ddqi, const Vector3d& InsPt, const std::vector<std::vector<Eigen::Matrix3d>>& i_R_j) {
	int n = dd_r_X_ddqi.cols();
	Matrix<double, 3, 5> J = Matrix<double, 3, 5>::Zero();
	Matrix3d Rotate = R.G_R_LMO4;
	if (n == 5) Rotate = R.G_R_SCS_Tib;
	for (int i = 1; i < n; i++) {
		J.col(i) = dd_r_X_ddqi.col(i) + i_R_j[0][i].col(2).cross(Rotate * InsPt);
	}
	return J;
}

Matrix<double, 3, 5> jacobian(const PRDataStructures::R_struct& R, const Matrix<double, 3, Dynamic>& dd_r_X_ddqi, const Vector3d& InsPt, const std::vector<std::vector<Eigen::Matrix3d>>& i_R_j, std::string location, const PRDataStructures::G_r_G_struct& G_r_G) {
	Matrix<double, 3, 5> J;
	if (location == "Foot") {
		int n = dd_r_X_ddqi.cols();
		J = Matrix<double, 3, 5>::Zero();
		for (int i = 1; i < n; i++) {
			J.col(i) = dd_r_X_ddqi.col(i) + i_R_j[0][i].col(2).cross(G_r_G.Cal2 + R.G_R_SCSpie * InsPt - G_r_G.SCS_Tib);
		}
	}
	else J = jacobian(R, dd_r_X_ddqi, InsPt, i_R_j);
	return J;
}