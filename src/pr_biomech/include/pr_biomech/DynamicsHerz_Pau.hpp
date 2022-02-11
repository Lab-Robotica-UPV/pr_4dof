#ifndef PR_BIOMECH__DYNAMICSHERZ_PAU_
#define PR_BIOMECH__DYNAMICSHERZ_PAU_

#include "pr_biomech/streaming_gdlf.hpp"

Eigen::Matrix<double, 3, Dynamic> i_1_r_Oi_1_Oi;
//std::vector<std::vector <double>> i_1_r_Oi_1_Oi;
std::vector<Eigen::Matrix3d> i_1_R_i;
std::vector<std::vector<Eigen::Matrix3d>> i_R_j;
std::vector<std::vector<Eigen::Vector3d>>i_r_Oi_Oj;


//Auxiliar Functions
void Denavit(Matrix<double, 5, 1> a, Matrix<double, 5, 1> alpha, Matrix<double, 6, 1> theta, Matrix<double, 6, 1> d, int dof, Vector3d ro1o2, Matrix<double, 3, Dynamic>& i_1_r_Oi_1_Oi, std::vector<std::vector<Eigen::Vector3d>>& i_r_Oi_Oj, std::vector<Eigen::Matrix3d>& i_1_R_i, std::vector<std::vector<Eigen::Matrix3d>>& i_R_j);
Matrix<double, 3, 5> jacobian(const PRDataStructures::R_struct& R, const Matrix<double, 3, Dynamic>& dd_r_X_ddqi, const Vector3d& InsPt);
Matrix<double, 3, 5> jacobian(const PRDataStructures::R_struct& R, const Matrix<double, 3, Dynamic>& dd_r_X_ddqi, const Vector3d& InsPt, std::string location, const PRDataStructures::G_r_G_struct& G_r_G);

Matrix3d GetRotMat2V(Vector3d V1, Vector3d V2, int dir1, int dir2);
// Esta función es para calcular la matiz de rotación considerando dos
// vectores, el primero representa la primera dirección(dir1) y la segunda
// dirección esta en el plano formado por V1 y V2 y lo mas cerca posible a
// V2.
#endif // PR_BIOMECH__DYNAMICSHERZ_PAU_