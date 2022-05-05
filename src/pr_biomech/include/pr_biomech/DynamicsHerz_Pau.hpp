#ifndef PR_BIOMECH__DYNAMICSHERZ_PAU_
#define PR_BIOMECH__DYNAMICSHERZ_PAU_

#include "pr_biomech/streaming_gdlf.hpp"

Eigen::Matrix<double, 3, Dynamic> i_1_r_Oi_1_Oi;
//std::vector<std::vector <double>> i_1_r_Oi_1_Oi;
std::vector<Eigen::Matrix3d> i_1_R_i;
std::vector<std::vector<Eigen::Matrix3d>> i_R_j;
std::vector<std::vector<Eigen::Vector3d>>i_r_Oi_Oj;


Matrix3d GetRotMat2V(Vector3d V1, Vector3d V2, int dir1, int dir2);
// Esta funci�n es para calcular la matiz de rotaci�n considerando dos
// vectores, el primero representa la primera direcci�n(dir1) y la segunda
// direcci�n esta en el plano formado por V1 y V2 y lo mas cerca posible a
// V2.
#endif // PR_BIOMECH__DYNAMICSHERZ_PAU_