#include "pr_biomech/streaming_gdlf.hpp"

void pr_biomech::StreamingGDLF::CalTheta2Theta3() {

	struct Prj_struct {
		Vector3d LMO4_r_KJC_LM, LMO4_r_KJC_FH;
		Vector3d u_r_KJC_LM, u_r_KJC_FH;
		Vector2d u_r_LM_FH;
	};
	Prj_struct Prj;

	Vector3d LMO4_r_G_KJC = R.G_R_LMO4.transpose() * G_r_G.Femur;

	Vector3d LMO4_r_KJC_LM = -LMO4_r_G_KJC + R.G_R_LMO4.transpose() * G_r_G.LM;
	
	Prj.LMO4_r_KJC_LM = LMO4_r_KJC_LM - (cal_data->Plane4Bar.LMO4_u.transpose() * LMO4_r_KJC_LM) * cal_data->Plane4Bar.LMO4_u;

	Vector3d LMO4_r_KJC_FH = -LMO4_r_G_KJC + R.G_R_LMO4.transpose() * G_r_G.FH;
	
	Prj.LMO4_r_KJC_FH = LMO4_r_KJC_FH - (cal_data->Plane4Bar.LMO4_u.transpose() * LMO4_r_KJC_FH) * cal_data->Plane4Bar.LMO4_u;
		
	Prj.u_r_KJC_LM = R.u_R_LMO4 * Prj.LMO4_r_KJC_LM;
	Prj.u_r_KJC_FH = R.u_R_LMO4 * Prj.LMO4_r_KJC_FH;
	Prj.u_r_LM_FH = Prj.u_r_KJC_FH.head(2) - Prj.u_r_KJC_LM.head(2);

	double delta_theta3 = -atan2(Prj.u_r_LM_FH(0), Prj.u_r_LM_FH(1));

	theta3 = cal_data->Plane4Bar.DELTA + delta_theta3;
	int Ifail = Sol4BarGivenTheta3(theta3, cal_data->Plane4Bar.L(0), cal_data->Plane4Bar.L(1), cal_data->Plane4Bar.L(2), cal_data->Plane4Bar.L(3), theta2, theta4);

}

void pr_biomech::StreamingGDLF::Denavit(Matrix<double, 8, 1> a, Matrix<double, 8, 1> alpha, Matrix<double, 8, 1> theta, Matrix<double, 8, 1> d,int dof,Vector3d O0, Matrix<double, 3, Dynamic>& i_1_r_Oi_1_Oi, std::vector<std::vector<Eigen::Vector3d>>& i_r_Oi_Oj, std::vector<Eigen::Matrix3d>& i_1_R_i, std::vector<std::vector<Eigen::Matrix3d>>& i_R_j)
{
	
	Vector3d ro1o2;

	//std::cout << ro1o2 << std::endl;
	// Indicamos las dimensiones de las variables
	i_1_R_i.resize(dof + 1);
	i_1_r_Oi_1_Oi.resize(3, dof + 1); //¿Este proceso se puede hacer antes?
	// Ponemos a cero las variables
	i_1_r_Oi_1_Oi.fill(0);
	for (int i = 0; i < dof + 1; i++)
	{
		i_1_R_i[i].setZero(3, 3);
		//std::cout << i_1_R_i[i] << std::endl;
	}

	// Ponemos a 0 i_R_j y redimensionamos
	i_R_j.resize(dof + 1);
	for (int i = 0; i < dof + 1; i++)
	{
		i_R_j[i].resize(dof + 1);
		for (int j = 0; j < dof + 1; j++)
		{
			i_R_j[i][j].setZero(3, 3);
		}
	}
	// Ponemos a 0 i_r_Oi_Oj y redimensionamos
	i_r_Oi_Oj.resize(dof);
	for (int i = 0; i < dof; i++)
	{
		i_r_Oi_Oj[i].resize(dof + 1);
		for (int j = 0; j < dof + 1; j++)
		{
			i_r_Oi_Oj[i][j].setZero(3);
		}
	}

	// Cálculo de las matrices y vectores relativos entre segmentos
	//for (int i = 2; i < dof + 1; i++)
    for (int i = 1; i < dof + 1; i++)
	{
		i_1_R_i[i] <<	cos(theta(i)),	-1 * sin(theta(i)) * cos(alpha(i)), sin(theta(i))* sin(alpha(i)),
						sin(theta(i)),	cos(theta(i))* cos(alpha(i)),		-1 * cos(theta(i)) * sin(alpha(i)),
						0,				sin(alpha(i)),						cos(alpha(i));
		// i_1_r_Oi_1_Oi(Eigen::all, i) <<	a(i)* cos(theta(i)),
		// 							a(i)* sin(theta(i)),
		// 							d(i);
		i_1_r_Oi_1_Oi.col(i) <<	a(i)* cos(theta(i)),
									a(i)* sin(theta(i)),
									d(i);
	}

	// Posicion inicial
	// i_1_r_Oi_1_Oi(Eigen::all, 1) << O0;
	i_1_r_Oi_1_Oi.col(1) = O0;
	// Calculamos i_R_j y 
	for (int i = 0; i < dof; i++)
	{
		i_R_j[i][i + 1] = i_1_R_i[i + 1];
		//i_r_Oi_Oj[i][i + 1] = i_1_r_Oi_1_Oi(Eigen::all, i + 1);
		i_r_Oi_Oj[i][i + 1] = i_1_r_Oi_1_Oi.col(i + 1);
		for (int j = i + 2; j < dof + 1; j++)
		{
			i_R_j[i][j] = i_R_j[i][j - 1] * i_1_R_i[j];
			//i_r_Oi_Oj[i][j] = i_r_Oi_Oj[i][j - 1] + i_R_j[i][j - 1] * i_1_r_Oi_1_Oi(Eigen::all, j);
			i_r_Oi_Oj[i][j] = i_r_Oi_Oj[i][j - 1] + i_R_j[i][j - 1] * i_1_r_Oi_1_Oi.col(j);
		}
	}
	for (int i = 0; i < dof + 1; i++)
	{
		i_R_j[i][i].setIdentity(3, 3);
		for (int j = 0; j < i; j++)
		{
			i_R_j[i][j] = i_R_j[j][i].transpose();
		}
	}
}

// Interpolación de los Coef
//Vector <double, Dynamic>  pr_biomech::StreamingGDLF::Interpol(double x,Vector <double, 2> X, Matrix <double, Dynamic, 2> Y, Vector <double, Dynamic> y) {
Matrix <double, Dynamic, 1>  pr_biomech::StreamingGDLF::Interpol(double x,Matrix <double, 2, 1> X, Matrix <double, Dynamic, 2> Y) {
	// Funcion para calcular la interpolación lineal
	// x representa el valor intermedio del eje x
	// X representa los dos valores mas próximos en el eje x
	// Y representa todos los valores mas próximos en el eje y a interpolar
	Matrix <double, Dynamic, 1> y;
	// Separamos los valores
	double x1 = X[0];
	double x2 = X[1];
	Matrix <double, 1, Dynamic> y1 = Y.col(0);
	Matrix <double, 1, Dynamic> y2 = Y.col(1);
	// Interpolamos
	y = y1 +((x - x1) / (x2 - x1)) * (y2 - y1);
	return y;
}



Matrix <double, 6, 11>  pr_biomech::StreamingGDLF::Interpol_3D(double x, Matrix <double, 2, 1> X,Eigen::Matrix <double, 6, 11> y1, Eigen::Matrix <double, 6, 11> y2) {
	// Funcion para calcular la interpolación lineal
	// x representa el valor intermedio del eje x
	// X representa los dos valores mas próximos en el eje x
	// Y representa todos los valores mas próximos en el eje y a interpolar
	Matrix <double, 6, 11> y;
	// Separamos los valores
	double x1 = X[0];
	double x2 = X[1];
	//Matrix <double, 6, 11> y1 = Y[0];
	//Matrix <double, 6, 11> y2 = Y[1];
	//// Interpolamos
	y = y1 + ((x - x1) / (x2 - x1)) * (y2 - y1);
	//return y;
	
	return y;
}


void pr_biomech::StreamingGDLF::FindQ(Matrix <double, Dynamic, 1> Q5, double q5, Matrix<int, 2, 1>& n_q5) {
	int n = 0;
	bool Stop_while = true;

	//std::cout << "Tamaño Q5: " << Q5.size() << std::endl;

	// Si q5 es mayor que el máximo de Q5:
	if (q5 > Q5.maxCoeff()) {
		n_q5[0] = Q5.size()+1;
		n_q5[1] = Q5.size()+1;
		Stop_while = false;
	}
	// Si q5 es menor que el mínimo de Q5
	if (q5 < Q5.minCoeff()) {
		n_q5[0] = -1;
		n_q5[1] = -1;
		Stop_while = false;
	}
	// Si q5 es igual al máximo
	if (q5 == Q5.maxCoeff()) {
		n_q5[0] = Q5.size() - 2;
		n_q5[1] = Q5.size() - 1;
	};
	// Si q5 es al mínimo
	if (q5 == Q5.minCoeff()) {
		n_q5[0] = 0;
		n_q5[1] = 1;
		Stop_while = false;
	}

	// Si q5 se encuentra dentro de Q5 se procede al while
	while (n < Q5.size() && Stop_while) {
		if (n != Q5.size() - 1 && Q5[n] > q5) {
			//std::cout << "n es: " << n << std::endl;
			n_q5[0] = n;
			n_q5[1] = n + 1;
			// Paramos el bucle While
			Stop_while = false;
		}
		else {
			// Si llegamos al último valor se guardan la posiciones penúltima y última
			n_q5[0] = Q5.size() - 2;
			n_q5[1] = Q5.size() - 1;
		};

		// Sumamos uno al contador
		n++;
	};


};