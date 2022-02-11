#include "pr_biomech/streaming_gdlf.hpp"
#include "pr_biomech/DynamicsHerz_Pau.hpp"
#include <cmath>


void pr_biomech::StreamingGDLF::DynamicsHerz_Pau()
{
	/*----FUCTIONS'S PARAMETERS----*/
	//Constant for the fuction

	int dof = 5;
	Matrix3d LMO4_R_u, u_R_G, G_R_u, u_R_O4, O4_R_u, G_R_O4, u_R_O5, O5_R_u, O5_R_G, G_R_O5;
	Matrix3d G_R_SCS_Fem, ACSpelvis_R_SCS_Fem;
	Matrix<double, 5, 1> Tau_G, Tau_G_pie, Tau_Fext, Tau_Mext, Tau_TOT, a, alpha;
	Matrix<double, 6, 1> d;
	Matrix<double, 3, 4> dd_r_LMO4_ddqi;
	Matrix<double, 3, 5> J_Gpie, J_Ipie, J_Fext, dd_r_SCS_Tib_ddqi, J_G4, J_G6, J_Tau_ext;
	Vector3d u_r_O5est_A, ro1o2, G_r_O4_O5, u_r_O4_O5, temp;
	Vector3d ACSpelvis_u_y, SCS_Tib_y_pie, G_r_AJC_AJC2, LMO4_rg4;
	Vector3d LMO4_O5;
	double beta, a4, d5, L_O5_IC;

	/*----INITIALIZATION----*/
	LMO4_R_u = R.LMO4_R_u;			//ESTA VARIABLE SE USA SOLO DOS VECES (CALCULO DE "G_r_G.O5" y "u_R_G") YO LA ELIMINARIA 
	u_r_O5est_A = A;				//ESTA VARIABLE SE USA UNA SOLA VEZ PARA CALCULAR "G_r_G.O5". YO LA ELIMINARIA

	Tau_G.setZero(dof, 1);
	Tau_G_pie.setZero(dof, 1);
	Tau_Fext.setZero(dof, 1);
	Tau_Mext.setZero(dof, 1);

	// PREGUNTAR A RAFA SI CONSIDERA ESTA INICIALIZACI�N NECESARIA, SI YA LA HIZO EL ES REDUNDANTE
	G_r_G.CPM.setZero(3, 1);
	G_r_G.CPL.setZero(3, 1);
	G_r_G.TibColLigTib.setZero(3, 1);
	G_r_G.FibColLigTib.setZero(3, 1);
	G_r_G.TibColLigTib_un.setZero(3, 1);
	G_r_G.FibColLigTib_un.setZero(3, 1);

	G_r_G.Ext.PatLig.Pat.setZero(3, 1);

	G_r_G.ViaPtsViaContours.CylCent.setZero(3, 1);
	G_r_G.ViaPtsViaContours.CylDir.setZero(3, 1);
	/*FIN PREGUNTA A RAFA*/

	J_Gpie.setZero(3, 5);
	J_Ipie.setZero(3, 5);
	J_Fext.setZero(3, 5);

	d.fill(0);

	dd_r_LMO4_ddqi.fill(0);
	dd_r_SCS_Tib_ddqi.fill(0);


	G_r_G.LMO4 = G_r_G.LFE;
	G_r_G.O1 = G_r_G.O4;
	ro1o2 = G_r_G.O1;						//ESTA VARIABLE SE USA UNA SOLA VEZ PARA LLAMAR A DNAVIT. YO LA ELIMINARIA

	G_r_G.O5 = G_r_G.KJC + R.G_R_LMO4 * LMO4_R_u * u_r_O5est_A;
	G_r_O4_O5 = G_r_G.O5 - G_r_G.O4;		//ESTA VARIABLE SE USA UNA SOLA VEZ PARA LLAMAR A DNAVIT. YO LA ELIMINARIA

	u_R_G = LMO4_R_u.transpose() * R.LMO4_R_G;
	G_R_u = u_R_G.transpose();				// ESTA VARIABLE REEMPLAZA A G.G_R_u PORQUE NO SE USA EN OTRAS PARTES DEL CODIGO
	u_r_O4_O5 = u_R_G * G_r_O4_O5;
	beta = atan2(u_r_O4_O5(1), u_r_O4_O5(0));

	u_R_O4 = RotMat(beta, 'z');
	O4_R_u = u_R_O4.transpose();
	temp = O4_R_u * u_r_O4_O5;
	a4 = temp(0);
	d5 = temp(2);
	// G_R_04 en realidad era un campo de R
	G_R_O4 = G_R_u * u_R_O4;
	u_R_O5 = RotMat(theta0 + theta2, 'z');
	O5_R_u = u_R_O5.transpose();
	O5_R_G = O5_R_u * u_R_G;
	//Y con G_R_05 igual
	G_R_O5 = O5_R_G.transpose();

	/*----DH VARIABLES----*/
	a <<
		0,
		0,
		0,
		a4,
		L(1);

	alpha <<
		0,
		-1 * (PI / 2),
		PI / 2,
		0,
		0;

	d(4) = d5;

	// declaro de momento q en el .hpp
	q(4, 0) = theta2 + theta0 - beta - 2 * PI;
	q(5, 0) = theta3 - theta2;

	q(1, 0) = atan2(G_R_O4(1, 2), G_R_O4(0, 2));
	q(2, 0) = atan2(sqrt(pow(G_R_O4(0, 2), 2) + pow(G_R_O4(1, 2), 2)), G_R_O4(2, 2));
	//q(4, 1) = atan2(sin(q(3)) * R.G_R_O4(3, 2), -sin(q(3)) * R.G_R_O4(3, 1));
	q(3, 0) = atan2(sin(q(2, 0)) * G_R_O4(2, 1), -sin(q(2, 0)) * G_R_O4(2, 0));
	//std::cout << q << std::endl;
	Denavit(a, alpha, q, d, dof, ro1o2, i_1_r_Oi_1_Oi, i_r_Oi_Oj, i_1_R_i, i_R_j);
	//Prints for verification ----------------------------------------------------------------------------------------Borrar al final
	/*
	for (int i = 0; i < 6;i++) {
		for (int j = 0; j < 6; j++){
			std::cout <<"i:" << i << "j:" << j << std::endl;
			std::cout << i_R_j[i][j] << std::endl;
		}
	}
	*/

	/*----LOCATION AND ROTATION MATRIX FOR FEMUR----*/
	G_R_SCS_Fem = GetRotMat2V((G_r_G.O4 - G_r_G.KJC), (G_r_G.MFE - G_r_G.LFE), 2, 3);	// ESTA VARIABLE REEMPLAZA A R.G_R_SCS_Fem PORQUE NO SE USA EN OTRAS PARTES DEL CODIGO
	//Prints for verification ----------------------------------------------------------------------------------------Borrar al final
	//std::cout << G_R_SCS_Fem << std::endl;
	G_r_G.SCS_Fem = G_r_G.O4;

	/*----ROTATION MATRIX FOR THE TIBIA RESPECT TO THE GROUND----*/
	R.G_R_SCS_Tib = i_R_j[0][5] * cal_data->O6_R_SCS_Tib;			// ESTA VARIABLE LA VA A�ADIR RAFA EN SU ESTRUCTURA R
	G_r_G.SCS_Tib = i_r_Oi_Oj[0][5] + (i_R_j[0][5] * cal_data->O6_r_O6.SCS_Tib);
	//Prints for verification ----------------------------------------------------------------------------------------Borrar al final
	//std::cout << G_r_G.SCS_Tib << std::endl;

	/*----FLECTION-EXTENSION FOR THE HIP----*/
	ACSpelvis_R_SCS_Fem = R.G_R_ACSpelvis.transpose() * G_R_SCS_Fem; //Utilizo G_R_SCS_Fem en lugar de R.G_R_SCS_Fem
	ACSpelvis_u_y = ACSpelvis_R_SCS_Fem.col(1);
	/*----ANGLE FOR THE PATIENT----*/
	Data.ang.cad.FlxExt(GlobalCnt) = atan2(-ACSpelvis_u_y(0), ACSpelvis_u_y(1)) * (180 / PI) - cal_data->ang_cad_FlxExt_i;
	Data.ang.cad.AbdAduc(GlobalCnt) = atan2(-ACSpelvis_u_y(2), ACSpelvis_u_y(1)) * (180 / PI) - cal_data->ang_cad_AbdAduc_i;
	Data.ang.cad.IntExt(GlobalCnt) = atan2(-ACSpelvis_R_SCS_Fem(2, 0), ACSpelvis_R_SCS_Fem(2, 2)) * (180 / PI);
	Data.ang.rod.FlxExt(GlobalCnt) = acos((G_R_SCS_Fem.col(1).transpose() * R.G_R_SCS_Tib.col(1))) * (180 / PI) - cal_data->ang_rod_FlxExt_i; //Utilizo G_R_SCS_Fem en lugar de R.G_R_SCS_Fem, no defino G_u_y tomo directamente la columna de R.G_R_SCS_Fem y la transpongo

	SCS_Tib_y_pie = R.G_R_SCS_Tib.transpose() * R.G_R_SCSpie.col(1);
	Data.ang.pie.FlxExt(GlobalCnt) = atan2(-SCS_Tib_y_pie(0), SCS_Tib_y_pie(1)) * (180 / PI) - cal_data->ang_pie_FlxExt_i;
	//Prints for verification ----------------------------------------------------------------------------------------Borrar al final
	/*
	std::cout << GlobalCnt << " cad.FlxExt:\t" << Data.ang.cad.FlxExt(GlobalCnt) << std::endl;
	std::cout << GlobalCnt << " cad.AbdAduc:\t" << Data.ang.cad.AbdAduc(GlobalCnt) << std::endl;
	std::cout << GlobalCnt << " cad.IntExt:\t" << Data.ang.cad.IntExt(GlobalCnt) << std::endl;
	std::cout << GlobalCnt << " rod.FlxExt:\t" << Data.ang.rod.FlxExt(GlobalCnt) << std::endl;
	std::cout << GlobalCnt << " pie.FlxExt:\t" << Data.ang.pie.FlxExt(GlobalCnt) << std::endl;
	*/

	G_r_G.LM2 = i_r_Oi_Oj[0][5] + (i_R_j[0][5] * cal_data->O6_r_O6.LM2);
	G_r_G.MM2 = i_r_Oi_Oj[0][5] + (i_R_j[0][5] * cal_data->O6_r_O6.MM2);
	G_r_G.AJC2 = (G_r_G.LM2 + G_r_G.MM2) / 2;
	G_r_AJC_AJC2 = G_r_G.AJC2 - G_r_G.AJC;
	u_n = i_R_j[0][4].col(2);
	//Prints for verification ----------------------------------------------------------------------------------------Borrar al final
	/*
	std::cout << GlobalCnt << " G_r_G.LM2:\n" << G_r_G.LM2 << std::endl;
	std::cout << GlobalCnt << " G_r_G.MM2:\n" << G_r_G.MM2 << std::endl;
	std::cout << GlobalCnt << " G_r_G.AJC2:\n" << G_r_G.AJC2 << std::endl;
	std::cout << GlobalCnt << " G_r_AJC_AJC2:\n" << G_r_AJC_AJC2 << std::endl;
	std::cout << GlobalCnt << " u_n:\n" << u_n << std::endl;
	*/

	/*----JACOBIAN AND COEFFICIENT OF MUSCLE FORCES----*/
	for (int i = 1; i < 4; i++)
	{
		dd_r_LMO4_ddqi.col(i) = i_R_j[0][i].col(2).cross((G_r_G.LMO4 - i_r_Oi_Oj[0][i]));
	}
	//Prints for verification ----------------------------------------------------------------------------------------Borrar al final
	//std::cout << GlobalCnt << " dd_r_LMO4_ddqi:\n" << dd_r_LMO4_ddqi << std::endl;

	L_O5_IC = L(1) / sin((theta4 - theta2)) * sin(PI - theta4);
	G_r_G.IC = i_r_Oi_Oj[0][4] + i_R_j[0][4].col(0) * L_O5_IC;
	//Prints for verification ----------------------------------------------------------------------------------------Borrar al final
	//std::cout << GlobalCnt << " G_r_G.IC:\n" << G_r_G.IC << std::endl;

	for (int i = 1; i < 4; i++)
	{
		dd_r_SCS_Tib_ddqi.col(i) = i_R_j[0][i].col(2).cross((G_r_G.SCS_Tib - i_r_Oi_Oj[0][i]));
	}
	dd_r_SCS_Tib_ddqi.col(4) = i_R_j[0][4].col(2).cross((G_r_G.SCS_Tib - G_r_G.IC));
	//Prints for verification ----------------------------------------------------------------------------------------Borrar al final
	//std::cout << GlobalCnt << " dd_r_SCS_Tib_ddqi:\n" << dd_r_SCS_Tib_ddqi << std::endl;

	LMO4_rg4 = R.LMO4_R_G * (i_r_Oi_Oj[0][3] + i_R_j[0][3] * cal_data->rg.col(3) - G_r_G.LMO4);
	J_G4 = jacobian(R, dd_r_LMO4_ddqi, LMO4_rg4);
	J_G6 = jacobian(R, dd_r_SCS_Tib_ddqi, cal_data->SCS_Tib_rg);
	//Prints for verification ----------------------------------------------------------------------------------------Borrar al final
	/*
	std::cout << GlobalCnt << " LMO4_rg4:\n" << LMO4_rg4 << std::endl;
	std::cout << GlobalCnt << " J_G4:\n" << J_G4 << std::endl;
	std::cout << GlobalCnt << " J_G6:\n" << J_G6 << std::endl;
	*/
	Tau_G = J_G4.transpose() * (cal_data->G_g * cal_data->m(3)) + (J_G6.transpose() * (cal_data->G_g * cal_data->m(5)));
	LMO4_O5 = R.LMO4_R_G * i_r_Oi_Oj[0][4];								//Esta variable nunca se usa
	//Prints for verification ----------------------------------------------------------------------------------------Borrar al final
	//std::cout << GlobalCnt << " Tau_G:\n" << Tau_G << std::endl;

	G_r_G.Fext_r = G_r_G.Fext + G_r_AJC_AJC2;
	G_r_G.Gpie_r = G_r_G.Gpie + G_r_AJC_AJC2;
	//Prints for verification ----------------------------------------------------------------------------------------Borrar al final
	//std::cout << GlobalCnt << " G_r_G.Fext_r:\n" << G_r_G.Fext_r << std::endl;
	//std::cout << GlobalCnt << " G_r_G.Gpie_r:\n" << G_r_G.Gpie_r << std::endl;

	G_r_G.CPM = G_r_G.SCS_Tib + R.G_R_SCS_Tib * cal_data->SCS_Tib_CPM_Tib;
	G_r_G.CPL = G_r_G.SCS_Tib + R.G_R_SCS_Tib * cal_data->SCS_Tib_CPL_Tib;
	temp = (G_r_G.CPM + G_r_G.CPL) / 2;
	G_r_G.CPM = temp + ((G_r_G.CPM - temp).transpose() * u_n) * u_n;
	G_r_G.CPL = temp + ((G_r_G.CPL - temp).transpose() * u_n) * u_n;
	//Prints for verification ----------------------------------------------------------------------------------------Borrar al final
	//std::cout << GlobalCnt << " G_r_G.CPM:\n" << G_r_G.CPM << std::endl;
	//std::cout << GlobalCnt << " G_r_G.CPL:\n" << G_r_G.CPL << std::endl;

	/*----JACOBIANS AND GENERALIZED FORCES MISSING----*/
	for (int i = 1; i < 4; i++)
	{
		J_Gpie.col(i) = i_R_j[0][i].col(2).cross((G_r_G.Gpie_r - i_r_Oi_Oj[0][i]));
		J_Ipie.col(i) = i_R_j[0][i].col(2);
		J_Fext.col(i) = i_R_j[0][i].col(2).cross((G_r_G.Fext_r - i_r_Oi_Oj[0][i]));
	}
	J_Ipie.col(4) = i_R_j[0][4].col(2);
	J_Gpie.col(4) = i_R_j[0][5].col(2).cross((G_r_G.Gpie_r - G_r_G.IC));
	J_Fext.col(4) = i_R_j[0][5].col(2).cross((G_r_G.Fext_r - G_r_G.IC));
	J_Tau_ext = J_Ipie;
	//Prints for verification ----------------------------------------------------------------------------------------Borrar al final
	/*
	std::cout << GlobalCnt << " J_Gpie:\n" << J_Gpie << std::endl;
	std::cout << GlobalCnt << " J_Ipie:\n" << J_Ipie << std::endl;
	std::cout << GlobalCnt << " J_Fext:\n" << J_Fext << std::endl;
	*/

	/*----GENERALIZED FORCES RESULTS----*/
	Tau_G_pie = J_Gpie.transpose() * cal_data->G_g * cal_data->m_pie;
	Tau_Fext = J_Fext.transpose() * G_Fext;
	Tau_Mext = J_Tau_ext.transpose() * G_Mext;
	Tau_TOT = -Tau_G - Tau_G_pie - Tau_Fext - Tau_Mext;
	Tau_TOT_7(4) = Tau_TOT(4);
	Tau_TOT_4 = Tau_TOT(3);
	//Prints for verification ----------------------------------------------------------------------------------------Borrar al final
	/*
	std::cout << GlobalCnt << " Tau_G_pie:\n" << Tau_G_pie << std::endl;
	std::cout << GlobalCnt << " Tau_Fext:\n" << Tau_Fext << std::endl;
	std::cout << GlobalCnt << " Tau_Mext:\n" << Tau_Mext << std::endl;
	std::cout << GlobalCnt << " Tau_TOT:\n" << Tau_TOT << std::endl;
	*/
	/*----STORE THE RESULTS IN DATA STRUCTURE----*/
	Data.GenForceHipFlxExt(GlobalCnt) = Tau_TOT(3);
	Data.GenForceKneeFlxExtGrav(GlobalCnt) = Tau_G(4) + Tau_G_pie(4);
	Data.GenForceKneeFlxExtFext(GlobalCnt) = Tau_Fext(4);
	Data.GenForceKneeFlxExtMext(GlobalCnt) = Tau_Mext(4);
	Data.GenForceKneeFlxExt(GlobalCnt) = Tau_TOT_7(4);
	Data.q.col(GlobalCnt) = q;

	//Prints for verification FINAL RESULTS ----------------------------------------------------------------------------------------Borrar al final
	//std::cout << GlobalCnt << " cad.FlxExt:\t" << Data.ang.cad.FlxExt(GlobalCnt) << std::endl;
	//std::cout << GlobalCnt << " cad.AbdAduc:\t" << Data.ang.cad.AbdAduc(GlobalCnt) << std::endl;
	//std::cout << GlobalCnt << " cad.IntExt:\t" << Data.ang.cad.IntExt(GlobalCnt) << std::endl;
	//std::cout << GlobalCnt << " rod.FlxExt:\t" << Data.ang.rod.FlxExt(GlobalCnt) << std::endl;
	//std::cout << GlobalCnt << " pie.FlxExt:\t" << Data.ang.pie.FlxExt(GlobalCnt) << std::endl;
	//std::cout << GlobalCnt << " Tau_TOT_7:\n" << Tau_TOT_7 << std::endl;
	//std::cout << GlobalCnt << " Tau_TOT_4:\n" << Tau_TOT_4 << std::endl;

	//std::cout << GlobalCnt << " GenForceHipFlxExt:\t" << Data.GenForceHipFlxExt(GlobalCnt) << std::endl;
	//std::cout << GlobalCnt << " GenForceKneeFlxExtGrav:\t" << Data.GenForceKneeFlxExtGrav(GlobalCnt) << std::endl;
	//std::cout << GlobalCnt << " GenForceKneeFlxExtFext:\t" << Data.GenForceKneeFlxExtFext(GlobalCnt) << std::endl;
	//std::cout << GlobalCnt << " GenForceKneeFlxExtMext:\t" << Data.GenForceKneeFlxExtMext(GlobalCnt) << std::endl;
	//std::cout << GlobalCnt << " GenForceKneeFlxExt:\t" << Data.GenForceKneeFlxExt(GlobalCnt) << std::endl;
	//std::cout << GlobalCnt << " q:\t" << Data.q.col(GlobalCnt) << std::endl;
}

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

Matrix<double, 3, 5> jacobian(const PRDataStructures::R_struct& R, const Matrix<double, 3, Dynamic>& dd_r_X_ddqi, const Vector3d& InsPt) {
	int n = dd_r_X_ddqi.cols();
	Matrix<double, 3, 5> J = Matrix<double, 3, 5>::Zero();
	Matrix3d Rotate = R.G_R_LMO4;
	if (n == 5) Rotate = R.G_R_SCS_Tib;
	for (int i = 1; i < n; i++) {
		J.col(i) = dd_r_X_ddqi.col(i) + i_R_j[0][i].col(2).cross(Rotate * InsPt);
	}
	return J;
}

Matrix<double, 3, 5> jacobian(const PRDataStructures::R_struct& R, const Matrix<double, 3, Dynamic>& dd_r_X_ddqi, const Vector3d& InsPt, std::string location, const PRDataStructures::G_r_G_struct& G_r_G) {
	Matrix<double, 3, 5> J;
	if (location == "Foot") {
		int n = dd_r_X_ddqi.cols();
		J = Matrix<double, 3, 5>::Zero();
		for (int i = 1; i < n; i++) {
			J.col(i) = dd_r_X_ddqi.col(i) + i_R_j[0][i].col(2).cross(G_r_G.Cal2 + R.G_R_SCSpie * InsPt - G_r_G.SCS_Tib);
		}
	}
	else J = jacobian(R, dd_r_X_ddqi, InsPt);
	return J;
}

Matrix3d GetRotMat2V(Vector3d V1, Vector3d V2, int dir1, int dir2) {
	// dir3 is the 3rd dimension (not dir1 nor dir2)
	int dir3;
	int sgn;
	if (dir1 == 1 && dir2 == 2 || dir1 == 2 && dir2 == 1) dir3 = 3;
	else if (dir1 == 1 && dir2 == 3 || dir1 == 3 && dir2 == 1) dir3 = 2;
	else dir3 = 1;
	Matrix3d R;
	if (dir2 == dir1 + 1 || (dir1 == 3 && dir2 == 1)) sgn = 1;
	else sgn = -1;
	R.col(dir1 - 1) = unit(V1);
	R.col(dir3 - 1) = sgn * (unit(V1.cross(V2)));
	R.col(dir2 - 1) = sgn * (unit(R.col(dir3 - 1).cross(V1)));
	return R;
}