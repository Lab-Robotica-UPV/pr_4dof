#include "pr_lib/pr_limits.hpp"
#include "pr_lib/pr_model.hpp"

Eigen::Matrix<double,4,2> PRLimits::LimActuators(bool robot_5p) {

	double q13_lim_ini, q23_lim_ini, q33_lim_ini, q42_lim_ini;
	double q_desp_max;

	if (!robot_5p){
		q13_lim_ini=0.6537; q23_lim_ini=0.6474; q33_lim_ini=0.6502; q42_lim_ini=0.549;
		q_desp_max=0.28;
	}

	else{
		q13_lim_ini=0.781, q23_lim_ini=0.777, q33_lim_ini=0.779, q42_lim_ini=0.707;
		q_desp_max=0.38;
	} 

	Eigen::Matrix<double,4,2> Mlim_q_ind;

	Mlim_q_ind <<   q13_lim_ini, q13_lim_ini+q_desp_max,
			        q23_lim_ini, q23_lim_ini+q_desp_max,
			        q33_lim_ini, q33_lim_ini+q_desp_max,
			        q42_lim_ini, q42_lim_ini+q_desp_max;

	return Mlim_q_ind;
}

Eigen::Vector4d PRLimits::LimAngles(bool robot_5p) {
    
	int a_esf_max, a_uni_max;

	if (!robot_5p){
		a_esf_max=38; a_uni_max=90;
	}

	else{
		a_esf_max=45; a_uni_max=90;
	}

	Eigen::Vector4d Vlim_Angp;

	Vlim_Angp << a_esf_max, a_esf_max, a_esf_max, a_uni_max;

	return Vlim_Angp;
}

Eigen::Matrix<int,7,1> PRLimits::VerFactPos(const std::array<double, 4> &X, const std::vector<double> &RParam, bool robot_5p){

	// A partir de la posicion (Xm, Zm) y la orientacion (Theta y Psi) del 
	// centro de la plataforma se determina si es factible llegar a ese punto.
	// Para esto se analizan las longitudes de las patas y el angulo maximo de
    // las juntas esfericas. La función responde en sus primeras 4 filas si
    // los actuadores han rebasado el limite permitido (1) o si están por
    // debajo del limite minimo (-1). Las filas 5-7 indican si las juntas
    // esfericas han alcanzado el limite maximo (1). En el caso que todo este
    // bien el vector de salida estará lleno de ceros. 

	// Inicializacion de la variable
	Eigen::Matrix<int,7,1> V_verif = Eigen::Matrix<int,7,1>::Zero();

	// Variable for Inverse Kinematics
	Eigen::Matrix<double,4,3> solq;

	// Inverse Kinematics
	PRModel::InverseKinematics(solq, X, RParam);

	// Calculo del angulo entre las patas del robot y la plataforma movil
	double theta = X[2];
	Eigen::Vector4d solAngP = AngPatas(theta, solq);
	// Paso a grados
	solAngP *= 180/M_PI;

	// Limites
	Eigen::Matrix<double,4,2> Mlim_q_ind = LimActuators(robot_5p);
	Eigen::Vector4d Vlim_Angp = LimAngles(robot_5p);


	// Verificacion actuadores y juntas esfericas
	for (int i=0;i<4;i++){

		if (i<3){
			// Pata 1-3
			if (solq(i,2) > Mlim_q_ind(0,1)) V_verif(i) = 1;
			else if (solq(i,2) < Mlim_q_ind(i,0)) V_verif(i) = -1;

			// Juntas esfericas
			if (solAngP(i) > Vlim_Angp(i)) V_verif(i+4) = 1;

		}

		else{
			// Pata 4
			if (solq(i,1) > Mlim_q_ind(i,1)) V_verif(i) = 1;
			else if (solq(i,1) < Mlim_q_ind(i,0)) V_verif(i) = -1;

		}

	}

	return V_verif;	

}

Eigen::Vector4d PRLimits::AngPatas(const double &theta, const Eigen::Matrix<double,4,3> &q){

	// Determina los angulos entre las patas y la plataforma movil, a partir de
	// la posicion y orientacion de la plataforma movil

	// Dimensionamiento previo del vector de los angulos entre patas y plataforma
	Eigen::Vector4d solAngP = Eigen::Vector4d::Zero();

	// Soluciones de los angulos entre patas y plataforma
	solAngP(0) = acos(cos(q(0,0)) * sin(q(0,1)) * sin(theta) + sin(q(0,0)) * sin(q(0,1)) * cos(theta));
	solAngP(1) = acos(cos(q(1,0)) * sin(q(1,1)) * sin(theta) + sin(q(1,0)) * sin(q(1,1)) * cos(theta));
	solAngP(2) = acos(cos(q(2,0)) * sin(q(2,1)) * sin(theta) + sin(q(2,0)) * sin(q(2,1)) * cos(theta));
	solAngP(3) = acos(-sin(q(3,0)) * sin(theta) + cos(q(3,0)) * cos(theta));

	return solAngP;
}
