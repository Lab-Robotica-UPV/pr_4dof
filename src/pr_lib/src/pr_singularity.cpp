#include "pr_lib/pr_singularity.hpp"
#include <iostream>


Eigen::Matrix<double,6,1> PRSingularity::CalculateAngOts(
        const double &theta, const double &psi,
        const Eigen::Matrix<double,4,3> &q,
		Eigen::Matrix<double,6,4> &OTS_ant,
        const std::vector<double> &RParam,
        const int iter_OTS, const double tol_OTS)
{
	double error_OTS;
	int ci;
	//Solución de los OTS

	//Solución del angulo entre ejes instantaneos de dos OTS
	Eigen::Matrix<double, 6, 1> sol_AngOTS = Eigen::Matrix<double,6,1>::Zero();
	//Sistema ecuaciones para resolver cada OTS
	Eigen::Matrix<double, 5, 1> f_OTS = Eigen::Matrix<double,5,1>::Zero();
	//Jacobiano de las ecuaciones para resolver cada OTS
	Eigen::Matrix<double, 5, 5> J_OTS = Eigen::Matrix<double,5,5>::Zero();

	Eigen::Matrix<double,5,1> X_OTS = Eigen::Matrix<double,5,1>::Zero();

	Eigen::Matrix<double, 3, 1> ang_OTS_i, ang_OTS_j;

    // SOLUCION DE LOS CUATRO OTS
	for (int op=1; op<=4; op++){
		
		// Punto inicial para solucionar el sistema de ecuaciones para un OTS
		X_OTS(0) = OTS_ant(0,op-1); 
		X_OTS(1) = OTS_ant(1,op-1); 
		X_OTS(2) = OTS_ant(2,op-1); 
		X_OTS(3) = OTS_ant(3,op-1); 
		X_OTS(4) = OTS_ant(5,op-1);
		
		// Error inicial para la resolucion del sistema basada en el punto inicial
		EqOTS(f_OTS, X_OTS(0), X_OTS(1), X_OTS(2), X_OTS(3), X_OTS(4), theta, psi, q, op, RParam[6], RParam[7], RParam[8], RParam[9], RParam[10]);
		error_OTS = f_OTS.norm();

		// Iteracion inicial
		ci = 1;
		
		// Algoritmo de Newton Raphson
		while (error_OTS>tol_OTS){
			// Funcion con las ecuaciones que determinan los componentes de un OTS
			EqOTS(f_OTS, X_OTS(0), X_OTS(1), X_OTS(2), X_OTS(3), X_OTS(4), theta, psi, q, op, RParam[6], RParam[7], RParam[8], RParam[9], RParam[10]);
			
			// Error de la solucion actual
			error_OTS = f_OTS.norm();
			
			// Jacobiano del sistema de ecuaciones para un OTS
			EqOTSJacobian(J_OTS, X_OTS(0), X_OTS(1), X_OTS(2), theta, psi, q, op, RParam[6], RParam[7], RParam[8], RParam[9], RParam[10]);
			
			// Calculo de la nueva solucion
			//Xn_OTS = X_OTS - linSolve(J_OTS, f_OTS);
			X_OTS = X_OTS - J_OTS.partialPivLu().solve(f_OTS);
			//Xn_OTS = X_OTS - J_OTS.inverse()*f_OTS;

			// Actualizo la solucion de un OTS
			//cout << X_OTS.transpose() << endl;
			
			// Incremento el contador de iteraciones
			ci++;
			// Condicion para evitar bucles infinitos
			if (ci>iter_OTS) 
                break;
		}
		
		// Almaceno la solucion del OTS seleccionado por op y actualizo la variable para el siguiente Ts
		OTS_ant(0,op-1)=X_OTS(0);
		OTS_ant(1,op-1)=X_OTS(1);
		OTS_ant(2,op-1)=X_OTS(2);
		OTS_ant(3,op-1)=X_OTS(3);
		OTS_ant(4,op-1)=0;
		OTS_ant(5,op-1)=X_OTS(4);
    }
	
    // ANGULO ENTRE DOS OUTPUT TWIST SCREW CALCULADOS (Componentes angular y lineal)
	// Indice donde se almacenara el angulo
	int k=0;
	// Contador primer OTS
	for (int i=0;i<3;i++){
		// Contador segundo OTS
		for (int j=i+1;j<4;j++){
			// OTS - Componente Angular
			ang_OTS_i = (OTS_ant.col(i)).head(3);
			ang_OTS_j = (OTS_ant.col(j)).head(3);
			sol_AngOTS(k) = acos(ang_OTS_i.dot(ang_OTS_j)/(ang_OTS_i.norm()*ang_OTS_j.norm()))*180/M_PI;
			
			if (sol_AngOTS(k)>90) sol_AngOTS(k)=180-sol_AngOTS(k);

			// Incremento el indice de almacenamiento del angulo entre dos OTS
			k++;

		}
	}
	
	return sol_AngOTS;
}

void PRSingularity::EqOTSJacobian(
		Eigen::Matrix<double,5,5> &J,
        const double &wx, const double &wy, const double &wz, 
        const double &theta, const double &psi, 
        const Eigen::Matrix<double,4,3> &q, 
        const int &op, 
        const double &Rm1, const double &Rm2, const double &Rm3, 
        const double &betaMD, const double &betaMI)
{
	double q11 = q(0,0), q12 = q(0,1), q13 = q(0,2);
	double q21 = q(1,0), q22 = q(1,1), q23 = q(1,2);
	double q31 = q(2,0), q32 = q(2,1), q33 = q(2,2);
	double q41 = q(3,0), q42 = q(3,1); 

	switch (op){
		case 1:
		
			J(0,0) = -Rm2 * (cos(psi) * sin(theta) * cos(betaMD) * cos(q22) - cos(psi) * sin(q21) * sin(betaMD) * sin(q22) - sin(psi) * sin(theta) * sin(betaMD) * cos(q22) - sin(psi) * sin(q21) * cos(betaMD) * sin(q22));
			J(0,1) = -sin(q22) * Rm2 * (cos(psi) * cos(theta) * sin(q21) * cos(betaMD) + cos(psi) * sin(theta) * cos(betaMD) * cos(q21) - sin(psi) * cos(theta) * sin(q21) * sin(betaMD) - sin(psi) * sin(theta) * sin(betaMD) * cos(q21));
			J(0,2) = -Rm2 * (cos(psi) * cos(theta) * cos(betaMD) * cos(q22) + cos(psi) * sin(betaMD) * cos(q21) * sin(q22) - sin(psi) * cos(theta) * sin(betaMD) * cos(q22) + sin(psi) * cos(betaMD) * cos(q21) * sin(q22));
			J(0,3) = cos(q21) * sin(q22);
			J(0,4) = sin(q21) * sin(q22);
			
			J(1,0) = -Rm3 * (cos(psi) * sin(theta) * cos(betaMI) * cos(q32) + cos(psi) * sin(q31) * sin(betaMI) * sin(q32) + sin(psi) * sin(theta) * sin(betaMI) * cos(q32) - sin(psi) * sin(q31) * cos(betaMI) * sin(q32));
			J(1,1) = -sin(q32) * Rm3 * (cos(psi) * cos(theta) * sin(q31) * cos(betaMI) + cos(psi) * sin(theta) * cos(betaMI) * cos(q31) + sin(psi) * cos(theta) * sin(q31) * sin(betaMI) + sin(psi) * sin(theta) * sin(betaMI) * cos(q31));
			J(1,2) = -Rm3 * (cos(psi) * cos(theta) * cos(betaMI) * cos(q32) - cos(psi) * sin(betaMI) * cos(q31) * sin(q32) + sin(psi) * cos(theta) * sin(betaMI) * cos(q32) + sin(psi) * cos(betaMI) * cos(q31) * sin(q32));
			J(1,3) = cos(q31) * sin(q32);
			J(1,4) = sin(q31) * sin(q32);

			J(2,0) = 0; J(2,1) = 0; J(2,2) = 0; J(2,3) = -sin(q41); J(2,4) = cos(q41);
			J(3,0) = 1; J(3,1) = 0; J(3,2) = -(sin(theta)/cos(theta)); J(3,3) = 0; J(3,4) = 0;
			J(4,0) = 2*wx; J(4,1) = 2*wy; J(4,2) = 2*wz; J(4,3) = 0; J(4,4) = 0;
			break;

		case 2:

			J(0,0) = -Rm1 * (sin(q11) * sin(q12) * sin(psi) - sin(theta) * cos(q12) * cos(psi));
			J(0,1) = sin(q12) * cos(psi) * Rm1 * (sin(q11) * cos(theta) + sin(theta) * cos(q11));
			J(0,2) = Rm1 * (cos(theta) * cos(q12) * cos(psi) + cos(q11) * sin(q12) * sin(psi));
			J(0,3) = cos(q11) * sin(q12);
			J(0,4) = sin(q11) * sin(q12);
			
			J(1,0) = -Rm3 * (cos(psi) * sin(theta) * cos(betaMI) * cos(q32) + cos(psi) * sin(q31) * sin(betaMI) * sin(q32) + sin(psi) * sin(theta) * sin(betaMI) * cos(q32) - sin(psi) * sin(q31) * cos(betaMI) * sin(q32));
			J(1,1) = -sin(q32) * Rm3 * (cos(psi) * cos(theta) * sin(q31) * cos(betaMI) + cos(psi) * sin(theta) * cos(betaMI) * cos(q31) + sin(psi) * cos(theta) * sin(q31) * sin(betaMI) + sin(psi) * sin(theta) * sin(betaMI) * cos(q31));
			J(1,2) = -Rm3 * (cos(psi) * cos(theta) * cos(betaMI) * cos(q32) - cos(psi) * sin(betaMI) * cos(q31) * sin(q32) + sin(psi) * cos(theta) * sin(betaMI) * cos(q32) + sin(psi) * cos(betaMI) * cos(q31) * sin(q32));
			J(1,3) = cos(q31) * sin(q32);
			J(1,4) = sin(q31) * sin(q32);

			J(2,0) = 0; J(2,1) = 0; J(2,2) = 0; J(2,3) = -sin(q41); J(2,4) = cos(q41);
			J(3,0) = 1; J(3,1) = 0; J(3,2) = -(sin(theta)/cos(theta)); J(3,3) = 0; J(3,4) = 0;
			J(4,0) = 2*wx; J(4,1) = 2*wy; J(4,2) = 2*wz; J(4,3) = 0; J(4,4) = 0;

			break;

		case 3:

			J(0,0) = -Rm1 * (sin(q11) * sin(q12) * sin(psi) - sin(theta) * cos(q12) * cos(psi));
			J(0,1) = sin(q12) * cos(psi) * Rm1 * (sin(q11) * cos(theta) + sin(theta) * cos(q11));
			J(0,2) = Rm1 * (cos(theta) * cos(q12) * cos(psi) + cos(q11) * sin(q12) * sin(psi));
			J(0,3) = cos(q11) * sin(q12);
			J(0,4) = sin(q11) * sin(q12);
			
			J(1,0) = -Rm2 * (cos(psi) * sin(theta) * cos(betaMD) * cos(q22) - cos(psi) * sin(q21) * sin(betaMD) * sin(q22) - sin(psi) * sin(theta) * sin(betaMD) * cos(q22) - sin(psi) * sin(q21) * cos(betaMD) * sin(q22));
			J(1,1) = -sin(q22) * Rm2 * (cos(psi) * cos(theta) * sin(q21) * cos(betaMD) + cos(psi) * sin(theta) * cos(betaMD) * cos(q21) - sin(psi) * cos(theta) * sin(q21) * sin(betaMD) - sin(psi) * sin(theta) * sin(betaMD) * cos(q21));
			J(1,2) = -Rm2 * (cos(psi) * cos(theta) * cos(betaMD) * cos(q22) + cos(psi) * sin(betaMD) * cos(q21) * sin(q22) - sin(psi) * cos(theta) * sin(betaMD) * cos(q22) + sin(psi) * cos(betaMD) * cos(q21) * sin(q22));
			J(1,3) = cos(q21) * sin(q22);
			J(1,4) = sin(q21) * sin(q22);

			J(2,0) = 0; J(2,1) = 0; J(2,2) = 0; J(2,3) = -sin(q41); J(2,4) = cos(q41);
			J(3,0) = 1; J(3,1) = 0; J(3,2) = -(sin(theta)/cos(theta)); J(3,3) = 0; J(3,4) = 0;
			J(4,0) = 2*wx; J(4,1) = 2*wy; J(4,2) = 2*wz; J(4,3) = 0; J(4,4) = 0;

			break;

		case 4:

			J(0,0) = -Rm1 * (sin(q11) * sin(q12) * sin(psi) - sin(theta) * cos(q12) * cos(psi));
			J(0,1) = sin(q12) * cos(psi) * Rm1 * (sin(q11) * cos(theta) + sin(theta) * cos(q11));
			J(0,2) = Rm1 * (cos(theta) * cos(q12) * cos(psi) + cos(q11) * sin(q12) * sin(psi));
			J(0,3) = cos(q11) * sin(q12);
			J(0,4) = sin(q11) * sin(q12);
			
			J(1,0) = -Rm2 * (cos(psi) * sin(theta) * cos(betaMD) * cos(q22) - cos(psi) * sin(q21) * sin(betaMD) * sin(q22) - sin(psi) * sin(theta) * sin(betaMD) * cos(q22) - sin(psi) * sin(q21) * cos(betaMD) * sin(q22));
			J(1,1) = -sin(q22) * Rm2 * (cos(psi) * cos(theta) * sin(q21) * cos(betaMD) + cos(psi) * sin(theta) * cos(betaMD) * cos(q21) - sin(psi) * cos(theta) * sin(q21) * sin(betaMD) - sin(psi) * sin(theta) * sin(betaMD) * cos(q21));
			J(1,2) = -Rm2 * (cos(psi) * cos(theta) * cos(betaMD) * cos(q22) + cos(psi) * sin(betaMD) * cos(q21) * sin(q22) - sin(psi) * cos(theta) * sin(betaMD) * cos(q22) + sin(psi) * cos(betaMD) * cos(q21) * sin(q22));
			J(1,3) = cos(q21) * sin(q22);
			J(1,4) = sin(q21) * sin(q22);

			J(2,0) = -Rm3 * (cos(psi) * sin(theta) * cos(betaMI) * cos(q32) + cos(psi) * sin(q31) * sin(betaMI) * sin(q32) + sin(psi) * sin(theta) * sin(betaMI) * cos(q32) - sin(psi) * sin(q31) * cos(betaMI) * sin(q32)); 
			J(2,1) = -sin(q32) * Rm3 * (cos(psi) * cos(theta) * sin(q31) * cos(betaMI) + cos(psi) * sin(theta) * cos(betaMI) * cos(q31) + sin(psi) * cos(theta) * sin(q31) * sin(betaMI) + sin(psi) * sin(theta) * sin(betaMI) * cos(q31)); 
			J(2,2) = -Rm3 * (cos(psi) * cos(theta) * cos(betaMI) * cos(q32) - cos(psi) * sin(betaMI) * cos(q31) * sin(q32) + sin(psi) * cos(theta) * sin(betaMI) * cos(q32) + sin(psi) * cos(betaMI) * cos(q31) * sin(q32)); 
			J(2,3) = cos(q31) * sin(q32); 
			J(2,4) = sin(q31) * sin(q32);

			J(3,0) = 1; J(3,1) = 0; J(3,2) = -(sin(theta)/cos(theta)); J(3,3) = 0; J(3,4) = 0;
			J(4,0) = 2*wx; J(4,1) = 2*wy; J(4,2) = 2*wz; J(4,3) = 0; J(4,4) = 0;

			break;

	}
}


void PRSingularity::EqOTS(
	   Eigen::Matrix<double,5,1> &f,
       const double &wx, const double &wy, const double &wz, 
       const double &vx, const double &vz, 
       const double &theta, const double &psi, 
       const Eigen::Matrix<double,4,3> &q, 
       const int &op, 
       const double &Rm1, const double &Rm2, const double &Rm3, 
       const double &betaMD, const double &betaMI
)
{
    //A partir de la orientacion (theta y psi) del centro de la plataforma del robot paralelo y de la solucion de la cinematica inversa determinamos las componentes del Output Screw Salida (OTS). El OTS a determinar se selecciona mediante la variable op.
	
	f(3) = wx - (sin(theta)/cos(theta))*wz;
	f(4) = pow(wx,2) + pow(wy,2) + pow(wz,2) -1;

	double q11 = q(0,0), q12 = q(0,1), q13 = q(0,2);
	double q21 = q(1,0), q22 = q(1,1), q23 = q(1,2);
	double q31 = q(2,0), q32 = q(2,1), q33 = q(2,2);
	double q41 = q(3,0), q42 = q(3,1); 

	switch (op){
	
		case 1:
			f(0) = cos(q21) * sin(q22) * vx + sin(q21) * sin(q22) * vz - Rm2 * (cos(psi) * cos(betaMD) * cos(q22) * sin(theta) - cos(psi) * sin(betaMD) * sin(q22) * sin(q21) - sin(psi) * cos(betaMD) * sin(q22) * sin(q21) - sin(psi) * sin(betaMD) * cos(q22) * sin(theta)) * wx - sin(q22) * Rm2 * (cos(psi) * cos(betaMD) * cos(q21) * sin(theta) + cos(psi) * cos(betaMD) * cos(theta) * sin(q21) - sin(psi) * sin(betaMD) * cos(q21) * sin(theta) - sin(psi) * sin(betaMD) * cos(theta) * sin(q21)) * wy - Rm2 * (cos(psi) * cos(betaMD) * cos(q22) * cos(theta) + cos(psi) * sin(betaMD) * cos(q21) * sin(q22) + sin(psi) * cos(betaMD) * cos(q21) * sin(q22) - sin(psi) * sin(betaMD) * cos(q22) * cos(theta)) * wz;
			f(1) = cos(q31) * sin(q32) * vx + sin(q31) * sin(q32) * vz - Rm3 * (cos(psi) * sin(theta) * cos(betaMI) * cos(q32) + cos(psi) * sin(q31) * sin(betaMI) * sin(q32) + sin(psi) * sin(theta) * sin(betaMI) * cos(q32) - sin(psi) * sin(q31) * cos(betaMI) * sin(q32)) * wx - sin(q32) * Rm3 * (cos(psi) * cos(theta) * sin(q31) * cos(betaMI) + cos(psi) * sin(theta) * cos(betaMI) * cos(q31) + sin(psi) * cos(theta) * sin(q31) * sin(betaMI) + sin(psi) * sin(theta) * sin(betaMI) * cos(q31)) * wy - Rm3 * (cos(psi) * cos(theta) * cos(betaMI) * cos(q32) - cos(psi) * sin(betaMI) * cos(q31) * sin(q32) + sin(psi) * cos(theta) * sin(betaMI) * cos(q32) + sin(psi) * cos(betaMI) * cos(q31) * sin(q32)) * wz;
			f(2) = -sin(q41) * vx + cos(q41) * vz;
			break;

		case 2:
			f(0) = cos(q11) * sin(q12) * vx + sin(q11) * sin(q12) * vz - Rm1 * (sin(q11) * sin(q12) * sin(psi) - sin(theta) * cos(q12) * cos(psi)) * wx + sin(q12) * cos(psi) * Rm1 * (sin(q11) * cos(theta) + sin(theta) * cos(q11)) * wy + Rm1 * (cos(theta) * cos(q12) * cos(psi) + cos(q11) * sin(q12) * sin(psi)) * wz;
			f(1) = cos(q31) * sin(q32) * vx + sin(q31) * sin(q32) * vz - Rm3 * (cos(psi) * sin(theta) * cos(betaMI) * cos(q32) + cos(psi) * sin(q31) * sin(betaMI) * sin(q32) + sin(psi) * sin(theta) * sin(betaMI) * cos(q32) - sin(psi) * sin(q31) * cos(betaMI) * sin(q32)) * wx - sin(q32) * Rm3 * (cos(psi) * cos(theta) * sin(q31) * cos(betaMI) + cos(psi) * sin(theta) * cos(betaMI) * cos(q31) + sin(psi) * cos(theta) * sin(q31) * sin(betaMI) + sin(psi) * sin(theta) * sin(betaMI) * cos(q31)) * wy - Rm3 * (cos(psi) * cos(theta) * cos(betaMI) * cos(q32) - cos(psi) * sin(betaMI) * cos(q31) * sin(q32) + sin(psi) * cos(theta) * sin(betaMI) * cos(q32) + sin(psi) * cos(betaMI) * cos(q31) * sin(q32)) * wz;
			f(2) = -sin(q41) * vx + cos(q41) * vz;
			break;

		case 3:
			f(0) = cos(q11) * sin(q12) * vx + sin(q11) * sin(q12) * vz - Rm1 * (sin(q11) * sin(q12) * sin(psi) - sin(theta) * cos(q12) * cos(psi)) * wx + sin(q12) * cos(psi) * Rm1 * (sin(q11) * cos(theta) + sin(theta) * cos(q11)) * wy + Rm1 * (cos(theta) * cos(q12) * cos(psi) + cos(q11) * sin(q12) * sin(psi)) * wz;
			f(1) = cos(q21) * sin(q22) * vx + sin(q21) * sin(q22) * vz - Rm2 * (cos(psi) * sin(theta) * cos(betaMD) * cos(q22) - cos(psi) * sin(q21) * sin(betaMD) * sin(q22) - sin(psi) * sin(theta) * sin(betaMD) * cos(q22) - sin(psi) * sin(q21) * cos(betaMD) * sin(q22)) * wx - sin(q22) * Rm2 * (cos(psi) * cos(theta) * sin(q21) * cos(betaMD) + cos(psi) * sin(theta) * cos(betaMD) * cos(q21) - sin(psi) * cos(theta) * sin(q21) * sin(betaMD) - sin(psi) * sin(theta) * sin(betaMD) * cos(q21)) * wy - Rm2 * (cos(psi) * cos(theta) * cos(betaMD) * cos(q22) + cos(psi) * sin(betaMD) * cos(q21) * sin(q22) - sin(psi) * cos(theta) * sin(betaMD) * cos(q22) + sin(psi) * cos(betaMD) * cos(q21) * sin(q22)) * wz;
			f(2) = -sin(q41) * vx + cos(q41) * vz;
			break;

		case 4:
			f(0) = cos(q11) * sin(q12) * vx + sin(q11) * sin(q12) * vz - Rm1 * (sin(q11) * sin(q12) * sin(psi) - sin(theta) * cos(q12) * cos(psi)) * wx + sin(q12) * cos(psi) * Rm1 * (sin(q11) * cos(theta) + sin(theta) * cos(q11)) * wy + Rm1 * (cos(theta) * cos(q12) * cos(psi) + cos(q11) * sin(q12) * sin(psi)) * wz;
			f(1) =  cos(q21) * sin(q22) * vx + sin(q21) * sin(q22) * vz - Rm2 * (cos(psi) * sin(theta) * cos(betaMD) * cos(q22) - cos(psi) * sin(q21) * sin(betaMD) * sin(q22) - sin(psi) * sin(theta) * sin(betaMD) * cos(q22) - sin(psi) * sin(q21) * cos(betaMD) * sin(q22)) * wx - sin(q22) * Rm2 * (cos(psi) * cos(theta) * sin(q21) * cos(betaMD) + cos(psi) * sin(theta) * cos(betaMD) * cos(q21) - sin(psi) * cos(theta) * sin(q21) * sin(betaMD) - sin(psi) * sin(theta) * sin(betaMD) * cos(q21)) * wy - Rm2 * (cos(psi) * cos(theta) * cos(betaMD) * cos(q22) + cos(psi) * sin(betaMD) * cos(q21) * sin(q22) - sin(psi) * cos(theta) * sin(betaMD) * cos(q22) + sin(psi) * cos(betaMD) * cos(q21) * sin(q22)) * wz;
			f(2) = cos(q31) * sin(q32) * vx + sin(q31) * sin(q32) * vz - Rm3 * (cos(psi) * sin(theta) * cos(betaMI) * cos(q32) + cos(psi) * sin(q31) * sin(betaMI) * sin(q32) + sin(psi) * sin(theta) * sin(betaMI) * cos(q32) - sin(psi) * sin(q31) * cos(betaMI) * sin(q32)) * wx - sin(q32) * Rm3 * (cos(psi) * cos(theta) * sin(q31) * cos(betaMI) + cos(psi) * sin(theta) * cos(betaMI) * cos(q31) + sin(psi) * cos(theta) * sin(q31) * sin(betaMI) + sin(psi) * sin(theta) * sin(betaMI) * cos(q31)) * wy - Rm3 * (cos(psi) * cos(theta) * cos(betaMI) * cos(q32) - cos(psi) * sin(betaMI) * cos(q31) * sin(q32) + sin(psi) * cos(theta) * sin(betaMI) * cos(q32) + sin(psi) * cos(betaMI) * cos(q31) * sin(q32)) * wz;
			break;

	}
}

Eigen::VectorXd PRSingularity::PosiblesAngOTS(
        const Eigen::Vector4d &q_ind_mod,
        const Eigen::Vector4d &X_cart,
        const Eigen::Matrix<double,6,4> &OTS_med,
        const double &des_qind,
        const Eigen::Vector2i i_qind,
        const Eigen::MatrixXi &minc_des,
        const int &ncomb,
		const double &tol_FK,
		const int &iter_FK,
        const int &iter_OTS,
		const double &tol_OTS,
        const std::vector<double> &RParam,
        const Eigen::Matrix<double,4,2> &Mlim_q_ind,
		const Eigen::Vector4d &Vlim_angp
)

{
	// ANGULO OMEGA PARA LOS OTS INVOLUCRADOS EN LA SINGULARIDAD PARA CADA POSIBLE NUEVA REFERENCIA MODIFICADA
	// Inicializacion del vector para almacenar angulos OMEGA
	Eigen::VectorXd solAngOTS_mod = Eigen::VectorXd::Zero(ncomb);
	Eigen::Vector4d qa;

	// Lazo para calculo en cada posible modificacion
	for (int c_comb=0; c_comb<ncomb; c_comb++){
		// Selecciono la referencia modificada para analizar
		qa = q_ind_mod;

		//Añado los incrementos para la combinacion selecionada por c_comb, solo para las patas involucradas en la singularidad 
		qa(i_qind(0)) += des_qind*minc_des(i_qind(0),c_comb);
		qa(i_qind(1)) += des_qind*minc_des(i_qind(1),c_comb);

		if ((qa(0)>Mlim_q_ind(0,0) && qa(0)<Mlim_q_ind(0,1)) && (qa(1)>Mlim_q_ind(1,0) && qa(1)<Mlim_q_ind(1,1)) && 
			(qa(2)>Mlim_q_ind(2,0) && qa(2)<Mlim_q_ind(2,1)) && (qa(3)>Mlim_q_ind(3,0) && qa(3)<Mlim_q_ind(3,1))) {

			std::array<double,4> q_sol;
			std::vector<double> x_pose = {X_cart(0),X_cart(1),X_cart(2),X_cart(3)};
			std::array<double,4> X;
			double x_m, z_m, theta, psi;
			Eigen::Matrix<double,4,3> q;

			for(int i=0; i<4; i++) {
				q_sol[i] = qa(i);
			}

			// RESOLUCION DE LA CINEMATICA DIRECTA-POSICION
			// Vector de posicion y orientacion medida de la plataforma
			X = PRModel::ForwardKinematics(q_sol,x_pose,RParam,tol_FK,iter_FK);

			// Posicion alcanzada por la plataforma para la referencia modificada
			x_m = X[0]; z_m = X[1]; theta = X[2]; psi = X[3];
					
			// RESOLUCION DE LA CINEMATICA INVERSA POSICION
			// Dimensionamiento previo de q
			PRModel::InverseKinematics(q,X,RParam);

			// RESOLUCION DE LOS OTS INVOLUCRADOS EN LA SINGULARIDAD
			// Matriz para los dos OTS buscados
			Eigen::Matrix<double,6,2> solOTS = Eigen::Matrix<double,6,2>::Zero();
			Eigen::Vector4d solAngP = Eigen::Vector4d::Zero();

			solAngP(0) = acos(cos(q(0)) * sin(q(1)) * sin(theta) + sin(q(0)) * sin(q(1)) * cos(theta));
			solAngP(1) = acos(cos(q(3)) * sin(q(4)) * sin(theta) + sin(q(3)) * sin(q(4)) * cos(theta));
			solAngP(2) = acos(cos(q(6)) * sin(q(7)) * sin(theta) + sin(q(6)) * sin(q(7)) * cos(theta));
			solAngP(3) = acos(-sin(q(9)) * sin(theta) + cos(q(9)) * cos(theta));
				
			if ((q(0,2)>Mlim_q_ind(0,0) && q(0,2)<Mlim_q_ind(0,1)) && (q(1,2)>Mlim_q_ind(1,0) && q(1,2)<Mlim_q_ind(1,1)) && 
				(q(2,2)>Mlim_q_ind(2,0) && q(2,2)<Mlim_q_ind(2,1)) && (q(3,1)>Mlim_q_ind(3,0) && q(3,1)<Mlim_q_ind(3,1)) && 
				solAngP(0)<Vlim_angp(0) && solAngP(1)<Vlim_angp(1) && solAngP(2)<Vlim_angp(2) && solAngP(3)<Vlim_angp(3)) {

				Eigen::Matrix<double,3,1> ang_OTS_1, ang_OTS_2;
				Eigen::Matrix<double,5,1> X_OTS;
				Eigen::Matrix<double,5,1> Xn_OTS;

				Eigen::Matrix<double,5,1> f_OTS;
				Eigen::Matrix<double,5,5> J_OTS;

				double error_OTS;

				// Lazo para resolver los dos OTS de la singularidad
				for (int c_OTS=0; c_OTS<2; c_OTS++){
					// Punto inicial para solucionar el sistema de ecuaciones para un OTS
					X_OTS << OTS_med(0,i_qind(c_OTS)), OTS_med(1,i_qind(c_OTS)), OTS_med(2,i_qind(c_OTS)), OTS_med(3,i_qind(c_OTS)), OTS_med(5,i_qind(c_OTS));
					// Error inicial para la resolucion del sistema
					EqOTS(f_OTS, X_OTS(0), X_OTS(1), X_OTS(2), X_OTS(3), X_OTS(4), theta, psi, q, i_qind(c_OTS)+1, RParam[6], RParam[7], RParam[8], RParam[9], RParam[10]);
					error_OTS = f_OTS.norm();
					// Iteracion inicial
					int ci = 1;
						
					// Algoritmo de Newton Raphson
					while (error_OTS>tol_OTS){
						// Funcion con las ecuaciones que determinan los componentes de un OTS
						EqOTS(f_OTS, X_OTS(0), X_OTS(1), X_OTS(2), X_OTS(3), X_OTS(4), theta, psi, q, i_qind(c_OTS)+1, RParam[6], RParam[7], RParam[8], RParam[9], RParam[10]);
						// Error de la solucion actual
						error_OTS = f_OTS.norm();
						// Jacobiano del sistema de ecuaciones para un OTS
						EqOTSJacobian(J_OTS, X_OTS(0), X_OTS(1), X_OTS(2), theta, psi, q, i_qind(c_OTS)+1, RParam[6], RParam[7], RParam[8], RParam[9], RParam[10]);
						// Calculo de la nueva solucion
						//Xn_OTS = X_OTS - linSolve(J_OTS, f_OTS);
						Xn_OTS = X_OTS - J_OTS.partialPivLu().solve(f_OTS);
						//Xn_OTS = X_OTS - J_OTS.inverse()*f_OTS;
						// Actualizo la solucion de un OTS
						X_OTS = Xn_OTS;
						// Incremento el contador de iteraciones
						ci++;
						// Condicion para evitar bucles infinitos
						if (ci>iter_OTS) break;

					}

					// Almaceno la solucion del OTS seleccionado por c_OTS
					solOTS(0,c_OTS) = X_OTS(0);
					solOTS(1,c_OTS) = X_OTS(1);
					solOTS(2,c_OTS) = X_OTS(2);
					solOTS(3,c_OTS) = X_OTS(3);
					solOTS(4,c_OTS) = 0;
					solOTS(5,c_OTS) = X_OTS(4);
				}

				// Nuevo angulo OMEGA para la referencia modificada
				ang_OTS_1 = (solOTS.col(0)).head(3);
				ang_OTS_2 = (solOTS.col(1)).head(3);
				solAngOTS_mod(c_comb) = acos(ang_OTS_1.dot(ang_OTS_2)/(ang_OTS_1.norm()*ang_OTS_2.norm()))*180/M_PI;

				if (solAngOTS_mod(c_comb) > 90) solAngOTS_mod(c_comb) = 180 - solAngOTS_mod(c_comb);					
			}
		}
	}

	return solAngOTS_mod;			
}

Eigen::Vector4d PRSingularity::CalculateQindModReleaser(
        const Eigen::Vector4d &X_cart, 
        const Eigen::Vector4d &q_ref, 
        const Eigen::Matrix<double,6,1> &angOTS, 
        const Eigen::Matrix<double,6,4> &solOTS,
		const Eigen::MatrixXi &minc_des,
		const double &fj_det,
		const std::vector<double> &RParam,
		Eigen::Vector4i &vc_des,
		const Eigen::Matrix<double,4,2> &Mlim_q_ind,
		const Eigen::Vector4d &Vlim_angp,
		const double &des_qind,
		const double &lmin_Ang_OTS,
		const double &lmin_FJac,
		const double &tol_FK,
		const int &iter_FK,
		const double &tol_OTS,
		const double &iter_OTS,
		const bool &enable
)
{
	Eigen::Vector4d q_ind_mod = q_ref+des_qind*vc_des.cast<double>();
	//std:: cout << q_ind_mod.transpose() << std::endl;
	std:: cout << X_cart.transpose() << std::endl;
	
	// Activacio0n de las modificaciones de las referencias
	if (enable){

		std::cout << "Holasing 0" << std::endl;

		double minAng_OTS, maxAng_OTS_mod;
		Eigen::Vector2i i_qind = Eigen::Vector2i::Zero();

		// Minimo angulo entre un par de ejes instantaneos de los OTS medidos
		minAng_OTS = angOTS.minCoeff();
		std::cout << minAng_OTS;
		std::cout << abs(fj_det) << std::endl;

		// Condicion para modificar la referencia
		if (minAng_OTS<lmin_Ang_OTS || abs(fj_det) < lmin_FJac){

			std::cout << "Holasing 1" << std::endl;
			
			// Identifico las patas que causan el minimo angulo de OTS
			// OJO! Se les resta una unidad (un 0 es la pata 1 y un 3 es la pata 4)
			if (minAng_OTS == angOTS(0)){ i_qind(0)=0; i_qind(1) = 1;}
			else if (minAng_OTS == angOTS(1)){ i_qind(0)=0; i_qind(1) = 2;}
			else if (minAng_OTS == angOTS(2)){ i_qind(0)=0; i_qind(1) = 3;}
			else if (minAng_OTS == angOTS(3)){ i_qind(0)=1; i_qind(1) = 2;}
			else if (minAng_OTS == angOTS(4)){ i_qind(0)=1; i_qind(1) = 3;}
			else if (minAng_OTS == angOTS(5)){ i_qind(0)=2; i_qind(1) = 3;}
		
			//Numero de posibles modificaciones
			int ncomb = minc_des.cols();

			Eigen::VectorXd solAngOTS_mod = PosiblesAngOTS(q_ind_mod, X_cart, solOTS, des_qind, i_qind, minc_des, 
				ncomb, tol_FK, iter_FK, iter_OTS, tol_OTS, RParam, Mlim_q_ind, Vlim_angp);

			
			// REFERENCIA MODIFICADA QUE PRODUCIRA EL MAXIMO ANGULO OMEGA
			// Determino el maximo angulo OMEGA
			maxAng_OTS_mod = solAngOTS_mod.maxCoeff();
			// Cargo la combinacion del mejor
			//if (maxAng_OTS_mod>minAng_OTS){
				for (int i=0; i<ncomb; i++){
					if (solAngOTS_mod(i) == maxAng_OTS_mod){
						vc_des(i_qind(0)) += minc_des(0,i);
						vc_des(i_qind(1)) += minc_des(1,i);
						std::cout << "Holasing 2" << std::endl;
						break;
					}
				}

				// Actualizo la referencia modificada
				q_ind_mod = q_ref+des_qind*vc_des.cast<double>();

			//}
		}
	}
	return q_ind_mod;
}

Eigen::Vector4d PRSingularity::CalculateQindModEvader(
        const Eigen::Vector4d &X_cart, 
        const Eigen::Vector4d &q_ref, 
        const Eigen::Matrix<double,6,1> &angOTS_ref, 
		const Eigen::Matrix<double,6,1> &angOTS_med,
        const Eigen::Matrix<double,6,4> &solOTS_med,
		const Eigen::MatrixXi &minc_des,
		const double &det_JDir_med,
		const double &det_JDir_ref,
		const std::vector<double> &RParam,
		Eigen::Vector4i &vc_des,
		const Eigen::Matrix<double,4,2> &Mlim_q_ind,
		const Eigen::Vector4d &Vlim_angp,
		const double &des_qind,
		const double &lmin_Ang_OTS,
		const double &lmin_JDir,
		const double &tol_FK,
		const int &iter_FK,
		const double &tol_OTS,
		const double &iter_OTS
)
{

	// Matriz que identifica las patas involucradas en el angulo OMEGA
	Eigen::Matrix<int,6,2> mi_qind_des;
	mi_qind_des << 1, 2,
				   1, 3,
				   1, 4,
				   2, 3,
				   2, 4,
				   3, 4;
	
	//Numero de posibles modificaciones
	int ncomb = minc_des.cols();

	double minAng_OTS_med, maxAng_OTS_mod;
	//Vector para identificr los actuadores a modificar
	Eigen::Vector2i i_qind = Eigen::Vector2i::Zero();

	//Referencia modificada
	Eigen::Vector4d q_ind_mod = q_ref+des_qind*vc_des.cast<double>();

	// Minimo angulo entre un par de ejes instantaneos de los OTS medidos
	minAng_OTS_med = angOTS_med.minCoeff();

	Eigen::VectorXd solAngOTS_mod;

	// Condicion para modificar la referencia
	if (minAng_OTS_med<lmin_Ang_OTS || abs(det_JDir_med) < lmin_JDir){
		// Identifico las patas que causan el minimo angulo de OTS
		for (int c_OTS=0; c_OTS<6; c_OTS++){
			if (angOTS_med(c_OTS) == minAng_OTS_med){
				i_qind(0) = mi_qind_des(c_OTS, 0);
				i_qind(1) = mi_qind_des(c_OTS, 1);
				break;
			}
		}

		// MODIFICACIONES POSIBLES SOBRE LA REFERENCIA
		// Calculo los angulos para todas las posibles modificaciones
		solAngOTS_mod = PosiblesAngOTS(q_ind_mod, X_cart, solOTS_med, des_qind, i_qind, minc_des, 
				ncomb, tol_FK, iter_FK, iter_OTS, tol_OTS, RParam, Mlim_q_ind, Vlim_angp);

		// REFERENCIA MODIFICADA QUE PRODUCIRA EL MAXIMO ANGULO OMEGA
		// Determino el maximo angulo OMEGA
		maxAng_OTS_mod = solAngOTS_mod.maxCoeff();
		// Cargo la mejor modificacion
		if (maxAng_OTS_mod > minAng_OTS_med){
			for (int i=0; i<ncomb; i++){
				if (solAngOTS_mod(i) == maxAng_OTS_mod){
					vc_des(i_qind(0)) += minc_des(0,i);
					vc_des(i_qind(1)) += minc_des(1,i);
					break;
				}
			}
			// Actualizo la referencia modificada
			q_ind_mod = q_ref + des_qind*vc_des.cast<double>();
		}
	}

	else{ // Condicion para retornar a la referencia
		// Minimo angulo entre un par de ejes instantaneos de los OTS de referencia
		double minAng_OTS_ref;
		int maxc_des, aux_maxc_des, ncomb_dec, aux_sumc_dec;
		Eigen::MatrixXi mdec_des;

		minAng_OTS_ref = angOTS_ref.minCoeff();
		if (minAng_OTS_ref >= lmin_Ang_OTS && abs(det_JDir_ref)>=lmin_JDir && (vc_des.cwiseAbs()).sum() >0){
			i_qind(0) = mi_qind_des(0,0);
			i_qind(1) = mi_qind_des(0,1);
			// Suma de incrementos de los actuadores 1 y 2
			maxc_des=abs(vc_des(mi_qind_des(0,0)))+abs(vc_des(mi_qind_des(0,1)));
			for (int c_OTS=1;c_OTS<6;c_OTS++){
				aux_maxc_des = abs(vc_des(mi_qind_des(c_OTS,0))) + abs(vc_des(mi_qind_des(c_OTS,1)));
				if (aux_maxc_des>maxc_des){
					// Actualizo la suma maxima de desplazamientos
					maxc_des = aux_maxc_des;
					// Actualizo el indice de las patas mas alejadas
					i_qind(0) = mi_qind_des(c_OTS,0); i_qind(1) = mi_qind_des(c_OTS,1);
				}
			}

			// Matriz con las posibles modificaciones para retorno a la referencia
			mdec_des = Eigen::MatrixXi::Zero(2,ncomb);
			// Numero de posibles modificaciones para retornar a la referencia
			ncomb_dec = 0;
			
			// Busco las combinaciones que ayudan a retornar a la referencia
			for (int i=0; i<ncomb; i++){
				aux_sumc_dec = abs(vc_des(i_qind(0))+minc_des(0,i))+abs(vc_des(i_qind(1))+minc_des(1,i));
				if (aux_sumc_dec<maxc_des){
					mdec_des(0,ncomb_dec) = minc_des(0,i); mdec_des(1,ncomb_dec) = minc_des(1,i);
					ncomb_dec++;
				}
			}
			
			// POSIBLES RETORNOS A LA REFERENCIA
			// Calculo los angulos para todas las posibles modificaciones
			solAngOTS_mod = PosiblesAngOTS(q_ind_mod, X_cart, solOTS_med, des_qind, i_qind, mdec_des, 
				ncomb_dec, tol_FK, iter_FK, iter_OTS, tol_OTS, RParam, Mlim_q_ind, Vlim_angp);
		
			// REFERENCIA MODIFICADA QUE PRODUCIRÁ EL MAXIMO ANGULO OMEGA
			// Determino el maximo angulo OMEGA
			maxAng_OTS_mod = solAngOTS_mod.maxCoeff();
			// Cargo la mejor modificacion
			for (int i=0;i<ncomb_dec;i++){
				if (solAngOTS_mod(i) == maxAng_OTS_mod){
					vc_des(i_qind(0)) += mdec_des(0,i);
					vc_des(i_qind(1)) += mdec_des(1,i);
					break;
				}
			}
			
			// Actualizo la referencia modificada
			q_ind_mod = q_ref + des_qind*vc_des.cast<double>();
		}
	}
	
	return q_ind_mod;
}