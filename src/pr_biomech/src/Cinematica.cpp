#include "pr_biomech/streaming_gdlf.hpp"


// Posicion y orientacion de los segmentos
void pr_biomech::StreamingGDLF::Seg_Kinematics() {
	//..............................................................................................
	// Cálculamos la posicion y orientacion de los segmentos 
	//..............................................................................................
	// Calculamos los puntos medios de los marcadores
	G_r_G.MidFE = (G_r_G.MFE + G_r_G.LFE) / 2; // Punto medio MFE y LFE
	G_r_G.MidM = (G_r_G.LM + G_r_G.MM) / 2; //Punto medio MFE y LFE
	G_r_G.MidDM = (G_r_G.MH1 + G_r_G.MH5) / 2; // Punto Medio D1M y DM5
	// Posicion y orientacion de la Pelvis
	pr_biomech::StreamingGDLF::PelCalcs();
	// Posicion y orientacion del femur
	G_r_G.Femur = (G_r_G.LFE + G_r_G.MFE) / 2;
	R.Femur = GetRotMat(G_r_G.LFE, G_r_G.MidFE, G_r_G.Pelvis, 2, 1, 3);
	// Tibia
	G_r_G.Tibia = (G_r_G.MM + G_r_G.LM) / 2;
	R.Tibia = GetRotMat(G_r_G.LM, G_r_G.MidM, G_r_G.FH, 2, 1, 3);
	// Pie
	G_r_G.Pie = (G_r_G.MH1 + G_r_G.MH5) / 2;
	R.Pie = GetRotMat(G_r_G.MH1, G_r_G.CAL, G_r_G.MidDM, 1, 2, 3);
	// Matriz de rotacion del femur segun Nidal
	R.G_R_LMO4 = GetRotMat(G_r_G.Pelvis, G_r_G.LFE, G_r_G.MFE, 2, 1, 3);

	// Matrices rotacion intermedias
	R.O0_O3 = R.Pelvis.transpose() * R.Femur * R_local.Femur_O3;
	R.Tibia_Pie = R.Tibia.transpose() * R.Pie;
	R.O5_O7 = R_local.Tibia_O5.transpose() * R.Tibia_Pie * R_local.Pie_O7;
	R.O3_O5 = (R.Femur * R_local.Femur_O3).transpose() * (R.Tibia * R_local.Tibia_O5);
    //std::cout << "R.Femur: " << R.Femur << std::endl;
    //std::cout << "R.Tibia: " << R.Tibia << std::endl;
    //std::cout << "R_local.Femur_O3: " << R_local.Femur_O3 << std::endl;
    //std::cout << "R_local.Pie_O7: " << R_local.Pie_O7 << std::endl;
    //std::cout << "R.O3_O5: " << R.O3_O5 << std::endl;

    //std::cout << "G_r_G.Femur " << G_r_G.Femur << std::endl;
    //std::cout << "G_r_G.Tibia " << G_r_G.Tibia << std::endl;
    //std::cout << "G_r_G.Pie " << G_r_G.Pie << std::endl;
    //std::cout << "G_r_G.Femur " << G_r_G.Femur << std::endl;
};

//Cinemática inversa
void pr_biomech::StreamingGDLF::Inv_Kinematics() {

	//Ponemos a 0 q.
	q.setZero(8);

    // ------------------------------------------------------------------------------------------------//
    // Calculo de la cinematica inversa
    // ------------------------------------------------------------------------------------------------//
    // Cadera
    theta3 = atan2(R.O0_O3(2, 2) * sin(alpha(3)) - R.O0_O3(2, 1) * cos(alpha(3)), R.O0_O3(2, 0));
    q_temp = theta3 - theta(3);
    if (q_temp >= PI) {
        theta3 = theta3 - PI;
    }
    else if (q_temp <= -PI) {
        theta3 = theta3 + PI;
    }
    q(3) = theta3 - theta(3);
    theta2 = atan2(R.O0_O3(2, 0) / cos(theta3), (R.O0_O3(2, 1) + R.O0_O3(2, 0) * tan(theta3) * cos(alpha(3))) / (-sin(alpha(3))));
    q(2) = theta2 - theta(2);
    //q1
    theta1 = atan2(R.O0_O3(0, 0) * sin(theta3) + R.O0_O3(1, 0) * cos(theta2) * cos(theta3), R.O0_O3(0, 0) * cos(theta2) * cos(theta3) - R.O0_O3(1, 0) * sin(theta3));
    q(1) = theta1 - theta(1);

    //Rodilla

    CalTheta2Theta3(); // Calculamos theta2 y theta3
    q(4) = -theta(4) - theta2 + Plane4Bar.ThetaL1X3;
    q(5) = -theta(5) - theta3 - Plane4Bar.ThetaL3X5 + theta2;

    PosKnee = theta3 - Plane4Bar.DELTA;

    //std::cout << "theta2: " << theta2 << std::endl;
    //std::cout << "theta3: " << theta3 << std::endl;
    //std::cout << "theta(4): " << theta(4) << std::endl;
    //std::cout << "theta(5): " << theta(5) << std::endl;
    //std::cout << "Plane4Bar.ThetaL3X5: " << Plane4Bar.ThetaL3X5 << std::endl;
    //std::cout << "Plane4Bar.ThetaL1X3: " << Plane4Bar.ThetaL1X3 << std::endl;

    if (q(5) > PI) {
        q(5) = q(5) - 2 * PI;
    }
    else if (q(5) < -PI) {
        q(5) = q(5) + 2 * PI;
    }

    //Tobillo
    theta7 = atan2(cos(alpha(7)) * R.O5_O7(2, 0), R.O5_O7(2, 1) - cos(alpha(6)) * sin(alpha(7)));
    if ((theta7 - theta(7)) >= PI / 2) {
        theta7 = theta7 - PI;
    }
    else if ((theta7 - theta(7)) <= -PI / 2) {
        theta7 = theta7 + PI;
    }
    q(7) = theta7 - theta(7);
    theta6 = atan2(cos(theta7) * R.O5_O7(1, 0) - cos(alpha(6)) * sin(theta7) * R.O5_O7(0, 0), cos(theta7) * R.O5_O7(0, 0) + cos(alpha(6)) * sin(theta7) * R.O5_O7(1, 0));
    q(6) = theta6 - theta(6);

    //std::cout << "q: " << q.transpose() << std::endl;
}

//Cinemática Directa
void pr_biomech::StreamingGDLF::Dir_Kinematics() {
    // Cálculo de DH
    Denavit(a, alpha, theta + q, d, dof, O0, i_1_r_Oi_1_Oi, i_r_Oi_Oj, i_1_R_i, i_R_j);
    // Calculo de las posicion es y orientaciones de los segmentos
    //Fémur
    R.Femur = R.Pelvis * i_R_j[0][3] * R_local.Femur_O3.transpose();
    G_r_G.Femur = R.Pelvis * i_r_Oi_Oj[0][5] + G_r_G.Pelvis - R.Femur * r_Local.Fem.O3;
    //Tibia
    R.Tibia = R.Pelvis * i_R_j[0][5] * R_local.Tibia_O5.transpose();
    G_r_G.Tibia = R.Pelvis * i_r_Oi_Oj[0][5] + G_r_G.Pelvis - R.Tibia * r_Local.Tib.O5;
    //Pie
    R.Pie = R.Pelvis * i_R_j[0][7] * R_local.Pie_O7.transpose();
    G_r_G.Pie = R.Pelvis * i_r_Oi_Oj[0][7] + G_r_G.Pelvis - R.Pie * r_Local.Pie.O7;
}