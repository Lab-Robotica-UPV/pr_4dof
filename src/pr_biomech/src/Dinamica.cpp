#include "pr_biomech/streaming_gdlf.hpp"

// Cálculo de las fuerzas generalizadas en las articulaciones (Tau)
void pr_biomech::StreamingGDLF::Inv_Dynamics() {
    // Calculamos el punto de giro
    L_O3_IC = Plane4Bar.L(1) / sin(theta4 - theta2) * sin(PI - theta4);
    G_r_G.IC = (R.Pelvis * i_r_Oi_Oj[0][4] + G_r_G.Pelvis) + R.Pelvis * i_R_j[0][4].col(0) * (L_O3_IC - Plane4Bar.L(2));
    //std::cout << "G_r_G.IC: " << G_r_G.IC << std::endl;
    
    // Cálculamos los ejes de giro y los puntos de las articulaciones
    Matrix <double, 3, 6> uJ; // Ejes de giro
    Matrix <double, 3, 6> rJ; // Punto de giro
    //std::cout << "rJ: " << std::endl;
    for (int i = 0; i < 6; i++)
    {
        if (i <= 3) {
            for (int j = 0; j < 6 + 1; j++)
            {
                uJ.col(i) = R.Pelvis * i_R_j[0][i].col(2);
                rJ.col(i) = R.Pelvis * i_r_Oi_Oj[0][i] + G_r_G.Pelvis;
            }
        }
        else {
            for (int j = 0; j < 6 + 1; j++)
            {
                uJ.col(i) = R.Pelvis * i_R_j[0][i + 1].col(2);
                rJ.col(i) = R.Pelvis * i_r_Oi_Oj[0][i + 1] + G_r_G.Pelvis;
            }
        }
        //std::cout << rJ.col(i).transpose() << std::endl;
    }
    rJ.col(3) = G_r_G.IC;
    //std::cout << G_r_G.IC << std::endl;


    // Calculamos el jacobiano de la fuerza externa
    Matrix <double, 6, 6> J_ext;

    //std::cout << "G_r_G.Fext: " << G_r_G.Fext << std::endl;
    for (int i = 0; i < 6; i++) {
      J_ext.col(i).topRows(3) = (uJ.col(i)).cross(G_r_G.Fext - rJ.col(i));
      J_ext.col(i).bottomRows(3) = uJ.col(i);
    }
    //std::cout << "J_ext: " << J_ext << std::endl;
    // Creamos el vector de fuerza 
    F_Ext.topRows(3) = G_Fext;
    F_Ext.bottomRows(3) = G_Mext;
    //std::cout << "F_Ext: " << F_Ext << std::endl;
    // Cálculamos la Tau
    Tau = J_ext.transpose() * F_Ext;



    
    //std::cout << "Tau Ext: " << Tau << std::endl;
};

// Cálculo de las fuerzas musculares
void pr_biomech::StreamingGDLF::MusForce() {
    // Contador de bucles
    int n = 0;
    // suma de los coeficientes
    double suma_coef = 0;
    // Coeficientes Lambda
    Eigen::Matrix <double, 6, 1> Lambda;
    // Fuerzas musculares
    Eigen::Matrix <double, 73, 1> TensMus;
    // Boleano tensiones negativas
    Eigen::Matrix <bool, 73, 1> n_TensNeg;
    // Boleano de tensiones negativas
    bool TensNeg = false;
    // Matriz de los coeficientes musculares
    //std::vector <std::vector <double>> A_origin; A_origin.resize(6);
    Eigen::Matrix <double, 6, 6> A_origin;
    Eigen::Matrix <double, 6, 6> A_neg;
    Eigen::Matrix <double, 6, 6> A;
    Eigen::Matrix <double, 6, 6> A_inv;

    //std::cout << A_neg << std::endl;

    A_origin << CoefMus1.transpose() * CoefMus1 / 2, CoefMus1.transpose()* CoefMus2 / 2, CoefMus1.transpose()* CoefMus3 / 2, CoefMus1.transpose()* CoefMus4 / 2, CoefMus1.transpose()* CoefMus5 / 2, CoefMus1.transpose()* CoefMus6 / 2,
                CoefMus2.transpose()* CoefMus1 / 2, CoefMus2.transpose()* CoefMus2 / 2, CoefMus2.transpose()* CoefMus3 / 2, CoefMus2.transpose()* CoefMus4 / 2, CoefMus2.transpose()* CoefMus5 / 2, CoefMus2.transpose()* CoefMus6 / 2,
                CoefMus3.transpose()* CoefMus1 / 2, CoefMus3.transpose()* CoefMus2 / 2, CoefMus3.transpose()* CoefMus3 / 2, CoefMus3.transpose()* CoefMus4 / 2, CoefMus3.transpose()* CoefMus5 / 2, CoefMus3.transpose()* CoefMus6 / 2,
                CoefMus4.transpose()* CoefMus1 / 2, CoefMus4.transpose()* CoefMus2 / 2, CoefMus4.transpose()* CoefMus3 / 2, CoefMus4.transpose()* CoefMus4 / 2, CoefMus4.transpose()* CoefMus5 / 2, CoefMus4.transpose()* CoefMus6 / 2,
                CoefMus5.transpose()* CoefMus1 / 2, CoefMus5.transpose()* CoefMus2 / 2, CoefMus5.transpose()* CoefMus3 / 2, CoefMus5.transpose()* CoefMus4 / 2, CoefMus5.transpose()* CoefMus5 / 2, CoefMus5.transpose()* CoefMus6 / 2,
                CoefMus6.transpose()* CoefMus1 / 2, CoefMus6.transpose()* CoefMus2 / 2, CoefMus6.transpose()* CoefMus3 / 2, CoefMus6.transpose()* CoefMus4 / 2, CoefMus6.transpose()* CoefMus5 / 2, CoefMus6.transpose()* CoefMus6 / 2;

    

    // Invertimos la matriz A
    //A_inv = A_origin.array().inverse();
    A_inv = A_origin.inverse();
   // Calculamos el valor de Lambda
    Lambda = A_inv * TauMus;
    //std::cout << Lambda.transpose() << std::endl;
    // Calculamos las tensiones musculares
    TensMus = (CoefMus1 * Lambda[0] + CoefMus2 * Lambda[1] + CoefMus3 * Lambda[2] + CoefMus4 * Lambda[3] + CoefMus5 * Lambda[4] + CoefMus6 * Lambda[5])/2;
    // iniciamos la variable anyNegative y comprovamos si existe algun valor negativo
    bool anyNegative = (TensMus.array() < 0).any();

    // Si hay valores negativos buscamos sus posiciones
    if (anyNegative) {
        TensNeg = true;
        for (int i = 0; i < 73; i++) {
            if (TensMus[i] < 0) {
                n_TensNeg[i] = true;
                TensMus[i] = 0;
            }
            else {
                n_TensNeg[i] = false;
            };
        };
    };

    //// Buscamos las tensiones negativas
    //for (int i = 0; i < 73; i++) {
    //    if (!TensNeg && TensMus[i] < 0) {
    //        n_TensNeg[i] = true;
    //        TensMus[i] = 0;
    //        TensNeg = true;
    //    }
    //    else if (TensMus[i] < 0) {
    //        n_TensNeg[i] = true;
    //        TensMus[i] = 0;
    //    }
    //    else {
    //        n_TensNeg[i] = false;
    //    };
    //};

    //std::cout << "TensMus: " << TensMus << std::endl;



    //std::cout << A << std::endl;

    //std::cout << "n_TensNeg: "  << n_TensNeg.transpose() << std::endl;

    // Ponemos a cero la matriz a restar
    A_neg.fill(0);
    // Igualamos la matriz A a la original
    A = A_origin;

    // Bucle de cálculo de las tensiones musculares
    while (TensNeg) {
        // Desactivamos la variable del While
        TensNeg = false;
        // Aumentamos el contador 
        n++;
        // Calculamos la matriz A sin los coeficientes negativos
        for (int i = 0; i < 73; i++) {
            if (n_TensNeg[i]) {
                // Matriz a restar
                A_neg << CoefMus1[i] * CoefMus1[i] / 2, CoefMus1[i] * CoefMus2[i] / 2, CoefMus1[i] * CoefMus3[i] / 2, CoefMus1[i] * CoefMus4[i] / 2, CoefMus1[i] * CoefMus5[i] / 2, CoefMus1[i] * CoefMus6[i] / 2,
                    CoefMus2[i] * CoefMus1[i] / 2, CoefMus2[i] * CoefMus2[i] / 2, CoefMus2[i] * CoefMus3[i] / 2, CoefMus2[i] * CoefMus4[i] / 2, CoefMus2[i] * CoefMus5[i] / 2, CoefMus2[i] * CoefMus6[i] / 2,
                    CoefMus3[i] * CoefMus1[i] / 2, CoefMus3[i] * CoefMus2[i] / 2, CoefMus3[i] * CoefMus3[i] / 2, CoefMus3[i] * CoefMus4[i] / 2, CoefMus3[i] * CoefMus5[i] / 2, CoefMus3[i] * CoefMus6[i] / 2,
                    CoefMus4[i] * CoefMus1[i] / 2, CoefMus4[i] * CoefMus2[i] / 2, CoefMus4[i] * CoefMus3[i] / 2, CoefMus4[i] * CoefMus4[i] / 2, CoefMus4[i] * CoefMus5[i] / 2, CoefMus4[i] * CoefMus6[i] / 2,
                    CoefMus5[i] * CoefMus1[i] / 2, CoefMus5[i] * CoefMus2[i] / 2, CoefMus5[i] * CoefMus3[i] / 2, CoefMus5[i] * CoefMus4[i] / 2, CoefMus5[i] * CoefMus5[i] / 2, CoefMus5[i] * CoefMus6[i] / 2,
                    CoefMus6[i] * CoefMus1[i] / 2, CoefMus6[i] * CoefMus2[i] / 2, CoefMus6[i] * CoefMus3[i] / 2, CoefMus6[i] * CoefMus4[i] / 2, CoefMus6[i] * CoefMus5[i] / 2, CoefMus6[i] * CoefMus6[i] / 2;
                // Matriz A 
                A = A - A_neg;
            };// Fin del If
        }; // Fin del for del cálculo de la matriz A

        // Invertimos la matriz A
        //A = A.array().inverse();
        A_inv = A.inverse();
        // Calculamos el valor de Lambda
        Lambda = A_inv * TauMus;

        // Calculamos las tensiones músculares de los músculos que anteriormente eran positivos
        for (int i = 0; i < 73; i++) {
            if (!n_TensNeg[i]) {
                TensMus[i] = (CoefMus1[i] * Lambda[0] + CoefMus2[i] * Lambda[1] + CoefMus3[i] * Lambda[2] + CoefMus4[i] * Lambda[3] + CoefMus5[i] * Lambda[4] + CoefMus6[i] * Lambda[5]) / 2;
            };
        };
        //Comprovamos si hay algun valor negativo
        anyNegative = (TensMus.array() < 0).any();
        // En caso afirmativo, desactivamos el nuevo musculo 
        if (anyNegative) {
            // Si uno de las tensiones anteriormente positivas es negativa, continuamos en el bucle while
            TensNeg = true;
            // Calculamos las fuerzas musculares para todos los músculos
            for (int i = 0; i < 73; i++) {
                TensMus[i] = (CoefMus1[i] * Lambda[0] + CoefMus2[i] * Lambda[1] + CoefMus3[i] * Lambda[2] + CoefMus4[i] * Lambda[3] + CoefMus5[i] * Lambda[4] + CoefMus6[i] * Lambda[5]) / 2;
                // Si el valor es negativo, igualamos a cero y desactivamos el músculo
                if (TensMus[i] < 0) {
                    TensMus[i] = 0;
                    n_TensNeg[i] = true;
                }
                else {
                    n_TensNeg[i] = false;
                };
            };

            for (int i = 0; i < 73; i++) {
                if (!n_TensNeg[i] && TensMus[i] < 0) {
                    TensMus[i] = 0;
                    n_TensNeg[i] = true;
                };
            };
        };
        if (TensNeg) {
            // Igualamos la matriz A a la original
            A = A_origin;
        };
        
    }; // Fin del While

    //std::cout << "A final: " << A << std::endl;
    //std::cout << "TensMus: " << TensMus << std::endl;
    //std::cout << " n " << n << std::endl;

    // Calculamos las fuerzas musculares
    for (int i = 0; i < 73; i++) {
        MuscleForce[i] = gdlf_data->AreaEfect[i] * TensMus[i];
    };
    //std::cout << "TensMus: " << TensMus.transpose() << std::endl;
    //std::cout << "MuscleForce: " << MuscleForce.transpose() << std::endl;

    // Calculamos la fuerza en el ligamento patelar
    PatForce = PatCoef[0] * MuscleForce[5] + PatCoef[1] * MuscleForce[6] + PatCoef[2] * MuscleForce[11] + PatCoef[3] * MuscleForce[12] + PatCoef[4] * MuscleForce[13] + PatCoef[5] * MuscleForce[14] + PatCoef[6] * MuscleForce[15] + PatCoef[7] * MuscleForce[16] + PatCoef[8] * MuscleForce[17];
    //PatForce = gdlf_data->AllPatCoef[1] * MuscleForce[6] + gdlf_data->AllPatCoef[2] * MuscleForce[7] + gdlf_data->AllPatCoef[3] * MuscleForce[12] + gdlf_data->AllPatCoef[4] * MuscleForce[13] + gdlf_data->AllPatCoef[5] * MuscleForce[14] + gdlf_data->AllPatCoef[6] * MuscleForce[15] + gdlf_data->AllPatCoef[7] * MuscleForce[16] + gdlf_data->AllPatCoef[8] * MuscleForce[17] + gdlf_data->AllPatCoef[9] * MuscleForce[18];
    //std::cout << PatForce << std::endl;
     

};

// Cálculo de las fuerzas rodilla
void pr_biomech::StreamingGDLF::CalcKneeForces() {
    // Posición  y orientación de la rodilla
    Eigen::Vector3d r_Knee = G_r_G.Femur;
    Eigen::Matrix3d R_Knee = R.Femur;
    // Ponemos a zero las variables de salida
    F_Knee.setZero();
    M_Knee.setZero();
    // Separamos la matriz en fuerzas y momentos
    KneeCoef_Force = KneeCoef.topRows(3);
    KneeCoef_Torque = KneeCoef.bottomRows(3);
    //std::cout << "KneeCoef_Force : \n" << KneeCoef_Force << std::endl;
    //std::cout << "KneeCoef_Torque : \n" << KneeCoef_Torque << std::endl;

    // Ploteamos la información
    //std::cout << "KneeCoef_Force: \n" << KneeCoef_Force.col(0) << std::endl;
    // Sumamos las fuerzas 
    int n = 0;
    for (int i=0; i<MusPos.size(); i++) {
        //std::cout << KneeCoef_Force(n) << std::endl;
        F_Knee = F_Knee + KneeCoef_Force.col(n) * MuscleForce[MusPos[i]];
        M_Knee = M_Knee + KneeCoef_Torque.col(n) * MuscleForce[MusPos[i]];
        n++;
        //std::cout << "M_Knee: " << n << " " << M_Knee << std::endl;
    };
    F_Knee = F_Knee + KneeCoef_Force.col(10) * PatForce;
    M_Knee = M_Knee + KneeCoef_Torque.col(10) * PatForce;
    //std::cout << "M_Knee: " << M_Knee << std::endl;
    //std::cout << "F_Knee musculos: " << F_Knee << std::endl;

    // Introducimos las fuerzas gravitacionales
    F_Knee = F_Knee + R.Femur.transpose() * gdlf_data->SegMass[5] * gdlf_data->G_g; //Tibia
    F_Knee = F_Knee + R.Femur.transpose() * gdlf_data->SegMass[7] * gdlf_data->G_g; //Pie 
    F_Knee = F_Knee + R.Femur.transpose() * G_Fext;
    //std::cout << "F_Knee F ext: " << F_Knee << std::endl;
    // Introducimos los momentos en la rodilla creados por la tibia
    rF = G_r_G.Tibia + R.Tibia * gdlf_data->r_Local.Tib.CoM;
    F = gdlf_data->SegMass[5] * gdlf_data->G_g;
    M_Knee = M_Knee + R.Femur.transpose() * (rF - G_r_G.Femur).cross(F);
    //std::cout << "M_Knee Tibia " << M_Knee << std::endl;
    // Introducimos los momentos en la rodilla creados por el Pie
    rF = G_r_G.Pie + R.Pie* gdlf_data->r_Local.Pie.CoM;
    F = gdlf_data->SegMass[7] * gdlf_data->G_g;
    M_Knee = M_Knee + R.Femur.transpose() * (rF - G_r_G.Femur).cross(F);
    //std::cout << "M_Knee Pie " << M_Knee << std::endl;
    // Introducimos el momento creado por la fuerza externa
    M_Knee = M_Knee + R.Femur.transpose() * G_Mext + R.Femur.transpose() * (G_r_G.Fext - G_r_G.Femur).cross(G_Fext);// +R.Femur.transpose() * ((G_r_G.Fext - G_r_G.Femur).cross(AUXILIAR_ForceSensorin.Force));
    //std::cout << "M_Knee F ext: " << M_Knee << std::endl;
    //std::cout << "Fuerza: " << AUXILIAR_ForceSensorin.Force.col(GlobalCnt) << std::endl;
    //std::cout << "G_g_G: " << G_r_G.Fext - G_r_G.Femur << std::endl;
    //(AUXILIAR_ForceSensorin.Force).cross(G_r_G.Fext - G_r_G.Femur);
    //std::cout << "M_Knee Fext " << M_Knee << std::endl;
    //AUXILIAR_ForceSensorin.Force.cross(G_r_G.Fext - G_r_G.Femur);
    //(G_r_G.Fext - G_r_G.Femur).cross(AUXILIAR_ForceSensorin.Force.col(GlobalCnt));
};