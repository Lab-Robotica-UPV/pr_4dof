#include "pr_biomech/streaming_gdlf.hpp"
#include <algorithm>

void pr_biomech::StreamingGDLF::FOpt() {

    Matrix3d LMO4_R_u, u_R_G, G_R_u, u_R_O4, O4_R_u, G_R_O4, u_R_O5, O5_R_u, O5_R_G, G_R_O5;
    Vector3d u_r_O5est_A, ro1o2, G_r_O4_O5, u_r_O4_O5, temp;
    Eigen::Matrix<double, 3, Dynamic> i_1_r_Oi_1_Oi;
    std::vector<Eigen::Matrix3d> i_1_R_i;
    std::vector<std::vector<Eigen::Matrix3d>> i_R_j;
    std::vector<std::vector<Eigen::Vector3d>>i_r_Oi_Oj;
    Matrix<double, 6, 1> d;
    Matrix<double, 5, 1> a, alpha;
    double beta, a4, d5, L_O5_IC;
    VectorXd CoefMus, Area_Mus;
    int pos;
    Matrix<double, 3, 5> dd_r_SCS_Tib_ddqi, J_G6;
    Matrix<double, 5, 1> Tau_G, Tau_G_pie;
    Vector3d G_r_AJC_AJC2;
    Matrix<double, 3, 5> J_Gpie, J_Fext;
    double Tau_Grav, Tau_Obj;
    Vector3d u_J_Knee;
    double Obj_local;

    int dof = 5;
    LMO4_R_u = R.LMO4_R_u;
    u_r_O5est_A = A;

    G_r_G.LMO4 = G_r_G.LFE;
	G_r_G.O1 = G_r_G.O4;
	ro1o2 = G_r_G.O1;	

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

    d.fill(0);

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

    q(4, 0) = theta2 + theta0 - beta - 2 * PI;
	q(5, 0) = theta3 - theta2;

	q(1, 0) = atan2(G_R_O4(1, 2), G_R_O4(0, 2));
	q(2, 0) = atan2(sqrt(pow(G_R_O4(0, 2), 2) + pow(G_R_O4(1, 2), 2)), G_R_O4(2, 2));
	//q(4, 1) = atan2(sin(q(3)) * R.G_R_O4(3, 2), -sin(q(3)) * R.G_R_O4(3, 1));
	q(3, 0) = atan2(sin(q(2, 0)) * G_R_O4(2, 1), -sin(q(2, 0)) * G_R_O4(2, 0));
	//std::cout << q << std::endl;
	Denavit(a, alpha, q, d, dof, ro1o2, i_1_r_Oi_1_Oi, i_r_Oi_Oj, i_1_R_i, i_R_j);
    // std::cout << "q: " << q.transpose() << std::endl;

    if (musculo_Obj != ""){
        
        // Q5 intermedia
        q5ind = FindLoc(q(4), gdlf_data->q5);

        if (std::find(Flx_name.begin(), Flx_name.end(), musculo_Obj) != Flx_name.end()){
            Area_Mus.resize(8);
            Area_Mus << 0.00118000000000000,0.00240000000000000,0.00438000000000000,0.00272000000000000,0.00147000000000000,0.00171000000000000,0.000590000000000000,0.000490000000000000;
            CoefMus = InterCoef(q(4),gdlf_data->q5.segment(q5ind(0), 2),gdlf_data->AllFlxCoefMat.middleCols(q5ind(0), 2));
            std::vector<std::string>::iterator it = std::find(Flx_name.begin(), Flx_name.end(), musculo_Obj);
            pos = std::distance(Flx_name.begin(), it);
            // Chequeamos que los coeficientes tengan valor negativo
            CoefMus = CoefMus.cwiseMax(0);
        }
        else if (std::find(Ext_name.begin(), Ext_name.end(), musculo_Obj) != Ext_name.end()){
            Area_Mus.resize(10);
            Area_Mus << 0.000880000000000000,0.00190500000000000,0.000980000000000000,0.00232000000000000,0.00269000000000000,0.00590000000000000,0.00190500000000000,0.00107000000000000,0.00144500000000000,0.00144500000000000;
            CoefMus = InterCoef(q(4),gdlf_data->q5.segment(q5ind(0), 2),gdlf_data->AllExtCoefMat2.middleCols(q5ind(0), 2));
            std::vector<std::string>::iterator it = std::find(Ext_name.begin(), Ext_name.end(), musculo_Obj);
            pos = std::distance(Ext_name.begin(), it);
            // Chequeamos que los coeficientes tengan valor negativo
            CoefMus = CoefMus.cwiseMin(0);
        }
        else {
            std::cout << "El nombre del musculo no es correcto.";
            Fext_Opt = Vector3d::Zero();
            return;
        }
        // Comprobamos que el coeficiente no sea nulo. Si es igual a 0, Obj = 0 y avisamos
        if (CoefMus(pos) != 0) {
            // Calculamos el momento necesario para dicha fuerza muscular
            VectorXd aux = (CoefMus.cwiseProduct(Area_Mus)).array().square();
            Obj_local = -Obj*aux.sum()/(CoefMus(pos)*pow(Area_Mus(pos),2));
        }
        else {
            std::cout << "El coeficiente del musculo elegido es 0.";
            Fext_Opt = Vector3d::Zero();
            return;
        }
    }
    else {
        Obj_local = -Obj;
    }

    // Calculamos el taud e la gravedad
    // Calculo del jacobiano de la tibia
    R.G_R_SCS_Tib = i_R_j[0][5] * cal_data->O6_R_SCS_Tib;
    G_r_G.SCS_Tib = i_r_Oi_Oj[0][5] + (i_R_j[0][5] * cal_data->O6_r_O6.SCS_Tib);

    L_O5_IC = L(1) / sin((theta4 - theta2)) * sin(PI - theta4);
	G_r_G.IC = i_r_Oi_Oj[0][4] + i_R_j[0][4].col(0) * L_O5_IC;

    for (int i = 1; i < 4; i++)
	{
		dd_r_SCS_Tib_ddqi.col(i) = i_R_j[0][i].col(2).cross((G_r_G.SCS_Tib - i_r_Oi_Oj[0][i]));
	}
	dd_r_SCS_Tib_ddqi.col(4) = i_R_j[0][4].col(2).cross((G_r_G.SCS_Tib - G_r_G.IC));

    J_G6 = jacobian(R, dd_r_SCS_Tib_ddqi, cal_data->SCS_Tib_rg, i_R_j);
    Tau_G = J_G6.transpose() * (cal_data->G_g * cal_data->m(5));

    // Jacobiano del pie y la fuerza externa
    G_r_G.LM2 = i_r_Oi_Oj[0][5] + (i_R_j[0][5] * cal_data->O6_r_O6.LM2);
	G_r_G.MM2 = i_r_Oi_Oj[0][5] + (i_R_j[0][5] * cal_data->O6_r_O6.MM2);
	G_r_G.AJC2 = (G_r_G.LM2 + G_r_G.MM2) / 2;
	G_r_AJC_AJC2 = G_r_G.AJC2 - G_r_G.AJC;
    G_r_G.Fext_r = G_r_G.Fext + G_r_AJC_AJC2;
	G_r_G.Gpie_r = G_r_G.Gpie + G_r_AJC_AJC2;

    for (int i = 1; i < 4; i++) // Calcular jacobiano de las fuerzas inerciales en el pie
	{
		J_Gpie.col(i) = i_R_j[0][i].col(2).cross((G_r_G.Gpie_r - i_r_Oi_Oj[0][i]));
		J_Fext.col(i) = i_R_j[0][i].col(2).cross((G_r_G.Fext_r - i_r_Oi_Oj[0][i]));
	}
	J_Gpie.col(4) = i_R_j[0][5].col(2).cross((G_r_G.Gpie_r - G_r_G.IC));
	J_Fext.col(4) = i_R_j[0][5].col(2).cross((G_r_G.Fext_r - G_r_G.IC));
    // std::cout << "J_Fext: t" << std::endl;
    // std::cout << J_Fext << std::endl;

    Tau_G_pie = J_Gpie.transpose() * cal_data->G_g * cal_data->m_pie;

    // Tau gravedad
    Tau_Grav = Tau_G_pie(4) + Tau_G(4);

    // Tau objetivo
    Tau_Obj = Obj_local - Tau_Grav;

    // Calculo de la fuerza externa minima
    u_J_Knee = unit(J_Fext.col(4));

    Fext_Opt = Tau_Obj*u_J_Knee/(J_Fext.col(4).transpose()*u_J_Knee);

    // std::cout << "u_J_Knee: " << u_J_Knee.transpose() << std::endl;
    // std::cout << "G_r_G.Fext_r" << G_r_G.Fext_r.transpose() << std::endl;
    // std::cout << "Fext_Opt: " << Fext_Opt.transpose() << std::endl;
    // std::cout << "G_r_G.Gpie_r" << G_r_G.Gpie_r.transpose() << std::endl;
    // G_r_G.print_data();
    // std::cout << "a: " << a.transpose() << std::endl;
    // std::cout << "alpha: " << alpha.transpose() << std::endl;
    // std::cout << "q: " << q.transpose() << std::endl;
    // std::cout << "d: " << d.transpose() << std::endl;
    // std::cout << "ro1o2: " << ro1o2.transpose() << std::endl;
    // std::cout << "i_R_j" << i_R_j[0][5].col(2).transpose() << std::endl;
    // std::cout << "G_r_G.Fext_r - G_r_G.IC" << (G_r_G.Fext_r - G_r_G.IC).transpose() << std::endl;
    // std::cout << "temp: " << temp.transpose() << std::endl;
    // std::cout << "O4_R_u: " << std::endl;
    // std::cout << O4_R_u << std::endl;
    // std::cout << "theta0: " << theta0 << std::endl;
    // std::cout << "theta2: " << theta2 << std::endl;
}