#include "pr_biomech/streaming_gdlf.hpp"

void pr_biomech::StreamingGDLF::CalTheta2Theta3() {

	struct Prj_struct {
		Vector3d LMO4_r_KJC_LM, LMO4_r_KJC_FH;
		Vector3d u_r_KJC_LM, u_r_KJC_FH;
		Vector2d u_r_LM_FH;
	};
	Prj_struct Prj;

	Vector3d LMO4_r_G_KJC = R.G_R_LMO4.transpose() * G_r_G.KJC;

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