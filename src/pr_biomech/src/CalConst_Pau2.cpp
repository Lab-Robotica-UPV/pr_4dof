#include "pr_biomech/streaming_gdlf.hpp"

void pr_biomech::StreamingGDLF::CalConst_Pau2() {

	G_r_G.MHmid = (G_r_G.MH1 + G_r_G.MH5) / 2; //pie

	G_r_G.O4 = G_r_G.HJC;

	// Matrices de rotacion
	R.G_R_SCSpie = GetRotMat(G_r_G.MH5, G_r_G.CAL, G_r_G.MHmid, 1, -2, 3);

	// Femur
	Matrix3d Xi;
	Xi << G_r_G.HJC, G_r_G.MFE, G_r_G.LFE;
	// The center of the Femur is the mean of its anatomical markers, it has the most accurate velocity and acceleration values
	G_r_G.Fem_Cent = Xi.rowwise().mean();

	// Tibia
	Xi << G_r_G.FH, G_r_G.MM, G_r_G.LM;
	// The center of the Tibia is the mean of its anatomical markers, it has the most accurate velocity and acceleration values
	G_r_G.Tib_Cent = Xi.rowwise().mean();

	// Pie
	Xi << G_r_G.MH1, G_r_G.MH5, G_r_G.CAL;
	G_r_G.Pie_Cent = Xi.rowwise().mean();

	G_r_G.ACSpelvis = G_r_G.HJC;
	G_r_G.KJC = (G_r_G.MFE + G_r_G.LFE) / 2;
	G_r_G.AJC = (G_r_G.MM + G_r_G.LM) / 2;

	G_r_G.SCSpie = G_r_G.AJC; // pie
	G_r_G.Gpie = G_r_G.SCSpie + R.G_R_SCSpie * cal_data->SCSpie_r_SCSpie_Gpie;
}