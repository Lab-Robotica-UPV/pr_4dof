#include "pr_biomech/streaming_gdlf.hpp"

void pr_biomech::StreamingGDLF::PelCalcs() {
	G_r_G.MPSIS = (G_r_G.RPSIS + G_r_G.LPSIS) / 2;
	G_r_G.MASIS = (G_r_G.RASIS + G_r_G.LASIS) / 2;
	Vector3d ACSpelvis = (G_r_G.RASIS + G_r_G.LASIS + G_r_G.RPSIS + G_r_G.LPSIS) / 4;
	Vector3d G_u_z_ACSpelvis = unit((G_r_G.RASIS - G_r_G.LASIS));
	Vector3d G_u_y_ACSpelvis = unit((G_r_G.MPSIS - G_r_G.MASIS).cross(G_r_G.RASIS - G_r_G.MASIS));
	Vector3d G_u_x_ACSpelvis = G_u_y_ACSpelvis.cross(G_u_z_ACSpelvis);
	R.G_R_ACSpelvis << G_u_x_ACSpelvis, G_u_y_ACSpelvis, G_u_z_ACSpelvis;
	HJC.col(GlobalCnt) = R.G_R_ACSpelvis * cal_data->ACSpelvis_r_PelAvg_ACSpelvis + ACSpelvis;
}