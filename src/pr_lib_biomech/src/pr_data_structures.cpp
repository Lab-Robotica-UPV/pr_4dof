#include "pr_lib_biomech/pr_data_structures.hpp"
#include <iostream>

using namespace PRDataStructures;
using namespace std;

void G_r_G_struct::print_data() {

	cout << "--------G_r_G_struct (all transposed)---------" << endl << endl;
	cout << " MARCADORES INICIALES" << endl;
	cout << "LASIS  : " << LASIS.transpose() << endl;
	cout << "RASIS  : " << RASIS.transpose() << endl;
	cout << "LPSIS  : " << LPSIS.transpose() << endl;
	cout << "RPSIS  : " << RPSIS.transpose() << endl;
	cout << "LFE    : " << LFE.transpose() << endl;
	cout << "MFE    : " << MFE.transpose() << endl;
	cout << "LM     : " << LM.transpose() << endl;
	cout << "MM     : " << MM.transpose() << endl;
	cout << "FH     : " << FH.transpose() << endl;
	cout << "CAL    : " << CAL.transpose() << endl;
	cout << "MH1    : " << MH1.transpose() << endl;
	cout << "MH5    : " << MH5.transpose() << endl;
	cout << "Fext   : " << Fext.transpose() << endl;

	cout << " MARCADORES CALCULADOS" << endl;
	cout << "MidFE     : " << MidFE.transpose() << endl;
	cout << "MidM      : " << MidM.transpose() << endl;
	cout << "MidDM     : " << MidDM.transpose() << endl;
	cout << "MASIS     : " << MASIS.transpose() << endl;
	cout << "MPSIS     : " << MPSIS.transpose() << endl;
	cout << "Pelvis    : " << Pelvis.transpose() << endl;
	cout << "Femur     : " << Femur.transpose() << endl;
	cout << "Tibia     : " << Tibia.transpose() << endl;
	cout << "Pie       : " << Pie.transpose() << endl;

	cout << "--------G_r_G_struct (all transposed) end---------\n" << endl;

}

void R_struct::print_data() {

	cout << "--------R_struct ---------" << endl;
	cout << "Pelvis:\n" << Pelvis << endl;
	cout << "Femur:\n"  << Femur << endl;
	cout << "Tibia:\n"  << Tibia << endl;
	cout << "Pie:\n"    << Pie << endl;
	cout << "--------R_struct end---------\n" << endl;

}

void Data_struct::print_data() {

	cout << "--------Data_struct (first elems)---------" << endl;
	cout << "Time: " << Time(0) << endl;
	cout << "Date: " << Date << endl;
	cout << "G_Mext" << G_Fext << endl;
	cout << "G_Mext" << G_Mext << endl;
	cout << "MuscleForce: " << MuscleForce.col(0).transpose() << endl;
	cout << "F_Knee: " << F_Knee.col(0).transpose() << endl;
	cout << "M_Knee: " << M_Knee.col(0).transpose() << endl;
	cout << "q: " << q.col(0).transpose() << endl;
	cout << "PatForce: " << PatForce(0) << endl;
	cout << "TauMus: " << TauMus.col(0).transpose()  << endl;
	cout << "PosKnee: " << PosKnee(0) << endl;
	cout << "input.forces: " << input.forces.col(0).transpose() << endl;
	cout << "input.LASIS: " << input.LASIS.col(0).transpose() << endl;
	cout << "input.RASIS: " << input.RASIS.col(0).transpose() << endl;
	cout << "input.LPSIS: " << input.LPSIS.col(0).transpose() << endl;
	cout << "input.RPSIS: " << input.RPSIS.col(0).transpose() << endl;
	cout << "input.RLE: " << input.RLE.col(0).transpose() << endl;
	cout << "input.RME: " << input.RME.col(0).transpose() << endl;
	cout << "input.RHF: " << input.RHF.col(0).transpose() << endl;
	cout << "input.RLM: " << input.RLM.col(0).transpose() << endl;
	cout << "input.RMM: " << input.RMM.col(0).transpose() << endl;
	cout << "input.RCA: " << input.RCA.col(0).transpose() << endl;
	cout << "input.RFM: " << input.RFM.col(0).transpose() << endl;
	cout << "input.RVM: " << input.RVM.col(0).transpose() << endl;
	cout << "input.P_Movil_1: " << input.P_Movil_1.col(0).transpose() << endl;
	cout << "input.P_Movil_2: " << input.P_Movil_2.col(0).transpose() << endl;
	cout << "input.P_Movil_3: " << input.P_Movil_3.col(0).transpose() << endl;
	cout << "input.P_Fija_1: " << input.P_Fija_1.col(0).transpose() << endl;
	cout << "input.P_Fija_2: " << input.P_Fija_2.col(0).transpose() << endl;
	cout << "input.P_Fija_3: " << input.P_Fija_3.col(0).transpose() << endl;

}

void Piel_struct::print_data() {
	cout << "--------Piel (first column transposed)---------" << endl;
	cout << "LASIS: " << LASIS.col(0).transpose() << endl;
	cout << "RASIS: " << RASIS.col(0).transpose() << endl;
	cout << "LPSIS: " << LPSIS.col(0).transpose() << endl;
	cout << "RPSIS: " << RPSIS.col(0).transpose() << endl;
	cout << "LFE: " << LFE.col(0).transpose() << endl;
	cout << "MFE: " << MFE.col(0).transpose() << endl;
	cout << "FH: " << FH.col(0).transpose() << endl;
	cout << "LM: " << LM.col(0).transpose() << endl;
	cout << "MM: " << MM.col(0).transpose() << endl;
	cout << "CAL: " << CAL.col(0).transpose() << endl;
	cout << "MH1: " << MH1.col(0).transpose() << endl;
	cout << "MH5: " << MH5.col(0).transpose() << endl;
	cout << "Fext: " << Fext.col(0).transpose() << endl;
	cout << "--------Piel (first column transposed) end---------\n" << endl;
}

void forLigForceCal_struct::print_data() {
	cout << "--------forLigForceCal (transposed)---------" << endl;
	cout << "FlxActiveOrg: " << FlxActiveOrg << endl;
	cout << "FlxLegend: ";
	for (int i = 0; i < FlxLegend.size(); i++) {
		cout << FlxLegend[i] << " ";
	}
	cout << endl;
	cout << "FlxMusForceEstOpt: " << FlxMusForceEstOpt.transpose() << endl;
	cout << "FlxMaxMusForce: " << FlxMaxMusForce.transpose() << endl;
	cout << "ExtLegend: ";
	for (int i = 0; i < FlxLegend.size(); i++) {
		cout << ExtLegend[i] << " ";
	}
	cout << endl;
	cout << "ExtMusForceEstOpt: " << ExtMusForceEstOpt.transpose() << endl;
	cout << "ExtMaxMusForce: " << ExtMaxMusForce.transpose() << endl;
	cout << "PatLig: " << PatLig << endl;
	cout << "RecFem12_1: " << RecFem12_1 << endl;
	cout << "RecFem12_2: " << RecFem12_2 << endl;
	cout << "TenFacLat: " << TenFacLat << endl;
	cout << "--------forLigForceCal (transposed) end---------" << endl;
}

void ang_struct::print_data() {
	cout << "-------- Ang_struct ---------" << endl;
	cout << "ang.cad.FlxExt: " << cad.FlxExt << endl;
	cout << "ang.cad.AbdAduc: " << cad.AbdAduc << endl;
	cout << "ang.cad.IntExt: " << cad.IntExt << endl;
	cout << "ang.rod.FlxExt: " << rod.FlxExt << endl;
	cout << "ang.pie.FlxExt: " << pie.FlxExt << endl;
	cout << "-------- Ang_struct end---------" << endl;
}

void AUXILIAR_ForceSensorin_struct::print_data() {
	cout << "--------AUXILIAR ForceSensorin (first column transposed)---------" << endl;
	cout << "Force: " << Force.col(0).transpose() << endl;
	cout << "Torque: " << Torque.col(0).transpose() << endl;
	cout << "--------AUXILIAR ForceSensorin (first column transposed) end---------\n" << endl;
}