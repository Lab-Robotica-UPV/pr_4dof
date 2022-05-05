#include "pr_lib_biomech/pr_data_structures.hpp"
#include <iostream>

using namespace PRDataStructures;
using namespace std;

void G_r_G_struct::print_data() {

	cout << "--------G_r_G_struct (all transposed)---------" << endl;
	cout << "ACSpelvis: " << ACSpelvis.transpose() << endl;
	cout << "AJC: " << AJC.transpose() << endl;
	cout << "AJC2: " << AJC2.transpose() << endl;
	cout << "CAL: " << CAL.transpose() << endl;
	cout << "CPL: " << CPL.transpose() << endl;
	cout << "CPM: " << CPM.transpose() << endl;
	cout << "Cal2: " << Cal2.transpose() << endl;
	cout << "Ext.PatLig.Pat: " << Ext.PatLig.Pat.transpose() << endl;
	cout << "FH: " << FH.transpose() << endl;
	cout << "FH2: " << FH2.transpose() << endl;
	cout << "Fem_Cent: " << Fem_Cent.transpose() << endl;
	cout << "Fext: " << Fext.transpose() << endl;
	cout << "Fext_r: " << Fext_r.transpose() << endl;
	cout << "Fext_u: " << Fext_u.transpose() << endl;
	cout << "FibColLigTib: " << FibColLigTib.transpose() << endl;
	cout << "FibColLigTib_un: " << FibColLigTib_un.transpose() << endl;
	cout << "Gpie: " << Gpie.transpose() << endl;
	cout << "Gpie_r: " << Gpie_r.transpose() << endl;
	cout << "HJC: " << HJC.transpose() << endl;
	cout << "IC: " << IC.transpose() << endl;
	cout << "KJC: " << KJC.transpose() << endl;
	cout << "LASIS: " << LASIS.transpose() << endl;
	cout << "LFE: " << LFE.transpose() << endl;
	cout << "LM: " << LM.transpose() << endl;
	cout << "LM2: " << LM2.transpose() << endl;
	cout << "LMO4: " << LMO4.transpose() << endl;
	cout << "LPSIS: " << LPSIS.transpose() << endl;
	cout << "MASIS: " << MASIS.transpose() << endl;
	cout << "MFE: " << MFE.transpose() << endl;
	cout << "MH1: " << MH1.transpose() << endl;
	cout << "MH1_2: " << MH1_2.transpose() << endl;
	cout << "MH5: " << MH5.transpose() << endl;
	cout << "MH5_2: " << MH5_2.transpose() << endl;
	cout << "MHmid: " << MHmid.transpose() << endl;
	cout << "MM: " << MM.transpose() << endl;
	cout << "MM2: " << MM2.transpose() << endl;
	cout << "MPSIS: " << MPSIS.transpose() << endl;
	cout << "O1: " << O1.transpose() << endl;
	cout << "O4: " << O4.transpose() << endl;
	cout << "O5: " << O5.transpose() << endl;
	cout << "P1: " << P1.transpose() << endl;
	cout << "P2: " << P2.transpose() << endl;
	cout << "P3: " << P3.transpose() << endl;
	cout << "P4: " << P4.transpose() << endl;
	cout << "Pie_Cent: " << Pie_Cent.transpose() << endl;
	cout << "RASIS: " << RASIS.transpose() << endl;
	cout << "RPSIS: " << RPSIS.transpose() << endl;
	cout << "SCS_Fem: " << SCS_Fem.transpose() << endl;
	cout << "SCS_Tib: " << SCS_Tib.transpose() << endl;
	cout << "SCSpie: " << SCSpie.transpose() << endl;
	cout << "TibColLigTib: " << TibColLigTib.transpose() << endl;
	cout << "TibColLigTib_un: " << TibColLigTib_un.transpose() << endl;
	cout << "Tib_Cent: " << Tib_Cent.transpose() << endl;
	cout << "ViaPtsViaContours.CylCent: " << ViaPtsViaContours.CylCent.transpose() << endl;
	cout << "ViaPtsViaContours.CylDir: " << ViaPtsViaContours.CylDir.transpose() << endl;
	cout << "g4: " << g4.transpose() << endl;
	cout << "g6: " << g6.transpose() << endl;
	cout << "--------G_r_G_struct (all transposed) end---------\n" << endl;

}

void R_struct::print_data() {

	cout << "--------R_struct ---------" << endl;
	cout << "u_R_LMO4:\n" << u_R_LMO4 << endl;
	cout << "LMO4_R_u:\n" << LMO4_R_u << endl;
	cout << "G_R_ACSpelvis:\n" << G_R_ACSpelvis << endl;
	cout << "G_R_SCSpie:\n" << G_R_SCSpie << endl;
	cout << "G_R_LMO4:\n" << G_R_LMO4 << endl;
	cout << "LMO4_R_G:\n" << LMO4_R_G << endl;
	cout << "G_R_SCS_Tib:\n" << G_R_SCS_Tib << endl;
	cout << "--------R_struct end---------\n" << endl;

}

void Data_struct::print_data() {

	cout << "--------Data_struct (first elems)---------" << endl;
	cout << "ang.cad.FlxExt: " << ang.cad.FlxExt(0) << endl;
	cout << "ang.cad.AbdAduc: " << ang.cad.AbdAduc(0) << endl;
	cout << "ang.cad.IntExt: " << ang.cad.IntExt(0) << endl;
	cout << "ang.rod.FlxExt: " << ang.rod.FlxExt(0) << endl;
	cout << "ang.pie.FlxExt: " << ang.pie.FlxExt(0) << endl;
	cout << "Time: " << Time(0) << endl;
	cout << "Date: " << Date << endl;
	cout << "GenForceHipFlxExt: " << GenForceHipFlxExt(0) << endl;
	cout << "GenForceKneeFlxExtGrav: " << GenForceKneeFlxExtGrav(0) << endl;
	cout << "GenForceKneeFlxExtFext: " << GenForceKneeFlxExtFext(0) << endl;
	cout << "GenForceKneeFlxExtMext: " << GenForceKneeFlxExtMext(0) << endl;
	cout << "GenForceKneeFlxExt: " << GenForceKneeFlxExt(0) << endl;
	cout << "q: " << q.col(0).transpose() << endl;
	cout << "BicFemCB: " << BicFemCB(0) << endl;
	cout << "GastLat: " << GastLat(0) << endl;
	cout << "GastMed: " << GastMed(0) << endl;
	cout << "BicFemCL: " << BicFemCL(0) << endl;
	cout << "SemTend: " << SemTend(0) << endl;
	cout << "SemMem: " << SemMem(0) << endl;
	cout << "Sat: " << Sat(0) << endl;
	cout << "Gra: " << Gra(0) << endl;
	cout << "VasInt123: " << VasInt123(0) << endl;
	cout << "VasMedInf12: " << VasMedInf12(0) << endl;
	cout << "VasMedMed12: " << VasMedMed12(0) << endl;
	cout << "VasMedSup34: " << VasMedSup34(0) << endl;
	cout << "VasLatSup12: " << VasLatSup12(0) << endl;
	cout << "VasInt456: " << VasInt456(0) << endl;
	cout << "VasLatInf4: " << VasLatInf4(0) << endl;
	cout << "RecFem12_1: " << RecFem12_1(0) << endl;
	cout << "RecFem12_2: " << RecFem12_2(0) << endl;
	cout << "TenFacLat: " << TenFacLat(0) << endl;
	cout << "FlxCoefMatRed: " << FlxCoefMatRed.col(0).transpose() << endl;
	cout << "ExtCoefMatRed: " << ExtCoefMatRed.col(0).transpose() << endl;
	cout << "F_OnTibUc: " << F_OnTibUc(0) << endl;
	cout << "F_OnTibUt: " << F_OnTibUt(0) << endl;
	cout << "F_ACL: " << F_ACL(0) << endl;
	cout << "F_PCL: " << F_PCL(0) << endl;
	cout << "Frame: " << Frame(0) << endl;
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
	cout << "--------Data_struct (first elems) end ---------\n" << endl;

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