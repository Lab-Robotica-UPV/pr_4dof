#include "pr_lib_biomech/pr_json_gdlf.hpp"

using namespace PRJsonData;

//void PRJsonGdlf::Lig_struct::print_data(){
//    std::cout << "--------Lig struct---------" << std::endl;
//    std::cout << "LigPat: : " << LigPat << std::endl;
//    std::cout << "RadioPat: : " << RadioPat << std::endl;
//    std::cout << "--------Lig struct end ----\n" << std::endl;
//}

void PRJsonGdlf::Plane4Bar_struct::print_data() {
    std::cout << "--------Plane4Bar struct---------" << std::endl;
    std::cout << "LMO4_u (transposed): " << LMO4_u.transpose() << std::endl;
    std::cout << "L (transpose): " << L.transpose() << std::endl;
    std::cout << "DELTA: " << DELTA << std::endl;
    std::cout << "u_R_LMO4:\n" << u_R_LMO4 << std::endl;
    std::cout << "ThetaL1X3: " << ThetaL1X3 << std::endl;
    std::cout << "ThetaL3X5: " << ThetaL3X5 << std::endl;
    std::cout << "--------Plane4Bar struct end ----\n" << std::endl;
}

void PRJsonGdlf::r_Local_struct::print_data() {
    std::cout << "--------r_Local struct---------" << std::endl;
    std::cout << "-----Femur----- " << std::endl;

    std::cout << "CoM" << Fem.CoM << std::endl;
    std::cout << "LFE" << Fem.LFE << std::endl;
    std::cout << "MFE" << Fem.MFE << std::endl;
    std::cout << "O3" << Fem.O3 << std::endl;

    std::cout << "-----Tibia----- " << std::endl;
    std::cout << "CoM" << Tib.CoM << std::endl;
    std::cout << "LM" << Tib.LM << std::endl;
    std::cout << "MM" << Tib.MM << std::endl;
    std::cout << "FH" << Tib.FH << std::endl;
    std::cout << "O5" << Tib.O5 << std::endl;

    std::cout << "-----Pie----- " << std::endl;
    std::cout << "CoM" << Pie.CoM << std::endl;
    std::cout << "MH1" << Pie.MH1 << std::endl;
    std::cout << "MH5" << Pie.MH5 << std::endl;
    std::cout << "CAL" << Pie.CAL << std::endl;
    std::cout << "O7" << Pie.O7 << std::endl;

    std::cout << "-----Pelvis----- " << std::endl;
    std::cout << "LASIS" << Pel.LASIS << std::endl;
    std::cout << "RASIS" << Pel.RASIS << std::endl;
    std::cout << "LPSIS" << Pel.LPSIS << std::endl;
    std::cout << "RPSIS" << Pel.RPSIS << std::endl;

    std::cout << "--------r_Local struct end ----\n" << std::endl;
}

void PRJsonGdlf::R_local_struct::print_data() {
    std::cout << "--------R_local struct---------" << std::endl;
    std::cout << "Femur_O3: \n" << Femur_O3 << std::endl;
    std::cout << "Tibia_O5: \n" << Tibia_O5 << std::endl;
    std::cout << "Pie_O7: \n" << Pie_O7 << std::endl;
    std::cout << "--------R_local struct end ----\n" << std::endl;
}

void PRJsonGdlf::MusPar_struct::print_data() {
	std::cout << "--------MusPar_struct struct---------" << std::endl;

	std::cout << "-----AddBrevDist----- " << std::endl;
	std::cout << "Area" << AddBrevDist.Area << std::endl;
	std::cout << "Ang" << AddBrevDist.Ang << std::endl;

	std::cout << "-----AddBrevMid----- " << std::endl;
	std::cout << "Area" << AddBrevMid.Area << std::endl;
	std::cout << "Ang" << AddBrevMid.Ang << std::endl;

	std::cout << "-----AddBrevPro----- " << std::endl;
	std::cout << "Area" << AddBrevPro.Area << std::endl;
	std::cout << "Ang" << AddBrevPro.Ang << std::endl;

	std::cout << "-----AddLong12----- " << std::endl;
	std::cout << "Area" << AddLong12.Area << std::endl;
	std::cout << "Ang" << AddLong12.Ang << std::endl;

	std::cout << "-----AddLong34----- " << std::endl;
	std::cout << "Area" << AddLong34.Area << std::endl;
	std::cout << "Ang" << AddLong34.Ang << std::endl;

	std::cout << "-----AddLong56----- " << std::endl;
	std::cout << "Area" << AddLong56.Area << std::endl;
	std::cout << "Ang" << AddLong56.Ang << std::endl;

	std::cout << "-----AddMagnDist----- " << std::endl;
	std::cout << "Area" << AddMagnDist.Area << std::endl;
	std::cout << "Ang" << AddMagnDist.Ang << std::endl;

	std::cout << "-----AddMagnMid12----- " << std::endl;
	std::cout << "Area" << AddMagnMid12.Area << std::endl;
	std::cout << "Ang" << AddMagnMid12.Ang << std::endl;

	std::cout << "-----AddMagnMid34----- " << std::endl;
	std::cout << "Area" << AddMagnMid34.Area << std::endl;
	std::cout << "Ang" << AddMagnMid34.Ang << std::endl;

	std::cout << "-----AddMagnMid56----- " << std::endl;
	std::cout << "Area" << AddMagnMid56.Area << std::endl;
	std::cout << "Ang" << AddMagnMid56.Ang << std::endl;

	std::cout << "-----AddMagnProx12----- " << std::endl;
	std::cout << "Area" << AddMagnProx12.Area << std::endl;
	std::cout << "Ang" << AddMagnProx12.Ang << std::endl;

	std::cout << "-----AddMagnProx34----- " << std::endl;
	std::cout << "Area" << AddMagnProx34.Area << std::endl;
	std::cout << "Ang" << AddMagnProx34.Ang << std::endl;

	std::cout << "-----BicFemCB----- " << std::endl;
	std::cout << "Area" << BicFemCB.Area << std::endl;
	std::cout << "Ang" << BicFemCB.Ang << std::endl;

	std::cout << "-----BicFemCL----- " << std::endl;
	std::cout << "Area" << BicFemCL.Area << std::endl;
	std::cout << "Ang" << BicFemCL.Ang << std::endl;

	std::cout << "-----ExtDigLong----- " << std::endl;
	std::cout << "Area" << ExtDigLong.Area << std::endl;
	std::cout << "Ang" << ExtDigLong.Ang << std::endl;

	std::cout << "-----ExtHalLong----- " << std::endl;
	std::cout << "Area" << ExtHalLong.Area << std::endl;
	std::cout << "Ang" << ExtHalLong.Ang << std::endl;

	std::cout << "-----FlxDigLong----- " << std::endl;
	std::cout << "Area" << FlxDigLong.Area << std::endl;
	std::cout << "Ang" << FlxDigLong.Ang << std::endl;

	std::cout << "-----FlxHalLong----- " << std::endl;
	std::cout << "Area" << FlxHalLong.Area << std::endl;
	std::cout << "Ang" << FlxHalLong.Ang << std::endl;

	std::cout << "-----GastLat----- " << std::endl;
	std::cout << "Area" << GastLat.Area << std::endl;
	std::cout << "Ang" << GastLat.Ang << std::endl;

	std::cout << "-----GastMed----- " << std::endl;
	std::cout << "Area" << GastMed.Area << std::endl;
	std::cout << "Ang" << GastMed.Ang << std::endl;

	std::cout << "-----GemellusInf----- " << std::endl;
	std::cout << "Area" << GemellusInf.Area << std::endl;
	std::cout << "Ang" << GemellusInf.Ang << std::endl;

	std::cout << "-----GemellusSup----- " << std::endl;
	std::cout << "Area" << GemellusSup.Area << std::endl;
	std::cout << "Ang" << GemellusSup.Ang << std::endl;

	std::cout << "-----GlutMaxInf12----- " << std::endl;
	std::cout << "Area" << GlutMaxInf12.Area << std::endl;
	std::cout << "Ang" << GlutMaxInf12.Ang << std::endl;

	std::cout << "-----GlutMaxInf34----- " << std::endl;
	std::cout << "Area" << GlutMaxInf34.Area << std::endl;
	std::cout << "Ang" << GlutMaxInf34.Ang << std::endl;

	std::cout << "-----GlutMaxInf5----- " << std::endl;
	std::cout << "Area" << GlutMaxInf5.Area << std::endl;
	std::cout << "Ang" << GlutMaxInf5.Ang << std::endl;

	std::cout << "-----GlutMaxInf6----- " << std::endl;
	std::cout << "Area" << GlutMaxInf6.Area << std::endl;
	std::cout << "Ang" << GlutMaxInf6.Ang << std::endl;

	std::cout << "-----GlutMaxSup14----- " << std::endl;
	std::cout << "Area" << GlutMaxSup14.Area << std::endl;
	std::cout << "Ang" << GlutMaxSup14.Ang << std::endl;

	std::cout << "-----GlutMaxSup25----- " << std::endl;
	std::cout << "Area" << GlutMaxSup25.Area << std::endl;
	std::cout << "Ang" << GlutMaxSup25.Ang << std::endl;

	std::cout << "-----GlutMaxSup36----- " << std::endl;
	std::cout << "Area" << GlutMaxSup36.Area << std::endl;
	std::cout << "Ang" << GlutMaxSup36.Ang << std::endl;

	std::cout << "-----GlutMedAnt123----- " << std::endl;
	std::cout << "Area" << GlutMedAnt123.Area << std::endl;
	std::cout << "Ang" << GlutMedAnt123.Ang << std::endl;

	std::cout << "-----GlutMedAnt456----- " << std::endl;
	std::cout << "Area" << GlutMedAnt456.Area << std::endl;
	std::cout << "Ang" << GlutMedAnt456.Ang << std::endl;

	std::cout << "-----GlutMedPost123----- " << std::endl;
	std::cout << "Area" << GlutMedPost123.Area << std::endl;
	std::cout << "Ang" << GlutMedPost123.Ang << std::endl;

	std::cout << "-----GlutMedPost345----- " << std::endl;
	std::cout << "Area" << GlutMedPost345.Area << std::endl;
	std::cout << "Ang" << GlutMedPost345.Ang << std::endl;

	std::cout << "-----GlutMedPost6----- " << std::endl;
	std::cout << "Area" << GlutMedPost6.Area << std::endl;
	std::cout << "Ang" << GlutMedPost6.Ang << std::endl;

	std::cout << "-----GlutMinLat----- " << std::endl;
	std::cout << "Area" << GlutMinLat.Area << std::endl;
	std::cout << "Ang" << GlutMinLat.Ang << std::endl;

	std::cout << "-----GlutMinMed----- " << std::endl;
	std::cout << "Area" << GlutMinMed.Area << std::endl;
	std::cout << "Ang" << GlutMinMed.Ang << std::endl;

	std::cout << "-----GlutMinMid----- " << std::endl;
	std::cout << "Area" << GlutMinMid.Area << std::endl;
	std::cout << "Ang" << GlutMinMid.Ang << std::endl;

	std::cout << "-----Gra----- " << std::endl;
	std::cout << "Area" << Gra.Area << std::endl;
	std::cout << "Ang" << Gra.Ang << std::endl;

	std::cout << "-----IliacusLat----- " << std::endl;
	std::cout << "Area" << IliacusLat.Area << std::endl;
	std::cout << "Ang" << IliacusLat.Ang << std::endl;

	std::cout << "-----IliacusMed----- " << std::endl;
	std::cout << "Area" << IliacusMed.Area << std::endl;
	std::cout << "Ang" << IliacusMed.Ang << std::endl;

	std::cout << "-----IliacusMid----- " << std::endl;
	std::cout << "Area" << IliacusMid.Area << std::endl;
	std::cout << "Ang" << IliacusMid.Ang << std::endl;

	std::cout << "-----ObtExtInf----- " << std::endl;
	std::cout << "Area" << ObtExtInf.Area << std::endl;
	std::cout << "Ang" << ObtExtInf.Ang << std::endl;

	std::cout << "-----ObtExtSup----- " << std::endl;
	std::cout << "Area" << ObtExtSup.Area << std::endl;
	std::cout << "Ang" << ObtExtSup.Ang << std::endl;

	std::cout << "-----ObtInt----- " << std::endl;
	std::cout << "Area" << ObtInt.Area << std::endl;
	std::cout << "Ang" << ObtInt.Ang << std::endl;

	std::cout << "-----Pectineus12----- " << std::endl;
	std::cout << "Area" << Pectineus12.Area << std::endl;
	std::cout << "Ang" << Pectineus12.Ang << std::endl;

	std::cout << "-----Pectineus34----- " << std::endl;
	std::cout << "Area" << Pectineus34.Area << std::endl;
	std::cout << "Ang" << Pectineus34.Ang << std::endl;

	std::cout << "-----PeroBrev----- " << std::endl;
	std::cout << "Area" << PeroBrev.Area << std::endl;
	std::cout << "Ang" << PeroBrev.Ang << std::endl;

	std::cout << "-----PeroLong----- " << std::endl;
	std::cout << "Area" << PeroLong.Area << std::endl;
	std::cout << "Ang" << PeroLong.Ang << std::endl;

	std::cout << "-----PeroTert----- " << std::endl;
	std::cout << "Area" << PeroTert.Area << std::endl;
	std::cout << "Ang" << PeroTert.Ang << std::endl;

	std::cout << "-----Piriformis----- " << std::endl;
	std::cout << "Area" << Piriformis.Area << std::endl;
	std::cout << "Ang" << Piriformis.Ang << std::endl;

	std::cout << "-----Plantaris----- " << std::endl;
	std::cout << "Area" << Plantaris.Area << std::endl;
	std::cout << "Ang" << Plantaris.Ang << std::endl;

	std::cout << "-----PsoasMajor1----- " << std::endl;
	std::cout << "Area" << PsoasMajor1.Area << std::endl;
	std::cout << "Ang" << PsoasMajor1.Ang << std::endl;

	std::cout << "-----PsoasMajor23----- " << std::endl;
	std::cout << "Area" << PsoasMajor23.Area << std::endl;
	std::cout << "Ang" << PsoasMajor23.Ang << std::endl;

	std::cout << "-----QuadFem12----- " << std::endl;
	std::cout << "Area" << QuadFem12.Area << std::endl;
	std::cout << "Ang" << QuadFem12.Ang << std::endl;

	std::cout << "-----QuadFem34----- " << std::endl;
	std::cout << "Area" << QuadFem34.Area << std::endl;
	std::cout << "Ang" << QuadFem34.Ang << std::endl;

	std::cout << "-----RecFem12_1----- " << std::endl;
	std::cout << "Area" << RecFem12_1.Area << std::endl;
	std::cout << "Ang" << RecFem12_1.Ang << std::endl;

	std::cout << "-----RecFem12_2----- " << std::endl;
	std::cout << "Area" << RecFem12_2.Area << std::endl;
	std::cout << "Ang" << RecFem12_2.Ang << std::endl;

	std::cout << "-----Sat----- " << std::endl;
	std::cout << "Area" << Sat.Area << std::endl;
	std::cout << "Ang" << Sat.Ang << std::endl;

	std::cout << "-----SemMem----- " << std::endl;
	std::cout << "Area" << SemMem.Area << std::endl;
	std::cout << "Ang" << SemMem.Ang << std::endl;

	std::cout << "-----SemTend----- " << std::endl;
	std::cout << "Area" << SemTend.Area << std::endl;
	std::cout << "Ang" << SemTend.Ang << std::endl;

	std::cout << "-----SoleiusLat----- " << std::endl;
	std::cout << "Area" << SoleiusLat.Area << std::endl;
	std::cout << "Ang" << SoleiusLat.Ang << std::endl;

	std::cout << "-----SoleusMed----- " << std::endl;
	std::cout << "Area" << SoleusMed.Area << std::endl;
	std::cout << "Ang" << SoleusMed.Ang << std::endl;

	std::cout << "-----TenFacLat----- " << std::endl;
	std::cout << "Area" << TenFacLat.Area << std::endl;
	std::cout << "Ang" << TenFacLat.Ang << std::endl;

	std::cout << "-----TibialAnt----- " << std::endl;
	std::cout << "Area" << TibialAnt.Area << std::endl;
	std::cout << "Ang" << TibialAnt.Ang << std::endl;

	std::cout << "-----TibialPostLat----- " << std::endl;
	std::cout << "Area" << TibialPostLat.Area << std::endl;
	std::cout << "Ang" << TibialPostLat.Ang << std::endl;

	std::cout << "-----TibialPostMed----- " << std::endl;
	std::cout << "Area" << TibialPostMed.Area << std::endl;
	std::cout << "Ang" << TibialPostMed.Ang << std::endl;

	std::cout << "-----VasInt123----- " << std::endl;
	std::cout << "Area" << VasInt123.Area << std::endl;
	std::cout << "Ang" << VasInt123.Ang << std::endl;

	std::cout << "-----VasInt456----- " << std::endl;
	std::cout << "Area" << VasInt456.Area << std::endl;
	std::cout << "Ang" << VasInt456.Ang << std::endl;

	std::cout << "-----VasLatInf4----- " << std::endl;
	std::cout << "Area" << VasLatInf4.Area << std::endl;
	std::cout << "Ang" << VasLatInf4.Ang << std::endl;

	std::cout << "-----VasLatSup12----- " << std::endl;
	std::cout << "Area" << VasLatSup12.Area << std::endl;
	std::cout << "Ang" << VasLatSup12.Ang << std::endl;

	std::cout << "-----VasMedInf12----- " << std::endl;
	std::cout << "Area" << VasMedInf12.Area << std::endl;
	std::cout << "Ang" << VasMedInf12.Ang << std::endl;

	std::cout << "-----VasMedMed12----- " << std::endl;
	std::cout << "Area" << VasMedMed12.Area << std::endl;
	std::cout << "Ang" << VasMedMed12.Ang << std::endl;

	std::cout << "-----VasMedSup34----- " << std::endl;
	std::cout << "Area" << VasMedSup34.Area << std::endl;
	std::cout << "Ang" << VasMedSup34.Ang << std::endl;

	std::cout << "--------MusPar struct end ----\n" << std::endl;
}

// Constructor. Receives a path and extracts the JSON content
PRJsonGdlf::Gdlf_data_struct::Gdlf_data_struct(std::string path)
    : Data_struct(path)
    {
	std::cout << "Cargando los datos" << std::endl;
	// All structures inside the original are extracted into Values of rapidjson library
	const Value& Plane4Bar_v = struct_value(document, "Plane4Bar");
	//std::cout << "Cargado los datos Plane4Bar" << std::endl;
	const Value& R_local_v = struct_value(document, "R_local");
	//std::cout << "Cargado los datos R_local" << std::endl;
	const Value& r_Local_v = struct_value(document, "r_Local");
	//std::cout << "Cargado los datos r_Local" << std::endl;
	//const Value& Lig_v = struct_value(document, "Lig");
	const Value& MusPar_v = struct_value(document, "MusPar");
	//std::cout << "Cargado MusPar" << std::endl;
	// Structs dentro de r_Local
	const Value& r_Local_Fem_v = struct_value(r_Local_v, "Fem");
	const Value& r_Local_Tib_v = struct_value(r_Local_v, "Tib");
	const Value& r_Local_Pie_v = struct_value(r_Local_v, "Pie");
	const Value& r_Local_Pel_v = struct_value(r_Local_v, "Pel");
	// Leemos los datos internos de r_local Fem
	read_member(r_Local_Fem_v, "CoM", r_Local.Fem.CoM);
	read_member(r_Local_Fem_v, "LFE", r_Local.Fem.LFE);
	read_member(r_Local_Fem_v, "MFE", r_Local.Fem.MFE);
	read_member(r_Local_Fem_v, "O3", r_Local.Fem.O3);
	// Leemos los datos internos de r_local Tib
	read_member(r_Local_Tib_v, "CoM", r_Local.Tib.CoM);
	read_member(r_Local_Tib_v, "LM", r_Local.Tib.LM);
	read_member(r_Local_Tib_v, "MM", r_Local.Tib.MM);
	read_member(r_Local_Tib_v, "FH", r_Local.Tib.FH);
	read_member(r_Local_Tib_v, "O5", r_Local.Tib.O5);
	// Leemos los datos internos de r_local Pie
	read_member(r_Local_Pie_v, "CoM", r_Local.Pie.CoM);
	read_member(r_Local_Pie_v, "MH1", r_Local.Pie.MH1);
	read_member(r_Local_Pie_v, "MH5", r_Local.Pie.MH5);
	read_member(r_Local_Pie_v, "CAL", r_Local.Pie.CAL);
	read_member(r_Local_Pie_v, "O7", r_Local.Pie.O7);
	// Leemos los datos internos de r_local Pel
	read_member(r_Local_Pel_v, "LASIS", r_Local.Pel.LASIS);
	read_member(r_Local_Pel_v, "LPSIS", r_Local.Pel.LPSIS);
	read_member(r_Local_Pel_v, "RASIS", r_Local.Pel.RASIS);
	read_member(r_Local_Pel_v, "RPSIS", r_Local.Pel.RPSIS);

	std::cout << "Variables internas de r_Local Cargado" << std::endl;
	// Structs dentro de MusPar
	const Value& MusPar_v_AddBrevDist = struct_value(MusPar_v, "AddBrevDist");
	const Value& MusPar_v_AddBrevMid = struct_value(MusPar_v, "AddBrevMid");
	const Value& MusPar_v_AddBrevPro = struct_value(MusPar_v, "AddBrevPro");
	const Value& MusPar_v_AddLong12 = struct_value(MusPar_v, "AddLong12");
	const Value& MusPar_v_AddLong34 = struct_value(MusPar_v, "AddLong34");
	const Value& MusPar_v_AddLong56 = struct_value(MusPar_v, "AddLong56");
	const Value& MusPar_v_AddMagnDist = struct_value(MusPar_v, "AddMagnDist");
	const Value& MusPar_v_AddMagnMid12 = struct_value(MusPar_v, "AddMagnMid12");
	const Value& MusPar_v_AddMagnMid34 = struct_value(MusPar_v, "AddMagnMid34");
	const Value& MusPar_v_AddMagnMid56 = struct_value(MusPar_v, "AddMagnMid56");
	const Value& MusPar_v_AddMagnProx12 = struct_value(MusPar_v, "AddMagnProx12");
	const Value& MusPar_v_AddMagnProx34 = struct_value(MusPar_v, "AddMagnProx34");
	const Value& MusPar_v_BicFemCB = struct_value(MusPar_v, "BicFemCB");
	const Value& MusPar_v_BicFemCL = struct_value(MusPar_v, "BicFemCL");
	const Value& MusPar_v_ExtDigLong = struct_value(MusPar_v, "ExtDigLong");
	const Value& MusPar_v_ExtHalLong = struct_value(MusPar_v, "ExtHalLong");
	const Value& MusPar_v_FlxDigLong = struct_value(MusPar_v, "FlxDigLong");
	const Value& MusPar_v_FlxHalLong = struct_value(MusPar_v, "FlxHalLong");
	const Value& MusPar_v_GastLat = struct_value(MusPar_v, "GastLat");
	const Value& MusPar_v_GastMed = struct_value(MusPar_v, "GastMed");
	const Value& MusPar_v_GemellusInf = struct_value(MusPar_v, "GemellusInf");
	const Value& MusPar_v_GemellusSup = struct_value(MusPar_v, "GemellusSup");
	const Value& MusPar_v_GlutMaxInf12 = struct_value(MusPar_v, "GlutMaxInf12");
	const Value& MusPar_v_GlutMaxInf34 = struct_value(MusPar_v, "GlutMaxInf34");
	const Value& MusPar_v_GlutMaxInf5 = struct_value(MusPar_v, "GlutMaxInf5");
	const Value& MusPar_v_GlutMaxInf6 = struct_value(MusPar_v, "GlutMaxInf6");
	const Value& MusPar_v_GlutMaxSup14 = struct_value(MusPar_v, "GlutMaxSup14");
	const Value& MusPar_v_GlutMaxSup25 = struct_value(MusPar_v, "GlutMaxSup25");
	const Value& MusPar_v_GlutMaxSup36 = struct_value(MusPar_v, "GlutMaxSup36");
	const Value& MusPar_v_GlutMedAnt123 = struct_value(MusPar_v, "GlutMedAnt123");
	const Value& MusPar_v_GlutMedAnt456 = struct_value(MusPar_v, "GlutMedAnt456");
	const Value& MusPar_v_GlutMedPost123 = struct_value(MusPar_v, "GlutMedPost123");
	const Value& MusPar_v_GlutMedPost345 = struct_value(MusPar_v, "GlutMedPost345");
	const Value& MusPar_v_GlutMedPost6 = struct_value(MusPar_v, "GlutMedPost6");
	const Value& MusPar_v_GlutMinLat = struct_value(MusPar_v, "GlutMinLat");
	const Value& MusPar_v_GlutMinMed = struct_value(MusPar_v, "GlutMinMed");
	const Value& MusPar_v_GlutMinMid = struct_value(MusPar_v, "GlutMinMid");
	const Value& MusPar_v_Gra = struct_value(MusPar_v, "Gra");
	const Value& MusPar_v_IliacusLat = struct_value(MusPar_v, "IliacusLat");
	const Value& MusPar_v_IliacusMed = struct_value(MusPar_v, "IliacusMed");
	const Value& MusPar_v_IliacusMid = struct_value(MusPar_v, "IliacusMid");
	const Value& MusPar_v_ObtExtInf = struct_value(MusPar_v, "ObtExtInf");
	const Value& MusPar_v_ObtExtSup = struct_value(MusPar_v, "ObtExtSup");
	const Value& MusPar_v_ObtInt = struct_value(MusPar_v, "ObtInt");
	const Value& MusPar_v_Pectineus12 = struct_value(MusPar_v, "Pectineus12");
	const Value& MusPar_v_Pectineus34 = struct_value(MusPar_v, "Pectineus34");
	const Value& MusPar_v_PeroBrev = struct_value(MusPar_v, "PeroBrev");
	const Value& MusPar_v_PeroLong = struct_value(MusPar_v, "PeroLong");
	const Value& MusPar_v_PeroTert = struct_value(MusPar_v, "PeroTert");
	const Value& MusPar_v_Piriformis = struct_value(MusPar_v, "Piriformis");
	const Value& MusPar_v_Plantaris = struct_value(MusPar_v, "Plantaris");
	const Value& MusPar_v_PsoasMajor1 = struct_value(MusPar_v, "PsoasMajor1");
	const Value& MusPar_v_PsoasMajor23 = struct_value(MusPar_v, "PsoasMajor23");
	const Value& MusPar_v_QuadFem12 = struct_value(MusPar_v, "QuadFem12");
	const Value& MusPar_v_QuadFem34 = struct_value(MusPar_v, "QuadFem34");
	const Value& MusPar_v_RecFem12_1 = struct_value(MusPar_v, "RecFem12_1");
	const Value& MusPar_v_RecFem12_2 = struct_value(MusPar_v, "RecFem12_2");
	const Value& MusPar_v_Sat = struct_value(MusPar_v, "Sat");
	const Value& MusPar_v_SemMem = struct_value(MusPar_v, "SemMem");
	const Value& MusPar_v_SemTend = struct_value(MusPar_v, "SemTend");
	const Value& MusPar_v_SoleiusLat = struct_value(MusPar_v, "SoleiusLat");
	const Value& MusPar_v_SoleusMed = struct_value(MusPar_v, "SoleusMed");
	const Value& MusPar_v_TenFacLat = struct_value(MusPar_v, "TenFacLat");
	const Value& MusPar_v_TibialAnt = struct_value(MusPar_v, "TibialAnt");
	const Value& MusPar_v_TibialPostLat = struct_value(MusPar_v, "TibialPostLat");
	const Value& MusPar_v_TibialPostMed = struct_value(MusPar_v, "TibialPostMed");
	const Value& MusPar_v_VasInt123 = struct_value(MusPar_v, "VasInt123");
	const Value& MusPar_v_VasInt456 = struct_value(MusPar_v, "VasInt456");
	const Value& MusPar_v_VasLatInf4 = struct_value(MusPar_v, "VasLatInf4");
	const Value& MusPar_v_VasLatSup12 = struct_value(MusPar_v, "VasLatSup12");
	const Value& MusPar_v_VasMedInf12 = struct_value(MusPar_v, "VasMedInf12");
	const Value& MusPar_v_VasMedMed12 = struct_value(MusPar_v, "VasMedMed12");
	const Value& MusPar_v_VasMedSup34 = struct_value(MusPar_v, "VasMedSup34");

	// Leemos el valor del area
	read_member(MusPar_v_AddBrevDist, "Area", MusPar.AddBrevDist.Area);
	read_member(MusPar_v_AddBrevMid, "Area", MusPar.AddBrevMid.Area);
	read_member(MusPar_v_AddBrevPro, "Area", MusPar.AddBrevPro.Area);
	read_member(MusPar_v_AddLong12, "Area", MusPar.AddLong12.Area);
	read_member(MusPar_v_AddLong34, "Area", MusPar.AddLong34.Area);
	read_member(MusPar_v_AddLong56, "Area", MusPar.AddLong56.Area);
	read_member(MusPar_v_AddMagnDist, "Area", MusPar.AddMagnDist.Area);
	read_member(MusPar_v_AddMagnMid12, "Area", MusPar.AddMagnMid12.Area);
	read_member(MusPar_v_AddMagnMid34, "Area", MusPar.AddMagnMid34.Area);
	read_member(MusPar_v_AddMagnMid56, "Area", MusPar.AddMagnMid56.Area);
	read_member(MusPar_v_AddMagnProx12, "Area", MusPar.AddMagnProx12.Area);
	read_member(MusPar_v_AddMagnProx34, "Area", MusPar.AddMagnProx34.Area);
	read_member(MusPar_v_BicFemCB, "Area", MusPar.BicFemCB.Area);
	read_member(MusPar_v_BicFemCL, "Area", MusPar.BicFemCL.Area);
	read_member(MusPar_v_ExtDigLong, "Area", MusPar.ExtDigLong.Area);
	read_member(MusPar_v_ExtHalLong, "Area", MusPar.ExtHalLong.Area);
	read_member(MusPar_v_FlxDigLong, "Area", MusPar.FlxDigLong.Area);
	read_member(MusPar_v_FlxHalLong, "Area", MusPar.FlxHalLong.Area);
	read_member(MusPar_v_GastLat, "Area", MusPar.GastLat.Area);
	read_member(MusPar_v_GastMed, "Area", MusPar.GastMed.Area);
	read_member(MusPar_v_GemellusInf, "Area", MusPar.GemellusInf.Area);
	read_member(MusPar_v_GemellusSup, "Area", MusPar.GemellusSup.Area);
	read_member(MusPar_v_GlutMaxInf12, "Area", MusPar.GlutMaxInf12.Area);
	read_member(MusPar_v_GlutMaxInf34, "Area", MusPar.GlutMaxInf34.Area);
	read_member(MusPar_v_GlutMaxInf5, "Area", MusPar.GlutMaxInf5.Area);
	read_member(MusPar_v_GlutMaxInf6, "Area", MusPar.GlutMaxInf6.Area);
	read_member(MusPar_v_GlutMaxSup14, "Area", MusPar.GlutMaxSup14.Area);
	read_member(MusPar_v_GlutMaxSup25, "Area", MusPar.GlutMaxSup25.Area);
	read_member(MusPar_v_GlutMaxSup36, "Area", MusPar.GlutMaxSup36.Area);
	read_member(MusPar_v_GlutMedAnt123, "Area", MusPar.GlutMedAnt123.Area);
	read_member(MusPar_v_GlutMedAnt456, "Area", MusPar.GlutMedAnt456.Area);
	read_member(MusPar_v_GlutMedPost123, "Area", MusPar.GlutMedPost123.Area);
	read_member(MusPar_v_GlutMedPost345, "Area", MusPar.GlutMedPost345.Area);
	read_member(MusPar_v_GlutMedPost6, "Area", MusPar.GlutMedPost6.Area);
	read_member(MusPar_v_GlutMinLat, "Area", MusPar.GlutMinLat.Area);
	read_member(MusPar_v_GlutMinMed, "Area", MusPar.GlutMinMed.Area);
	read_member(MusPar_v_GlutMinMid, "Area", MusPar.GlutMinMid.Area);
	read_member(MusPar_v_Gra, "Area", MusPar.Gra.Area);
	read_member(MusPar_v_IliacusLat, "Area", MusPar.IliacusLat.Area);
	read_member(MusPar_v_IliacusMed, "Area", MusPar.IliacusMed.Area);
	read_member(MusPar_v_IliacusMid, "Area", MusPar.IliacusMid.Area);
	read_member(MusPar_v_ObtExtInf, "Area", MusPar.ObtExtInf.Area);
	read_member(MusPar_v_ObtExtSup, "Area", MusPar.ObtExtSup.Area);
	read_member(MusPar_v_ObtInt, "Area", MusPar.ObtInt.Area);
	read_member(MusPar_v_Pectineus12, "Area", MusPar.Pectineus12.Area);
	read_member(MusPar_v_Pectineus34, "Area", MusPar.Pectineus34.Area);
	read_member(MusPar_v_PeroBrev, "Area", MusPar.PeroBrev.Area);
	read_member(MusPar_v_PeroLong, "Area", MusPar.PeroLong.Area);
	read_member(MusPar_v_PeroTert, "Area", MusPar.PeroTert.Area);
	read_member(MusPar_v_Piriformis, "Area", MusPar.Piriformis.Area);
	read_member(MusPar_v_Plantaris, "Area", MusPar.Plantaris.Area);
	read_member(MusPar_v_PsoasMajor1, "Area", MusPar.PsoasMajor1.Area);
	read_member(MusPar_v_PsoasMajor23, "Area", MusPar.PsoasMajor23.Area);
	read_member(MusPar_v_QuadFem12, "Area", MusPar.QuadFem12.Area);
	read_member(MusPar_v_QuadFem34, "Area", MusPar.QuadFem34.Area);
	read_member(MusPar_v_RecFem12_1, "Area", MusPar.RecFem12_1.Area);
	read_member(MusPar_v_RecFem12_2, "Area", MusPar.RecFem12_2.Area);
	read_member(MusPar_v_Sat, "Area", MusPar.Sat.Area);
	read_member(MusPar_v_SemMem, "Area", MusPar.SemMem.Area);
	read_member(MusPar_v_SemTend, "Area", MusPar.SemTend.Area);
	read_member(MusPar_v_SoleiusLat, "Area", MusPar.SoleiusLat.Area);
	read_member(MusPar_v_SoleusMed, "Area", MusPar.SoleusMed.Area);
	read_member(MusPar_v_TenFacLat, "Area", MusPar.TenFacLat.Area);
	read_member(MusPar_v_TibialAnt, "Area", MusPar.TibialAnt.Area);
	read_member(MusPar_v_TibialPostLat, "Area", MusPar.TibialPostLat.Area);
	read_member(MusPar_v_TibialPostMed, "Area", MusPar.TibialPostMed.Area);
	read_member(MusPar_v_VasInt123, "Area", MusPar.VasInt123.Area);
	read_member(MusPar_v_VasInt456, "Area", MusPar.VasInt456.Area);
	read_member(MusPar_v_VasLatInf4, "Area", MusPar.VasLatInf4.Area);
	read_member(MusPar_v_VasLatSup12, "Area", MusPar.VasLatSup12.Area);
	read_member(MusPar_v_VasMedInf12, "Area", MusPar.VasMedInf12.Area);
	read_member(MusPar_v_VasMedMed12, "Area", MusPar.VasMedMed12.Area);
	read_member(MusPar_v_VasMedSup34, "Area", MusPar.VasMedSup34.Area);
	// Leemos el valor del angulo de penación
	read_member(MusPar_v_AddBrevDist, "Ang", MusPar.AddBrevDist.Ang);
	read_member(MusPar_v_AddBrevMid, "Ang", MusPar.AddBrevMid.Ang);
	read_member(MusPar_v_AddBrevPro, "Ang", MusPar.AddBrevPro.Ang);
	read_member(MusPar_v_AddLong12, "Ang", MusPar.AddLong12.Ang);
	read_member(MusPar_v_AddLong34, "Ang", MusPar.AddLong34.Ang);
	read_member(MusPar_v_AddLong56, "Ang", MusPar.AddLong56.Ang);
	read_member(MusPar_v_AddMagnDist, "Ang", MusPar.AddMagnDist.Ang);
	read_member(MusPar_v_AddMagnMid12, "Ang", MusPar.AddMagnMid12.Ang);
	read_member(MusPar_v_AddMagnMid34, "Ang", MusPar.AddMagnMid34.Ang);
	read_member(MusPar_v_AddMagnMid56, "Ang", MusPar.AddMagnMid56.Ang);
	read_member(MusPar_v_AddMagnProx12, "Ang", MusPar.AddMagnProx12.Ang);
	read_member(MusPar_v_AddMagnProx34, "Ang", MusPar.AddMagnProx34.Ang);
	read_member(MusPar_v_BicFemCB, "Ang", MusPar.BicFemCB.Ang);
	read_member(MusPar_v_BicFemCL, "Ang", MusPar.BicFemCL.Ang);
	read_member(MusPar_v_ExtDigLong, "Ang", MusPar.ExtDigLong.Ang);
	read_member(MusPar_v_ExtHalLong, "Ang", MusPar.ExtHalLong.Ang);
	read_member(MusPar_v_FlxDigLong, "Ang", MusPar.FlxDigLong.Ang);
	read_member(MusPar_v_FlxHalLong, "Ang", MusPar.FlxHalLong.Ang);
	read_member(MusPar_v_GastLat, "Ang", MusPar.GastLat.Ang);
	read_member(MusPar_v_GastMed, "Ang", MusPar.GastMed.Ang);
	read_member(MusPar_v_GemellusInf, "Ang", MusPar.GemellusInf.Ang);
	read_member(MusPar_v_GemellusSup, "Ang", MusPar.GemellusSup.Ang);
	read_member(MusPar_v_GlutMaxInf12, "Ang", MusPar.GlutMaxInf12.Ang);
	read_member(MusPar_v_GlutMaxInf34, "Ang", MusPar.GlutMaxInf34.Ang);
	read_member(MusPar_v_GlutMaxInf5, "Ang", MusPar.GlutMaxInf5.Ang);
	read_member(MusPar_v_GlutMaxInf6, "Ang", MusPar.GlutMaxInf6.Ang);
	read_member(MusPar_v_GlutMaxSup14, "Ang", MusPar.GlutMaxSup14.Ang);
	read_member(MusPar_v_GlutMaxSup25, "Ang", MusPar.GlutMaxSup25.Ang);
	read_member(MusPar_v_GlutMaxSup36, "Ang", MusPar.GlutMaxSup36.Ang);
	read_member(MusPar_v_GlutMedAnt123, "Ang", MusPar.GlutMedAnt123.Ang);
	read_member(MusPar_v_GlutMedAnt456, "Ang", MusPar.GlutMedAnt456.Ang);
	read_member(MusPar_v_GlutMedPost123, "Ang", MusPar.GlutMedPost123.Ang);
	read_member(MusPar_v_GlutMedPost345, "Ang", MusPar.GlutMedPost345.Ang);
	read_member(MusPar_v_GlutMedPost6, "Ang", MusPar.GlutMedPost6.Ang);
	read_member(MusPar_v_GlutMinLat, "Ang", MusPar.GlutMinLat.Ang);
	read_member(MusPar_v_GlutMinMed, "Ang", MusPar.GlutMinMed.Ang);
	read_member(MusPar_v_GlutMinMid, "Ang", MusPar.GlutMinMid.Ang);
	read_member(MusPar_v_Gra, "Ang", MusPar.Gra.Ang);
	read_member(MusPar_v_IliacusLat, "Ang", MusPar.IliacusLat.Ang);
	read_member(MusPar_v_IliacusMed, "Ang", MusPar.IliacusMed.Ang);
	read_member(MusPar_v_IliacusMid, "Ang", MusPar.IliacusMid.Ang);
	read_member(MusPar_v_ObtExtInf, "Ang", MusPar.ObtExtInf.Ang);
	read_member(MusPar_v_ObtExtSup, "Ang", MusPar.ObtExtSup.Ang);
	read_member(MusPar_v_ObtInt, "Ang", MusPar.ObtInt.Ang);
	read_member(MusPar_v_Pectineus12, "Ang", MusPar.Pectineus12.Ang);
	read_member(MusPar_v_Pectineus34, "Ang", MusPar.Pectineus34.Ang);
	read_member(MusPar_v_PeroBrev, "Ang", MusPar.PeroBrev.Ang);
	read_member(MusPar_v_PeroLong, "Ang", MusPar.PeroLong.Ang);
	read_member(MusPar_v_PeroTert, "Ang", MusPar.PeroTert.Ang);
	read_member(MusPar_v_Piriformis, "Ang", MusPar.Piriformis.Ang);
	read_member(MusPar_v_Plantaris, "Ang", MusPar.Plantaris.Ang);
	read_member(MusPar_v_PsoasMajor1, "Ang", MusPar.PsoasMajor1.Ang);
	read_member(MusPar_v_PsoasMajor23, "Ang", MusPar.PsoasMajor23.Ang);
	read_member(MusPar_v_QuadFem12, "Ang", MusPar.QuadFem12.Ang);
	read_member(MusPar_v_QuadFem34, "Ang", MusPar.QuadFem34.Ang);
	read_member(MusPar_v_RecFem12_1, "Ang", MusPar.RecFem12_1.Ang);
	read_member(MusPar_v_RecFem12_2, "Ang", MusPar.RecFem12_2.Ang);
	read_member(MusPar_v_Sat, "Ang", MusPar.Sat.Ang);
	read_member(MusPar_v_SemMem, "Ang", MusPar.SemMem.Ang);
	read_member(MusPar_v_SemTend, "Ang", MusPar.SemTend.Ang);
	read_member(MusPar_v_SoleiusLat, "Ang", MusPar.SoleiusLat.Ang);
	read_member(MusPar_v_SoleusMed, "Ang", MusPar.SoleusMed.Ang);
	read_member(MusPar_v_TenFacLat, "Ang", MusPar.TenFacLat.Ang);
	read_member(MusPar_v_TibialAnt, "Ang", MusPar.TibialAnt.Ang);
	read_member(MusPar_v_TibialPostLat, "Ang", MusPar.TibialPostLat.Ang);
	read_member(MusPar_v_TibialPostMed, "Ang", MusPar.TibialPostMed.Ang);
	read_member(MusPar_v_VasInt123, "Ang", MusPar.VasInt123.Ang);
	read_member(MusPar_v_VasInt456, "Ang", MusPar.VasInt456.Ang);
	read_member(MusPar_v_VasLatInf4, "Ang", MusPar.VasLatInf4.Ang);
	read_member(MusPar_v_VasLatSup12, "Ang", MusPar.VasLatSup12.Ang);
	read_member(MusPar_v_VasMedInf12, "Ang", MusPar.VasMedInf12.Ang);
	read_member(MusPar_v_VasMedMed12, "Ang", MusPar.VasMedMed12.Ang);
	read_member(MusPar_v_VasMedSup34, "Ang", MusPar.VasMedSup34.Ang);



	// all members are read from the original document or from the substructures
	read_member(document, "ACSpelvis_r_PelAvg_ACSpelvis", ACSpelvis_r_PelAvg_ACSpelvis);
	read_member(document, "n_q5", n_q5);
	read_member(document, "G_g", G_g);
	read_member(document, "Lengths", Lengths);
	read_member(document, "Widths", Widths);
	read_member(document, "SegMass", SegMass);
	//std::cout << "Cargado n_q5: " << n_q5 <<  std::endl;
	// Redimensionamos las variables
	AllPatCoef.resize(9, n_q5);
	AllTauGrav.resize(6, n_q5);
	MusCoef1.resize(73, n_q5);
	MusCoef2.resize(73, n_q5);
	MusCoef3.resize(73, n_q5);
	MusCoef4.resize(73, n_q5);
	MusCoef5.resize(73, n_q5);
	MusCoef6.resize(73, n_q5);
	q5.resize(n_q5);	
	//read_member(document, "AllKneeCoef", AllKneeCoef);
	read_member(document, "AllPatCoef", AllPatCoef);
	read_member(document, "AllTauGrav", AllTauGrav);
	read_member(document, "MusCoef1", MusCoef1);
	read_member(document, "MusCoef2", MusCoef2);
	read_member(document, "MusCoef3", MusCoef3);
	read_member(document, "MusCoef4", MusCoef4);
	read_member(document, "MusCoef5", MusCoef5);
	read_member(document, "MusCoef6", MusCoef6);
	read_member(document, "AllKneeCoef", AllKneeCoef);
	//for (int i = 0; i < AllKneeCoef.size(); i++) {

	//	std::cout << AllKneeCoef[i] << std::endl << std::endl;

	//}
	read_member(document, "q5", q5);
//	std::cout << "Variable q5 cargada: " << q5.transpose() << std::endl;

	std::cout << "Variables doubles cargadas" << std::endl;

	//// Cargamos la informacion de los structs dentro de sus valores
	read_member(Plane4Bar_v, "LMO4_u", Plane4Bar.LMO4_u);
	read_member(Plane4Bar_v, "L", Plane4Bar.L);
	read_member(Plane4Bar_v, "DELTA", Plane4Bar.DELTA);
	read_member(Plane4Bar_v, "u_R_LMO4", Plane4Bar.u_R_LMO4);
	read_member(Plane4Bar_v, "ThetaL1X3", Plane4Bar.ThetaL1X3);
	read_member(Plane4Bar_v, "ThetaL3X5", Plane4Bar.ThetaL3X5);
	//std::cout << "Structs cargados" << std::endl;

	// Calculamos el area efectiva de los músculosAreaEfect[0]=AddBrevDist_struc.Area
	AreaEfect[0] = MusPar.BicFemCL.Area * cos(MusPar.BicFemCL.Ang);
	AreaEfect[1] = MusPar.BicFemCB.Area * cos(MusPar.BicFemCB.Ang);
	AreaEfect[2] = MusPar.GastLat.Area * cos(MusPar.GastLat.Ang);
	AreaEfect[3] = MusPar.GastMed.Area * cos(MusPar.GastMed.Ang);
	AreaEfect[4] = MusPar.Gra.Area * cos(MusPar.Gra.Ang);
	AreaEfect[5] = MusPar.RecFem12_1.Area * cos(MusPar.RecFem12_1.Ang);
	AreaEfect[6] = MusPar.RecFem12_2.Area * cos(MusPar.RecFem12_2.Ang);
	AreaEfect[7] = MusPar.Sat.Area * cos(MusPar.Sat.Ang);
	AreaEfect[8] = MusPar.SemMem.Area * cos(MusPar.SemMem.Ang);
	AreaEfect[9] = MusPar.SemTend.Area * cos(MusPar.SemTend.Ang);
	AreaEfect[10] = MusPar.TenFacLat.Area * cos(MusPar.TenFacLat.Ang);
	AreaEfect[11] = MusPar.VasInt123.Area * cos(MusPar.VasInt123.Ang);
	AreaEfect[12] = MusPar.VasInt456.Area * cos(MusPar.VasInt456.Ang);
	AreaEfect[13] = MusPar.VasLatInf4.Area * cos(MusPar.VasLatInf4.Ang);
	AreaEfect[14] = MusPar.VasLatSup12.Area * cos(MusPar.VasLatSup12.Ang);
	AreaEfect[15] = MusPar.VasMedInf12.Area * cos(MusPar.VasMedInf12.Ang);
	AreaEfect[16] = MusPar.VasMedMed12.Area * cos(MusPar.VasMedMed12.Ang);
	AreaEfect[17] = MusPar.VasMedSup34.Area * cos(MusPar.VasMedSup34.Ang);
	AreaEfect[18] = MusPar.IliacusLat.Area * cos(MusPar.IliacusLat.Ang);
	AreaEfect[19] = MusPar.IliacusMid.Area * cos(MusPar.IliacusMid.Ang);
	AreaEfect[20] = MusPar.IliacusMed.Area * cos(MusPar.IliacusMed.Ang);
	AreaEfect[21] = MusPar.PsoasMajor1.Area * cos(MusPar.PsoasMajor1.Ang);
	AreaEfect[22] = MusPar.PsoasMajor23.Area * cos(MusPar.PsoasMajor23.Ang);
	AreaEfect[23] = MusPar.AddBrevPro.Area * cos(MusPar.AddBrevPro.Ang);
	AreaEfect[24] = MusPar.AddBrevMid.Area * cos(MusPar.AddBrevMid.Ang);
	AreaEfect[25] = MusPar.AddBrevDist.Area * cos(MusPar.AddBrevDist.Ang);
	AreaEfect[26] = MusPar.AddLong12.Area * cos(MusPar.AddLong12.Ang);
	AreaEfect[27] = MusPar.AddLong34.Area * cos(MusPar.AddLong34.Ang);
	AreaEfect[28] = MusPar.AddLong56.Area * cos(MusPar.AddLong56.Ang);
	AreaEfect[29] = MusPar.AddMagnDist.Area * cos(MusPar.AddMagnDist.Ang);
	AreaEfect[30] = MusPar.AddMagnMid12.Area * cos(MusPar.AddMagnMid12.Ang);
	AreaEfect[31] = MusPar.AddMagnMid34.Area * cos(MusPar.AddMagnMid34.Ang);
	AreaEfect[32] = MusPar.AddMagnMid56.Area * cos(MusPar.AddMagnMid56.Ang);
	AreaEfect[33] = MusPar.AddMagnProx12.Area * cos(MusPar.AddMagnProx12.Ang);
	AreaEfect[34] = MusPar.AddMagnProx34.Area * cos(MusPar.AddMagnProx34.Ang);
	AreaEfect[35] = MusPar.GemellusInf.Area * cos(MusPar.GemellusInf.Ang);
	AreaEfect[36] = MusPar.GemellusSup.Area * cos(MusPar.GemellusSup.Ang);
	AreaEfect[37] = MusPar.GlutMaxSup14.Area * cos(MusPar.GlutMaxSup14.Ang);
	AreaEfect[38] = MusPar.GlutMaxSup25.Area * cos(MusPar.GlutMaxSup25.Ang);
	AreaEfect[39] = MusPar.GlutMaxSup36.Area * cos(MusPar.GlutMaxSup36.Ang);
	AreaEfect[40] = MusPar.GlutMaxInf12.Area * cos(MusPar.GlutMaxInf12.Ang);
	AreaEfect[41] = MusPar.GlutMaxInf34.Area * cos(MusPar.GlutMaxInf34.Ang);
	AreaEfect[42] = MusPar.GlutMaxInf5.Area * cos(MusPar.GlutMaxInf5.Ang);
	AreaEfect[43] = MusPar.GlutMaxInf6.Area * cos(MusPar.GlutMaxInf6.Ang);
	AreaEfect[44] = MusPar.GlutMedAnt123.Area * cos(MusPar.GlutMedAnt123.Ang);
	AreaEfect[45] = MusPar.GlutMedAnt456.Area * cos(MusPar.GlutMedAnt456.Ang);
	AreaEfect[46] = MusPar.GlutMedPost123.Area * cos(MusPar.GlutMedPost123.Ang);
	AreaEfect[47] = MusPar.GlutMedPost345.Area * cos(MusPar.GlutMedPost345.Ang);
	AreaEfect[48] = MusPar.GlutMedPost6.Area * cos(MusPar.GlutMedPost6.Ang);
	AreaEfect[49] = MusPar.GlutMinLat.Area * cos(MusPar.GlutMinLat.Ang);
	AreaEfect[50] = MusPar.GlutMinMid.Area * cos(MusPar.GlutMinMid.Ang);
	AreaEfect[51] = MusPar.GlutMinMed.Area * cos(MusPar.GlutMinMed.Ang);
	AreaEfect[52] = MusPar.ObtExtInf.Area * cos(MusPar.ObtExtInf.Ang);
	AreaEfect[53] = MusPar.Pectineus12.Area * cos(MusPar.Pectineus12.Ang);
	AreaEfect[54] = MusPar.Pectineus34.Area * cos(MusPar.Pectineus34.Ang);
	AreaEfect[55] = MusPar.Piriformis.Area * cos(MusPar.Piriformis.Ang);
	AreaEfect[56] = MusPar.QuadFem12.Area * cos(MusPar.QuadFem12.Ang);
	AreaEfect[57] = MusPar.QuadFem34.Area * cos(MusPar.QuadFem34.Ang);
	AreaEfect[58] = MusPar.ObtExtSup.Area * cos(MusPar.ObtExtSup.Ang);
	AreaEfect[59] = MusPar.ObtInt.Area * cos(MusPar.ObtInt.Ang);
	AreaEfect[60] = MusPar.ExtDigLong.Area * cos(MusPar.ExtDigLong.Ang);
	AreaEfect[61] = MusPar.ExtHalLong.Area * cos(MusPar.ExtHalLong.Ang);
	AreaEfect[62] = MusPar.FlxDigLong.Area * cos(MusPar.FlxDigLong.Ang);
	AreaEfect[63] = MusPar.FlxHalLong.Area * cos(MusPar.FlxHalLong.Ang);
	AreaEfect[64] = MusPar.PeroBrev.Area * cos(MusPar.PeroBrev.Ang);
	AreaEfect[65] = MusPar.PeroLong.Area * cos(MusPar.PeroLong.Ang);
	AreaEfect[66] = MusPar.PeroTert.Area * cos(MusPar.PeroTert.Ang);
	AreaEfect[67] = MusPar.Plantaris.Area * cos(MusPar.Plantaris.Ang);
	AreaEfect[68] = MusPar.SoleusMed.Area * cos(MusPar.SoleusMed.Ang);
	AreaEfect[69] = MusPar.SoleiusLat.Area * cos(MusPar.SoleiusLat.Ang);
	AreaEfect[70] = MusPar.TibialAnt.Area * cos(MusPar.TibialAnt.Ang);
	AreaEfect[71] = MusPar.TibialPostMed.Area * cos(MusPar.TibialPostMed.Ang);
	AreaEfect[72] = MusPar.TibialPostLat.Area * cos(MusPar.TibialPostLat.Ang);

	//std::cout << "Area Efectiva: " << AreaEfect.transpose() << std::endl;
	

    if (correct_reading==false){
        std::cout << "Something could not be read. Non correct readings:" << std::endl;
        for (int i=0; i<non_correct.size(); i++) std::cout <<  non_correct[i] << " "; 
        throw std::string("Gdlf data reading not correct");
    }
}


void PRJsonGdlf::Gdlf_data_struct::print_data(){
    std::cout << "-----------------------------------GDLF data --------------------------------------\n" << std::endl;
	Data_struct::print_data();
	std::cout << "ACSpelvis_r_PelAvg_ACSpelvis (transposed): " << ACSpelvis_r_PelAvg_ACSpelvis.transpose() << std::endl;
	//std::cout << "AllKneeCoef: " << AllKneeCoef << std::endl;
	std::cout << "AllPatCoef: " << AllPatCoef << std::endl;
	std::cout << "AllTauGrav: " << AllTauGrav << std::endl;
	std::cout << "DH_parameters:" << DH_parameters << std::endl;
	std::cout << "G_g.transpose(): " << G_g.transpose() << std::endl;
	std::cout << "Lengths.transpose(): " << Lengths.transpose() << std::endl;
	std::cout << "Widths.transpose(): " << Widths.transpose() << std::endl;
	std::cout << "MusCoef1:\n" << MusCoef1 << std::endl;
	std::cout << "MusCoef2:\n" << MusCoef2 << std::endl;
	std::cout << "MusCoef3:\n" << MusCoef3 << std::endl;
	std::cout << "MusCoef4:\n" << MusCoef4 << std::endl;
	std::cout << "MusCoef5:\n" << MusCoef5 << std::endl;
	std::cout << "MusCoef6:\n" << MusCoef6 << std::endl;
	std::cout << "q5.transpose(): " << q5.transpose() << std::endl;
	std::cout << "SegMass.transpose(): " << SegMass.transpose() << std::endl;

	//Lig.print_data();
	Plane4Bar.print_data();
	r_Local.print_data();
	R_local.print_data();
	//MusPar.print_data();
	std::cout << "----------------------------------- GDLF data --------------------------------------\n" << std::endl;

}