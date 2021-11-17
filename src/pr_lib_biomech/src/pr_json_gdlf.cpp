#include "pr_lib_biomech/pr_json_gdlf.hpp"

using namespace PRJsonData;

void PRJsonGdlf::A_r_B_struct::print_data(){
    std::cout << "--------A_r_B struct---------" << std::endl;
    std::cout << "CPL (1st col transposed): " << CPL.col(0).transpose() << std::endl;
    std::cout << "CPL_BicFemCBTib (1st col transposed): " << CPL_BicFemCBTib.col(0).transpose() << std::endl;
    std::cout << "CPL_BicFemCLTib (1st col transposed): " << CPL_BicFemCLTib.col(0).transpose() << std::endl;
    std::cout << "CPL_GastLatFoot (1st col transposed): " << CPL_GastLatFoot.col(0).transpose() << std::endl;
    std::cout << "CPL_GastMedFoot (1st col transposed): " << CPL_GastMedFoot.col(0).transpose() << std::endl;
    std::cout << "CPL_Gpie_r (1st col transposed): " << CPL_Gpie_r.col(0).transpose() << std::endl;
    std::cout << "CPL_GraTib (1st col transposed): " << CPL_GraTib.col(0).transpose() << std::endl;
    std::cout << "CPL_PatLigTib (1st col transposed): " << CPL_PatLigTib.col(0).transpose() << std::endl;
    std::cout << "CPL_SatTib (1st col transposed): " << CPL_SatTib.col(0).transpose() << std::endl;
    std::cout << "CPL_SemMemTib (1st col transposed): " << CPL_SemMemTib.col(0).transpose() << std::endl;
    std::cout << "CPL_SemTendTib (1st col transposed): " << CPL_SemTendTib.col(0).transpose() << std::endl;
    std::cout << "CPL_TenFacLatTib (1st col transposed): " << CPL_TenFacLatTib.col(0).transpose() << std::endl;
    std::cout << "CPL_g6 (1st col transposed): " << CPL_g6.col(0).transpose() << std::endl;
    std::cout << "CPM (1st col transposed): " << CPM.col(0).transpose() << std::endl;
    std::cout << "CPM_BicFemCBTib (1st col transposed): " << CPM_BicFemCBTib.col(0).transpose() << std::endl;
    std::cout << "CPM_BicFemCLTib (1st col transposed): " << CPM_BicFemCLTib.col(0).transpose() << std::endl;
    std::cout << "CPM_GastLatFoot (1st col transposed): " << CPM_GastLatFoot.col(0).transpose() << std::endl;
    std::cout << "CPM_GastMedFoot (1st col transposed): " << CPM_GastMedFoot.col(0).transpose() << std::endl;
    std::cout << "CPM_Gpie_r (1st col transposed): " << CPM_Gpie_r.col(0).transpose() << std::endl;
    std::cout << "CPM_GraTib (1st col transposed): " << CPM_GraTib.col(0).transpose() << std::endl;
    std::cout << "CPM_PatLigTib (1st col transposed): " << CPM_PatLigTib.col(0).transpose() << std::endl;
    std::cout << "CPM_SatTib (1st col transposed): " << CPM_SatTib.col(0).transpose() << std::endl;
    std::cout << "CPM_SemMemTib (1st col transposed): " << CPM_SemMemTib.col(0).transpose() << std::endl;
    std::cout << "CPM_SemTendTib (1st col transposed): " << CPM_SemTendTib.col(0).transpose() << std::endl;
    std::cout << "CPM_TenFacLatTib (1st col transposed): " << CPM_TenFacLatTib.col(0).transpose() << std::endl;
    std::cout << "CPM_g6 (1st col transposed): " << CPM_g6.col(0).transpose() << std::endl;
    std::cout << "KJC (1st col transposed): " << KJC.col(0).transpose() << std::endl;
    std::cout << "KJC_BicFemCBTib (1st col transposed): " << KJC_BicFemCBTib.col(0).transpose() << std::endl;
    std::cout << "KJC_BicFemCLTib (1st col transposed): " << KJC_BicFemCLTib.col(0).transpose() << std::endl;
    std::cout << "KJC_GastLatFoot (1st col transposed): " << KJC_GastLatFoot.col(0).transpose() << std::endl;
    std::cout << "KJC_GastMedFoot (1st col transposed): " << KJC_GastMedFoot.col(0).transpose() << std::endl;
    std::cout << "KJC_Gpie_r (1st col transposed): " << KJC_Gpie_r.col(0).transpose() << std::endl;
    std::cout << "KJC_GraTib (1st col transposed): " << KJC_GraTib.col(0).transpose() << std::endl;
    std::cout << "KJC_PatLigTib (1st col transposed): " << KJC_PatLigTib.col(0).transpose() << std::endl;
    std::cout << "KJC_SatTib (1st col transposed): " << KJC_SatTib.col(0).transpose() << std::endl;
    std::cout << "KJC_SemMemTib (1st col transposed): " << KJC_SemMemTib.col(0).transpose() << std::endl;
    std::cout << "KJC_SemTendTib (1st col transposed): " << KJC_SemTendTib.col(0).transpose() << std::endl;
    std::cout << "KJC_TenFacLatTib (1st col transposed): " << KJC_TenFacLatTib.col(0).transpose() << std::endl;
    std::cout << "KJC_g6 (1st col transposed): " << KJC_g6.col(0).transpose() << std::endl;
    std::cout << "--------A_r_B struct end ----\n" << std::endl;
}

void PRJsonGdlf::G_uMatPelvis_struct::print_data(){
    std::cout << "--------A_r_B struct---------" << std::endl;
    std::cout << "Flx.SemTend.Tib_Pel (1st col transposed): " << Flx.SemTend.Tib_Pel.col(0).transpose() << std::endl;
    std::cout << "Flx.SemMem.Tib_Pel (1st col transposed): " << Flx.SemMem.Tib_Pel.col(0).transpose() << std::endl;
    std::cout << "Flx.BicFemCB.Tib_Fem (1st col transposed): " << Flx.BicFemCB.Tib_Fem.col(0).transpose() << std::endl;
    std::cout << "Flx.GastLat.Foot_Fem (1st col transposed): " << Flx.GastLat.Foot_Fem.col(0).transpose() << std::endl;
    std::cout << "Flx.GastMed.Foot_Fem (1st col transposed): " << Flx.GastMed.Foot_Fem.col(0).transpose() << std::endl;
    std::cout << "Flx.BicFemCL.Tib_Pel (1st col transposed): " << Flx.BicFemCL.Tib_Pel.col(0).transpose() << std::endl;
    std::cout << "Flx.Sat.Tib_Pel (1st col transposed): " << Flx.Sat.Tib_Pel.col(0).transpose() << std::endl;
    std::cout << "Flx.Gra.Tib_Pel (1st col transposed): " << Flx.Gra.Tib_Pel.col(0).transpose() << std::endl;
    std::cout << "Ext.PatLig.Tib_Pat (1st col transposed): " << Ext.PatLig.Tib_Pat.col(0).transpose() << std::endl;
    std::cout << "Ext.TenFacLat.Tib_Pel (1st col transposed): " << Ext.TenFacLat.Tib_Pel.col(0).transpose() << std::endl;
}

// Constructor. Receives a path and extracts the JSON content
PRJsonGdlf::Gdlf_data_struct::Gdlf_data_struct(std::string path)
    : Data_struct(path)
    {

    // All structures inside the original are extracted into Values of rapidjson library
    const Value &A_r_B_v = struct_value(document, "A_r_B");
    const Value &G_uMatPelvis_v = struct_value(document, "G_uMatPelvis");
    const Value &Flx_v = struct_value(G_uMatPelvis_v, "Flx");
    const Value &SemTend_v = struct_value(Flx_v, "SemTend");
    const Value &SemMem_v = struct_value(Flx_v, "SemMem");
    const Value &BicFemCB_v = struct_value(Flx_v, "BicFemCB");
    const Value &GastLat_v = struct_value(Flx_v, "GastLat");
    const Value &GastMed_v = struct_value(Flx_v, "GastMed");
    const Value &BicFemCL_v = struct_value(Flx_v, "BicFemCL");
    const Value &Sat_v = struct_value(Flx_v, "Sat");
    const Value &Gra_v = struct_value(Flx_v, "Gra");
    const Value &Ext_v = struct_value(G_uMatPelvis_v, "Ext");
    const Value &PatLig_v = struct_value(Ext_v, "PatLig");
    const Value &TenFacLat_v = struct_value(Ext_v, "TenFacLat");

    // all members are read from the original document or from the substructures
    read_member(A_r_B_v, "CPL", A_r_B.CPL);
    read_member(A_r_B_v, "CPL_BicFemCBTib", A_r_B.CPL_BicFemCBTib);
    read_member(A_r_B_v, "CPL_BicFemCLTib", A_r_B.CPL_BicFemCLTib);
    read_member(A_r_B_v, "CPL_GastLatFoot", A_r_B.CPL_GastLatFoot);
    read_member(A_r_B_v, "CPL_GastMedFoot", A_r_B.CPL_GastMedFoot);
    read_member(A_r_B_v, "CPL_Gpie_r", A_r_B.CPL_Gpie_r);
    read_member(A_r_B_v, "CPL_GraTib", A_r_B.CPL_GraTib);
    read_member(A_r_B_v, "CPL_PatLigTib", A_r_B.CPL_PatLigTib);
    read_member(A_r_B_v, "CPL_SatTib", A_r_B.CPL_SatTib);
    read_member(A_r_B_v, "CPL_SemMemTib", A_r_B.CPL_SemMemTib);
    read_member(A_r_B_v, "CPL_SemTendTib", A_r_B.CPL_SemTendTib);
    read_member(A_r_B_v, "CPL_TenFacLatTib", A_r_B.CPL_TenFacLatTib);
    read_member(A_r_B_v, "CPL_g6", A_r_B.CPL_g6);
    read_member(A_r_B_v, "CPM", A_r_B.CPM);
    read_member(A_r_B_v, "CPM_BicFemCBTib", A_r_B.CPM_BicFemCBTib);
    read_member(A_r_B_v, "CPM_BicFemCLTib", A_r_B.CPM_BicFemCLTib);
    read_member(A_r_B_v, "CPM_GastLatFoot", A_r_B.CPM_GastLatFoot);
    read_member(A_r_B_v, "CPM_GastMedFoot", A_r_B.CPM_GastMedFoot);
    read_member(A_r_B_v, "CPM_Gpie_r", A_r_B.CPM_Gpie_r);
    read_member(A_r_B_v, "CPM_GraTib", A_r_B.CPM_GraTib);
    read_member(A_r_B_v, "CPM_PatLigTib", A_r_B.CPM_PatLigTib);
    read_member(A_r_B_v, "CPM_SatTib", A_r_B.CPM_SatTib);
    read_member(A_r_B_v, "CPM_SemMemTib", A_r_B.CPM_SemMemTib);
    read_member(A_r_B_v, "CPM_SemTendTib", A_r_B.CPM_SemTendTib);
    read_member(A_r_B_v, "CPM_TenFacLatTib", A_r_B.CPM_TenFacLatTib);
    read_member(A_r_B_v, "CPM_g6", A_r_B.CPM_g6);
    read_member(A_r_B_v, "KJC", A_r_B.KJC);
    read_member(A_r_B_v, "KJC_BicFemCBTib", A_r_B.KJC_BicFemCBTib);
    read_member(A_r_B_v, "KJC_BicFemCLTib", A_r_B.KJC_BicFemCLTib);
    read_member(A_r_B_v, "KJC_GastLatFoot", A_r_B.KJC_GastLatFoot);
    read_member(A_r_B_v, "KJC_GastMedFoot", A_r_B.KJC_GastMedFoot);
    read_member(A_r_B_v, "KJC_Gpie_r", A_r_B.KJC_Gpie_r);
    read_member(A_r_B_v, "KJC_GraTib", A_r_B.KJC_GraTib);
    read_member(A_r_B_v, "KJC_PatLigTib", A_r_B.KJC_PatLigTib);
    read_member(A_r_B_v, "KJC_SatTib", A_r_B.KJC_SatTib);
    read_member(A_r_B_v, "KJC_SemMemTib", A_r_B.KJC_SemMemTib);
    read_member(A_r_B_v, "KJC_SemTendTib", A_r_B.KJC_SemTendTib);
    read_member(A_r_B_v, "KJC_TenFacLatTib", A_r_B.KJC_TenFacLatTib);
    read_member(A_r_B_v, "KJC_g6", A_r_B.KJC_g6);

    read_member(document, "AllExtCoefMat2", AllExtCoefMat2);
    read_member(document, "AllFlxCoefMat", AllFlxCoefMat);

    read_member(SemTend_v, "Tib_Pel", G_uMatPelvis.Flx.SemTend.Tib_Pel);
    read_member(SemMem_v, "Tib_Pel", G_uMatPelvis.Flx.SemMem.Tib_Pel);
    read_member(BicFemCB_v, "Tib_Fem", G_uMatPelvis.Flx.BicFemCB.Tib_Fem);
    read_member(GastLat_v, "Foot_Fem", G_uMatPelvis.Flx.GastLat.Foot_Fem);
    read_member(GastMed_v, "Foot_Fem", G_uMatPelvis.Flx.GastMed.Foot_Fem);
    read_member(BicFemCL_v, "Tib_Pel", G_uMatPelvis.Flx.BicFemCL.Tib_Pel);
    read_member(Sat_v, "Tib_Pel", G_uMatPelvis.Flx.Sat.Tib_Pel);
    read_member(Gra_v, "Tib_Pel", G_uMatPelvis.Flx.Gra.Tib_Pel);
    read_member(PatLig_v, "Tib_Pat", G_uMatPelvis.Ext.PatLig.Tib_Pat);
    read_member(TenFacLat_v, "Tib_Pel", G_uMatPelvis.Ext.TenFacLat.Tib_Pel);

    read_member(document, "PatLigCoefMat", PatLigCoefMat);
    read_member(document, "q5", q5);

    if (correct_reading==false){
        std::cout << "Something could not be read. Non correct readings:" << std::endl;
        for (int i=0; i<non_correct.size(); i++) std::cout <<  non_correct[i] << " "; 
        throw std::string("Gdlf data reading not correct");
    }
}

void PRJsonGdlf::Gdlf_data_struct::print_data(){
    std::cout << "-----------------------------------GDLF data --------------------------------------\n" << std::endl;
    Data_struct::print_data();
    A_r_B.print_data();
    std::cout << "AllExtCoefMat2 (1st col transposed): " << AllExtCoefMat2.col(0).transpose() << std::endl;
    std::cout << "AllFlxCoefMat (1st col transposed): " << AllFlxCoefMat.col(0).transpose() << std::endl;
    G_uMatPelvis.print_data();
    std::cout << "PatLigCoefMat (1st col transposed): " << PatLigCoefMat.col(0).transpose() << std::endl;
    std::cout << "q5: " << q5.transpose() << std::endl;    
    std::cout << "-----------------------------------GDLF data --------------------------------------\n" << std::endl;
}