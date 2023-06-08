#include "pr_lib_biomech/pr_json_calibration.hpp"

using namespace PRJsonData;

void PRJsonCal::Plane4Bar_struct::print_data(){
    std::cout << "--------Plane4Bar struct---------" << std::endl;
    std::cout << "LMO4_u (transposed): " << LMO4_u.transpose() << std::endl;
    std::cout << "L (transpose): " << L.transpose() << std::endl;
    std::cout << "DELTA: " << DELTA << std::endl;
    std::cout << "u_R_LMO4:\n" << u_R_LMO4 << std::endl;
    std::cout << "ThetaL1X3: " << ThetaL1X3 << std::endl;
    std::cout << "ThetaL3X5: " << ThetaL3X5 << std::endl;
    std::cout << "--------Plane4Bar struct end ----\n" << std::endl;
}

void PRJsonCal::R_local_struct::print_data(){
    std::cout << "--------R_local struct---------" << std::endl;
    std::cout << "Femur_O3: \n" << Femur_O3 << std::endl;
    std::cout << "Tibia_O5: \n" << Tibia_O5 << std::endl;
    std::cout << "Pie_O7: \n" << Pie_O7 << std::endl;
    std::cout << "--------R_local struct end ----\n" << std::endl;
}

void PRJsonCal::r_Local_struct::print_data() {
    std::cout << "--------r_Local struct---------" << std::endl;
    std::cout << "-----Femur----- " << std::endl;
   
    std::cout << "CoM" << Fem.CoM << std::endl;
    std::cout << "LFE" << Fem.LFE << std::endl;
    std::cout << "MFE" << Fem.MFE << std::endl;
    std::cout << "O3"  << Fem.O3 << std::endl;

    std::cout << "-----Tibia----- " << std::endl;
    std::cout << "CoM" << Tib.CoM << std::endl;
    std::cout << "LM"  << Tib.LM << std::endl;
    std::cout << "MM"  << Tib.MM << std::endl;
    std::cout << "FH"  << Tib.FH << std::endl;
    std::cout << "O5"  << Tib.O5 << std::endl;

    std::cout << "-----Pie----- " << std::endl;
    std::cout << "CoM" << Pie.CoM << std::endl;
    std::cout << "MH1" << Pie.MH1 << std::endl;
    std::cout << "MH5" << Pie.MH5 << std::endl;
    std::cout << "CAL" << Pie.CAL << std::endl;
    std::cout << "O7"  << Pie.O7 << std::endl;

    std::cout << "-----Pelvis----- " << std::endl;
    std::cout << "LASIS" << Pel.LASIS << std::endl;
    std::cout << "RASIS" << Pel.RASIS << std::endl;
    std::cout << "LPSIS" << Pel.LPSIS << std::endl;
    std::cout << "RPSIS" << Pel.RPSIS << std::endl;

    std::cout << "--------r_Local struct end ----\n" << std::endl;
}

// Constructor. Receives a path and extracts the JSON content

    PRJsonCal::Calibration_data_struct::Calibration_data_struct(std::string path)
        : Data_struct(path) {

    // All structures inside the original are extracted into Values of rapidjson library
    const Value& Plane4Bar_v = struct_value(document,"Plane4Bar");
    const Value& R_local_v = struct_value(document,"R_local");
    const Value& r_Local_v = struct_value(document, "r_Local");

    const Value& r_Local_Fem_v = struct_value(r_Local_v, "Fem");
    const Value& r_Local_Tib_v = struct_value(r_Local_v, "Tib");
    const Value& r_Local_Pie_v = struct_value(r_Local_v, "Pie");
    const Value& r_Local_Pel_v = struct_value(r_Local_v, "Pel");

    // all members are read from the original document or from the substructures
    read_member(document, "ACSpelvis_r_PelAvg_ACSpelvis", ACSpelvis_r_PelAvg_ACSpelvis);
    read_member(document, "DH_parameters", DH_parameters);

    read_member(Plane4Bar_v, "LMO4_u"   , Plane4Bar.LMO4_u);
    read_member(Plane4Bar_v, "L"        , Plane4Bar.L);
    read_member(Plane4Bar_v, "DELTA"    , Plane4Bar.DELTA);
    read_member(Plane4Bar_v, "u_R_LMO4" , Plane4Bar.u_R_LMO4);
    read_member(Plane4Bar_v, "ThetaL1X3", Plane4Bar.ThetaL1X3);
    read_member(Plane4Bar_v, "ThetaL3X5", Plane4Bar.ThetaL3X5);

    read_member(r_Local_Fem_v, "CoM", r_Local.Fem.CoM);
    read_member(r_Local_Fem_v, "LFE", r_Local.Fem.LFE);
    read_member(r_Local_Fem_v, "MFE", r_Local.Fem.MFE);
    read_member(r_Local_Fem_v, "O3", r_Local.Fem.O3);

    read_member(r_Local_Tib_v, "CoM", r_Local.Tib.CoM);
    read_member(r_Local_Tib_v, "MM" , r_Local.Tib.MM);
    read_member(r_Local_Tib_v, "LM" , r_Local.Tib.LM);
    read_member(r_Local_Tib_v, "FH" , r_Local.Tib.FH);
    read_member(r_Local_Tib_v, "O5" , r_Local.Tib.O5);

    read_member(r_Local_Pie_v, "CoM", r_Local.Pie.CoM);
    read_member(r_Local_Pie_v, "MH1", r_Local.Pie.MH1);
    read_member(r_Local_Pie_v, "MH5", r_Local.Pie.MH5);
    read_member(r_Local_Pie_v, "CAL", r_Local.Pie.CAL);
    read_member(r_Local_Pie_v, "O7" , r_Local.Pie.O7);

    read_member(r_Local_Pel_v, "LASIS", r_Local.Pel.LASIS);
    read_member(r_Local_Pel_v, "LPSIS", r_Local.Pel.LPSIS);
    read_member(r_Local_Pel_v, "RASIS", r_Local.Pel.RASIS);
    read_member(r_Local_Pel_v, "RPSIS", r_Local.Pel.RPSIS);

    read_member(R_local_v, "Femur_O3", R_local.Femur_O3);
    read_member(R_local_v, "Tibia_O5", R_local.Tibia_O5);
    read_member(R_local_v, "Pie_O7", R_local.Pie_O7);

    if (correct_reading==false){
        std::cout << "Something could not be read. Non correct readings:" << std::endl;
        for (int i=0; i<non_correct.size(); i++) std::cout <<  non_correct[i] << " "; 
        throw std::string("Calibration data reading not correct");
    }
}

// Prints all the data
void PRJsonCal::Calibration_data_struct::print_data(){
    std::cout << "-----------------------------------Calibration data --------------------------------------\n" << std::endl;
    Data_struct::print_data();
    std::cout << "ACSpelvis_r_PelAvg_ACSpelvis (transposed): " << ACSpelvis_r_PelAvg_ACSpelvis.transpose() << std::endl;
    std::cout << "DH_parameters:\n" << DH_parameters << std::endl;

    Plane4Bar.print_data();
    r_Local.print_data();
    R_local.print_data();
    std::cout << "-----------------------------------Calibration data --------------------------------------\n" << std::endl;
}