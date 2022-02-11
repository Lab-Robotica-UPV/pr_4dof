#include "pr_lib_biomech/pr_json_calibration.hpp"

using namespace PRJsonData;

void PRJsonCal::O6_r_O6_struct::print_data(){
    std::cout << "--------O6_r_O6 struct---------" << std::endl;
    std::cout << "LM2 (transposed): " << LM2.transpose() << std::endl;
    std::cout << "MM2 (transposed): " << MM2.transpose() << std::endl;
    std::cout << "SCS_Tib (transposed): " << SCS_Tib.transpose() << std::endl;
    std::cout << "--------O6_r_O6 struct end ----\n" << std::endl;
}  


void PRJsonCal::Plane4Bar_struct::print_data(){
    std::cout << "--------Plane4Bar struct---------" << std::endl;
    std::cout << "LMO4_u (transposed): " << LMO4_u.transpose() << std::endl;
    std::cout << "L (transpose): " << L.transpose() << std::endl;
    std::cout << "A (transpose): " << A.transpose() << std::endl;
    std::cout << "theta0: " << theta0 << std::endl;
    std::cout << "DELTA: " << DELTA << std::endl;
    std::cout << "u_R_LMO4:\n" << u_R_LMO4 << std::endl;
    std::cout << "--------Plane4Bar struct end ----\n" << std::endl;
}


void PRJsonCal::Widths_struct::print_data(){
    std::cout << "--------Widths struct---------" << std::endl;
    std::cout << "Fem: " << Fem << std::endl;
    std::cout << "Tib: " << Tib << std::endl;
    std::cout << "--------Widths struct end ----\n" << std::endl;
}

// Constructor. Receives a path and extracts the JSON content
PRJsonCal::Calibration_data_struct::Calibration_data_struct(std::string path)
    : Data_struct(path){

    // All structures inside the original are extracted into Values of rapidjson library
    const Value &O6_r_O6_v = struct_value(document,"O6_r_O6");
    const Value &Plane4Bar_v = struct_value(document,"Plane4Bar");
    const Value &Widths_v = struct_value(document,"Widths");

    // all members are read from the original document or from the substructures
    read_member(document, "ACSpelvis_r_PelAvg_ACSpelvis", ACSpelvis_r_PelAvg_ACSpelvis);
    read_member(document,"O6_R_SCS_Tib",O6_R_SCS_Tib);
    read_member(O6_r_O6_v,"LM2",O6_r_O6.LM2);
    read_member(O6_r_O6_v,"MM2",O6_r_O6.MM2);
    read_member(O6_r_O6_v,"SCS_Tib",O6_r_O6.SCS_Tib);
    read_member(Plane4Bar_v,"LMO4_u",Plane4Bar.LMO4_u);
    read_member(Plane4Bar_v,"L",Plane4Bar.L);
    read_member(Plane4Bar_v,"A",Plane4Bar.A);
    read_member(Plane4Bar_v,"theta0",Plane4Bar.theta0);
    read_member(Plane4Bar_v,"DELTA",Plane4Bar.DELTA);
    read_member(Plane4Bar_v,"u_R_LMO4",Plane4Bar.u_R_LMO4);
    read_member(document,"SCS_Tib_CPL_Tib",SCS_Tib_CPL_Tib);
    read_member(document,"SCS_Tib_CPM_Tib",SCS_Tib_CPM_Tib);
    read_member(document,"SCS_Tib_rg",SCS_Tib_rg);
    read_member(document,"SCSpie_r_SCSpie_Gpie",SCSpie_r_SCSpie_Gpie);
    read_member(Widths_v,"Fem",Widths.Fem);
    read_member(Widths_v,"Tib",Widths.Tib);
    read_member(document,"ang_cad_AbdAduc_i",ang_cad_AbdAduc_i);
    read_member(document,"ang_cad_FlxExt_i",ang_cad_FlxExt_i);
    read_member(document,"ang_pie_FlxExt_i",ang_pie_FlxExt_i);
    read_member(document,"ang_rod_FlxExt_i",ang_rod_FlxExt_i);
    read_member(document,"m",m);
    read_member(document,"m_pie",m_pie);
    read_member(document,"rg",rg);
    read_member(document, "G_g", G_g);

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
    std::cout << "O6_R_SCS_Tib:\n" << O6_R_SCS_Tib << std::endl;
    O6_r_O6.print_data();
    Plane4Bar.print_data();
    std::cout << "SCS_Tib_CPL_Tib (transpose): " << SCS_Tib_CPL_Tib.transpose() << std::endl;
    std::cout << "SCS_Tib_CPM_Tib (transpose):  " << SCS_Tib_CPM_Tib.transpose()  << std::endl;
    std::cout << "SCS_Tib_rg (transpose): " << SCS_Tib_rg.transpose() << std::endl;
    std::cout << "SCSpie_r_SCSpie_Gpie (transpose): " << SCSpie_r_SCSpie_Gpie.transpose() << std::endl;
    Widths.print_data();
    std::cout << "ang_cad_AbdAduc_i : " << ang_cad_AbdAduc_i  << std::endl;
    std::cout << "ang_cad_FlxExt_i: " << ang_cad_FlxExt_i << std::endl;
    std::cout << "ang_pie_FlxExt_i: " << ang_pie_FlxExt_i << std::endl;
    std::cout << "ang_rod_FlxExt_i: " << ang_rod_FlxExt_i << std::endl;
    std::cout << "m (transpose): " << m.transpose() << std::endl;
    std::cout << "rg:\n" << rg << std::endl;
    std::cout << "G_g (transpose): " << G_g << std::endl;
    std::cout << "-----------------------------------Calibration data --------------------------------------\n" << std::endl;
}