#include "pr_lib_biomech/pr_json_calibration.hpp"


void PRJsonCalibration::Lengths_struct::print_data(){
    std::cout << "--------Lengths struct---------" << std::endl;
    std::cout << "Fem: " << Fem << std::endl;
    std::cout << "Tib: " << Tib << std::endl;
    std::cout << "Pie: " << Pie << std::endl;
    std::cout << "--------Lengths struct end ----\n" << std::endl;
}

void PRJsonCalibration::Lig_struct::print_data(){
    std::cout << "--------Lig struct---------" << std::endl;
    std::cout << "TibColLig.SCS_Tib (transposed): " << TibColLig.SCS_Tib.transpose() << std::endl;
    std::cout << "FibColLig.SCS_Tib (transposed): " << FibColLig.SCS_Tib.transpose() << std::endl;
    std::cout << "--------Lig struct end ----\n" << std::endl;
}

void PRJsonCalibration::O6_r_O6_struct::print_data(){
    std::cout << "--------O6_r_O6 struct---------" << std::endl;
    std::cout << "LMavg (transposed): " << LMavg.transpose() << std::endl;
    std::cout << "MMavg (transposed): " << MMavg.transpose() << std::endl;
    std::cout << "FHavg (transposed): " << FHavg.transpose() << std::endl;
    std::cout << "LM2 (transposed): " << LM2.transpose() << std::endl;
    std::cout << "MM2 (transposed): " << MM2.transpose() << std::endl;
    std::cout << "FH2 (transposed): " << FH2.transpose() << std::endl;
    std::cout << "AJC2 (transposed): " << AJC2.transpose() << std::endl;
    std::cout << "O5 (transposed): " << O5.transpose() << std::endl;
    std::cout << "SCS_Tib (transposed): " << SCS_Tib.transpose() << std::endl;
    std::cout << "CAL2 (transposed): " << CAL2.transpose() << std::endl;
    std::cout << "MH1_2 (transposed): " << MH1_2.transpose() << std::endl;
    std::cout << "MH5_2 (transposed): " << MH5_2.transpose() << std::endl;
    std::cout << "--------O6_r_O6 struct end ----\n" << std::endl;
}  


void PRJsonCalibration::P1_r_P1_struct::print_data(){
    std::cout << "--------P1_r_P1 struct---------" << std::endl;
    std::cout << "LMavg (transposed): " << LMavg.transpose() << std::endl;
    std::cout << "MMavg (transposed): " << MMavg.transpose() << std::endl;
    std::cout << "FHavg (transposed): " << FHavg.transpose() << std::endl;
    std::cout << "--------P1_r_P1 struct end ----\n" << std::endl;  
}

void PRJsonCalibration::Plane4Bar_struct::print_data(){
    std::cout << "--------Plane4Bar struct---------" << std::endl;
    std::cout << "LMO4_W_Fem_Tib (1st col transposed): " << LMO4_W_Fem_Tib.col(0).transpose() << std::endl;
    std::cout << "LMO4_W_Fem_Tib_u (1st col transposed): " << LMO4_W_Fem_Tib_u.col(0).transpose() << std::endl;
    std::cout << "LMO4_u (transposed): " << LMO4_u.transpose() << std::endl;
    std::cout << "index (1st elem): " << index(0) << std::endl;
    std::cout << "indexlogical (1st elem): " << indexlogical(0) << std::endl;
    std::cout << "L (transpose): " << L.transpose() << std::endl;
    std::cout << "A (transpose): " << A.transpose() << std::endl;
    std::cout << "B (transpose): " << B.transpose() << std::endl;
    std::cout << "theta0: " << theta0 << std::endl;
    std::cout << "theta2 (1st elem): " << theta2(0) << std::endl;
    std::cout << "theta3 (1st elem): " << theta3(0) << std::endl;
    std::cout << "DELTA: " << DELTA << std::endl;
    std::cout << "u_R_LMO4:\n" << u_R_LMO4 << std::endl;
    std::cout << "--------Plane4Bar struct end ----\n" << std::endl;
}


void  PRJsonCalibration::ViaPtsViaContours_struct::print_data(){
    std::cout << "--------ViaPtsViaContours struct---------" << std::endl;
    std::cout << "Gra.SCS_Tib (1st col transposed): " << Gra.SCS_Tib.col(0).transpose() << std::endl;
    std::cout << "Sat.LMO4 (1st col transposed): " << Sat.LMO4.col(0).transpose() << std::endl;
    std::cout << "Sat.ACSpelvis (transposed): " << Sat.ACSpelvis.transpose() << std::endl;
    std::cout << "SemTend.SCS_Tib (1st col transposed): " << SemTend.SCS_Tib.col(0).transpose() << std::endl;
    std::cout << "Cyl.CylCentLMO4 (transposed): " << Cyl.CylCentLMO4.transpose() << std::endl;
    std::cout << "Cyl.CylDirLMO4 (transposed): " << Cyl.CylDirLMO4.transpose() << std::endl;
    std::cout << "Cyl.CylRad: " << Cyl.CylRad << std::endl;
    std::cout << "--------ViaPtsViaContours struct end ----\n" << std::endl; 
}

void PRJsonCalibration::Widths_struct::print_data(){
    std::cout << "--------Widths struct---------" << std::endl;
    std::cout << "Fem: " << Fem << std::endl;
    std::cout << "Tib: " << Tib << std::endl;
    std::cout << "--------Widths struct end ----\n" << std::endl;
}

// Constructor. Receives a path and extracts the JSON content
PRJsonCalibration::Calibration_data_struct::Calibration_data_struct(std::string path){
    // Complete JSON string
    std::string json_string;
    std::ifstream file(path);
    if (file.is_open()){
        // JSON is encoded in just one line
        getline(file, json_string);
        //std::cout << json_string << std::endl;
        file.close();
        // JSON document from rapidjson
        Document document;
        // document parses all JSON content
        if (document.Parse(json_string.c_str()).HasParseError()){
            std::cout << "Error while parsing JSON string" << std::endl;
            correct_reading = false;
        }
        else{
            // All structures inside the original are extracted into Values of rapidjson library
            const Value &Lengths_v = struct_value(document, "Lengths");
            const Value &Lig_v = struct_value(document,"Lig");
            const Value &TibColLig_v = struct_value(Lig_v,"TibColLig");
            const Value &FibColLig_v = struct_value(Lig_v,"FibColLig");
            const Value &O6_r_O6_v = struct_value(document,"O6_r_O6");
            const Value &P1_r_P1_v = struct_value(document,"P1_r_P1");
            const Value &Plane4Bar_v = struct_value(document,"Plane4Bar");
            const Value &ViaPtsViaContours_v = struct_value(document,"ViaPtsViaContours");
            const Value &Gra_v = struct_value(ViaPtsViaContours_v,"Gra");
            const Value &Sat_v = struct_value(ViaPtsViaContours_v,"Sat");
            const Value &SemTend_v = struct_value(ViaPtsViaContours_v,"SemTend");
            const Value &Cyl_v = struct_value(ViaPtsViaContours_v,"Cyl");
            const Value &Widths_v = struct_value(document,"Widths");

            // all members are read from the original document or from the substructures
            read_member(document, "ACSpelvis_r_PelAvg_ACSpelvis", ACSpelvis_r_PelAvg_ACSpelvis);
            read_member(Lengths_v,"Fem",Lengths.Fem);
            read_member(Lengths_v,"Tib",Lengths.Tib);
            read_member(Lengths_v,"Pie",Lengths.Pie);
            read_member(TibColLig_v, "SCS_Tib",Lig.TibColLig.SCS_Tib);
            read_member(FibColLig_v, "SCS_Tib",Lig.FibColLig.SCS_Tib);
            read_member(document,"Mass",Mass);
            read_member(document,"O6_R_SCS_Tib",O6_R_SCS_Tib);
            read_member(O6_r_O6_v,"LMavg",O6_r_O6.LMavg);
            read_member(O6_r_O6_v,"MMavg",O6_r_O6.MMavg);
            read_member(O6_r_O6_v,"FHavg",O6_r_O6.FHavg);
            read_member(O6_r_O6_v,"LM2",O6_r_O6.LM2);
            read_member(O6_r_O6_v,"MM2",O6_r_O6.MM2);
            read_member(O6_r_O6_v,"FH2",O6_r_O6.FH2);
            read_member(O6_r_O6_v,"AJC2",O6_r_O6.AJC2);
            read_member(O6_r_O6_v,"O5",O6_r_O6.O5);
            read_member(O6_r_O6_v,"SCS_Tib",O6_r_O6.SCS_Tib);
            read_member(O6_r_O6_v,"CAL2",O6_r_O6.CAL2);
            read_member(O6_r_O6_v,"MH1_2",O6_r_O6.MH1_2);
            read_member(O6_r_O6_v,"MH5_2",O6_r_O6.MH5_2);
            read_member(P1_r_P1_v,"LMavg",P1_r_P1.LMavg);
            read_member(P1_r_P1_v,"MMavg",P1_r_P1.MMavg);
            read_member(P1_r_P1_v,"FHavg",P1_r_P1.FHavg);
            read_member(document,"PatMovRad",PatMovRad);
            read_member(Plane4Bar_v,"LMO4_W_Fem_Tib",Plane4Bar.LMO4_W_Fem_Tib);
            read_member(Plane4Bar_v,"LMO4_W_Fem_Tib_u",Plane4Bar.LMO4_W_Fem_Tib_u);
            read_member(Plane4Bar_v,"LMO4_u",Plane4Bar.LMO4_u);
            read_member(Plane4Bar_v,"index",Plane4Bar.index);
            read_member(Plane4Bar_v,"indexlogical",Plane4Bar.indexlogical);
            read_member(Plane4Bar_v,"L",Plane4Bar.L);
            read_member(Plane4Bar_v,"A",Plane4Bar.A);
            read_member(Plane4Bar_v,"B",Plane4Bar.B);
            read_member(Plane4Bar_v,"theta0",Plane4Bar.theta0);
            read_member(Plane4Bar_v,"theta2",Plane4Bar.theta2);
            read_member(Plane4Bar_v,"theta3",Plane4Bar.theta3);
            read_member(Plane4Bar_v,"DELTA",Plane4Bar.DELTA);
            read_member(Plane4Bar_v,"u_R_LMO4",Plane4Bar.u_R_LMO4);
            read_member(document,"SCS_Fem_rg",SCS_Fem_rg);
            read_member(document,"SCS_Tib_CPL_Tib",SCS_Tib_CPL_Tib);
            read_member(document,"SCS_Tib_CPM_Tib",SCS_Tib_CPM_Tib);
            read_member(document,"SCS_Tib_FibColLigTib",SCS_Tib_FibColLigTib);
            read_member(document,"SCS_Tib_TibColLigTib",SCS_Tib_TibColLigTib);
            read_member(document,"SCS_Tib_rg",SCS_Tib_rg);
            read_member(document,"SCSpie_Ig",SCSpie_Ig);
            read_member(document,"SCSpie_r_SCSpie_Gpie",SCSpie_r_SCSpie_Gpie);
            read_member(Gra_v,"SCS_Tib",ViaPtsViaContours.Gra.SCS_Tib);
            read_member(Sat_v,"LMO4",ViaPtsViaContours.Sat.LMO4);
            read_member(Sat_v,"ACSpelvis",ViaPtsViaContours.Sat.ACSpelvis);
            read_member(SemTend_v,"SCS_Tib",ViaPtsViaContours.SemTend.SCS_Tib);
            read_member(Cyl_v,"CylCentLMO4",ViaPtsViaContours.Cyl.CylCentLMO4);
            read_member(Cyl_v,"CylDirLMO4",ViaPtsViaContours.Cyl.CylDirLMO4);
            read_member(Cyl_v,"CylRad",ViaPtsViaContours.Cyl.CylRad);
            read_member(Widths_v,"Fem",Widths.Fem);
            read_member(Widths_v,"Tib",Widths.Tib);
            read_member(document,"a",a);
            read_member(document,"a4",a4);
            read_member(document,"alpha",alpha);
            read_member(document,"ang_cad_AbdAduc_i",ang_cad_AbdAduc_i);
            read_member(document,"ang_cad_FlxExt_i",ang_cad_FlxExt_i);
            read_member(document,"ang_pie_FlxExt_i",ang_pie_FlxExt_i);
            read_member(document,"ang_rod_FlxExt_i",ang_rod_FlxExt_i);
            read_member(document,"d",d);
            read_member(document,"m",m);
            read_member(document,"m_pie",m_pie);
            read_member(document,"rg",rg);

        }
    }
    else{
        std::cout << "Calibration data file not found" << std::endl;
        correct_reading = false;
    }
    if (correct_reading==false){
        std::cout << "Something could not be read. Non correct readings:" << std::endl;
        for (int i=0; i<non_correct.size(); i++) std::cout <<  non_correct[i] << " "; 
        throw std::string("Calibration data reading not correct");
    }
}

// Prints all the data
void PRJsonCalibration::Calibration_data_struct::print_data(){
    std::cout << "-----------------------------------Calibration data --------------------------------------\n" << std::endl;
    std::cout << "correct_reading: " << correct_reading << std::endl;
    std::cout << "non correct: "; 
    for (int i=0; i<non_correct.size(); i++) std::cout <<  non_correct[i] << " "; 
    std::cout << "\n";
    std::cout << "ACSpelvis_r_PelAvg_ACSpelvis (transposed): " << ACSpelvis_r_PelAvg_ACSpelvis.transpose() << std::endl;
    Lengths.print_data();
    Lig.print_data();
    std::cout << "Mass: " << Mass << std::endl;
    std::cout << "O6_R_SCS_Tib:\n" << O6_R_SCS_Tib << std::endl;
    O6_r_O6.print_data();
    P1_r_P1.print_data();
    std::cout << "PatMovRad: " << PatMovRad << std::endl;
    Plane4Bar.print_data();
    std::cout << "SCS_Fem_rg (transpose): " << SCS_Fem_rg.transpose() << std::endl;
    std::cout << "SCS_Tib_CPL_Tib (transpose): " << SCS_Tib_CPL_Tib.transpose() << std::endl;
    std::cout << "SCS_Tib_CPM_Tib (transpose):  " << SCS_Tib_CPM_Tib.transpose()  << std::endl;
    std::cout << "SCS_Tib_FibColLigTib (transpose): " << SCS_Tib_FibColLigTib.transpose() << std::endl;
    std::cout << "SCS_Tib_TibColLigTib (transpose): " << SCS_Tib_TibColLigTib.transpose() << std::endl;
    std::cout << "SCS_Tib_rg (transpose): " << SCS_Tib_rg.transpose() << std::endl;
    std::cout << "SCSpie_Ig:\n" << SCSpie_Ig << std::endl;
    std::cout << "SCSpie_r_SCSpie_Gpie (transpose): " << SCSpie_r_SCSpie_Gpie.transpose() << std::endl;
    ViaPtsViaContours.print_data();
    Widths.print_data();
    std::cout << "a (transposed): " << a.transpose() << std::endl;
    std::cout << "a4: " << a4 << std::endl;
    std::cout << "alpha (transposed): " << alpha.transpose() << std::endl;
    std::cout << "ang_cad_AbdAduc_i : " << ang_cad_AbdAduc_i  << std::endl;
    std::cout << "ang_cad_FlxExt_i: " << ang_cad_FlxExt_i << std::endl;
    std::cout << "ang_pie_FlxExt_i: " << ang_pie_FlxExt_i << std::endl;
    std::cout << "ang_rod_FlxExt_i: " << ang_rod_FlxExt_i << std::endl;
    std::cout << "d (transpose): " << d.transpose() << std::endl;
    std::cout << "m (transpose): " << m.transpose() << std::endl;
    std::cout << "rg:\n" << rg << std::endl;
    std::cout << "-----------------------------------Calibration data --------------------------------------\n" << std::endl;
}

// Generic function that reads from a rapidjson document or value (substructure) and a generic var
template<typename D, typename T>
void PRJsonCalibration::Calibration_data_struct::read_member(const D &doc, const std::string member, T &var){
    Value::ConstMemberIterator itr = doc.FindMember(member.c_str());
    if (itr != doc.MemberEnd()){ 
        assign_value(doc, itr, var);
    }
    else{ 
        correct_reading = false;
        non_correct.push_back(member);
    }    
}

// Function overloading if the var is int
template<typename D>
void PRJsonCalibration::Calibration_data_struct::assign_value(const D &doc, const Value::ConstMemberIterator &itr, int &var){
    var = itr->value.GetInt();
}

// Function overloading if the var is double
template<typename D>
void PRJsonCalibration::Calibration_data_struct::assign_value(const D &doc, const Value::ConstMemberIterator &itr, double &var){
    var = itr->value.GetDouble();
}

template<typename D>
void PRJsonCalibration::Calibration_data_struct::assign_value(const D &doc, const Value::ConstMemberIterator &itr, std::string &var){
    var = itr->value.GetString();
}

// Function overloading if the var is a Eigen Matrix
template<typename D, typename Derived>
void PRJsonCalibration::Calibration_data_struct::assign_value(const D &doc, const Value::ConstMemberIterator &itr, Eigen::MatrixBase<Derived> &var){
    const Value &vec = itr->value.GetArray();
    int dim1 = vec.Size();
    if (vec[0].IsArray()){
        int dim2 = vec[0].GetArray().Size();
        if (var.rows()==0 && var.cols()==0){
            var.derived().resize(dim1,dim2);
        }
        for (int i=0; i<dim1; i++){
            for (int j=0; j<dim2; j++){
                assign_value(var(i,j), vec[i].GetArray()[j]);
            }
        }
    }
    else{
        if (var.size() == 0){ var.derived().resize(dim1,1);//resize_matrix(var,dim1);
        }
        for (int i=0; i<dim1; i++){
            assign_value(var(i), vec[i]);
        }  
    }

}

// Function overload if the Eigen matrix is of integers
void PRJsonCalibration::Calibration_data_struct::assign_value(int &var, const Value &val){
    var = val.GetInt();
}

// Function overload if the Eigen matrix is of doubles
void PRJsonCalibration::Calibration_data_struct::assign_value(double &var, const Value &val){
    var = val.GetDouble();
}

// Extracts substructures from the document (or another substructure) and places in a Value variable
template<typename D>
const Value& PRJsonCalibration::Calibration_data_struct::struct_value(const D &doc, const std::string member){ 
    Value::ConstMemberIterator itr = doc.FindMember(member.c_str());
    if (itr != doc.MemberEnd()){ 
        const Value &val = doc[member.c_str()];
        return val;
    }
    else{ 
        correct_reading = false;
    }
}