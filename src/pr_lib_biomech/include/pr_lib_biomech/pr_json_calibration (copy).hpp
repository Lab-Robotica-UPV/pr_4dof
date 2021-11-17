#ifndef PR_LIB__JSON_CALIBARTION_
#define PR_LIB__JSON_CALIBRATION_

#include "document.h"     // rapidjson's DOM-style API
#include <cstdio>
#include <iostream>
#include <vector>
#include "eigen3/Eigen/Dense"
#include <fstream>

using namespace rapidjson;

namespace PRJsonCalibration{

    struct Calibration_data_struct;
    struct Lengths_struct;
    struct Lig_struct;
    struct TibColLig_struct;
    struct FibColLig_struct;
    struct O6_r_O6_struct;
    struct P1_r_P1_struct;
    struct Plane4Bar_struct;
    struct ViaPtsViaContours_struct;
    struct Gra_struct;
    struct Sat_struct;
    struct SemTend_struct;
    struct Cyl_struct;
    struct Widths_struct;



    struct Lengths_struct{
        double Fem, Tib, Pie;
        void print_data();
    };

    struct TibColLig_struct{
        Eigen::Vector3d SCS_Tib;
    };

    struct FibColLig_struct{
        Eigen::Vector3d SCS_Tib;
    };

    struct Lig_struct{
        TibColLig_struct TibColLig;
        FibColLig_struct FibColLig;
        void print_data();
    };

    struct O6_r_O6_struct{
        Eigen::Vector3d LMavg;
        Eigen::Vector3d MMavg;
        Eigen::Vector3d FHavg;
        Eigen::Vector3d LM2;
        Eigen::Vector3d MM2;
        Eigen::Vector3d FH2;
        Eigen::Vector3d AJC2;
        Eigen::Vector3d O5;
        Eigen::Vector3d SCS_Tib;
        Eigen::Vector3d CAL2;
        Eigen::Vector3d MH1_2;
        Eigen::Vector3d MH5_2;
        void print_data(); 
    };

    struct P1_r_P1_struct{
        Eigen::Vector3d LMavg;
        Eigen::Vector3d MMavg;
        Eigen::Vector3d FHavg;
        void print_data();
    };

    struct Plane4Bar_struct{
        Eigen::MatrixXd LMO4_W_Fem_Tib;
        Eigen::MatrixXd LMO4_W_Fem_Tib_u;
        Eigen::Vector3d LMO4_u;
        Eigen::VectorXi index;
        Eigen::VectorXi indexlogical;
        Eigen::Matrix<double,6, 1> L;
        Eigen::Vector2d A;
        Eigen::Vector2d B;
        double theta0;
        Eigen::VectorXd theta2;
        Eigen::VectorXd theta3;
        double DELTA;
        Eigen::Matrix3d u_R_LMO4;

        void print_data();
    };

    struct Gra_struct{
        Eigen::Matrix<double,3,121> SCS_Tib;
    };

    struct Sat_struct{
        Eigen::Matrix<double,3,121> LMO4;
        Eigen::Vector3d ACSpelvis;
    };

    struct SemTend_struct{
        Eigen::Matrix<double,3,121> SCS_Tib;
    };

    struct Cyl_struct{
        Eigen::Vector3d CylCentLMO4;
        Eigen::Vector3d CylDirLMO4;
        double CylRad;
    };

    struct ViaPtsViaContours_struct{
        Gra_struct Gra;
        Sat_struct Sat;
        SemTend_struct SemTend;
        Cyl_struct Cyl;
        void print_data();
    };

    struct Widths_struct{
        double Fem;
        double Tib;
        void print_data();
    };

    struct Calibration_data_struct{

        Eigen::Vector3d ACSpelvis_r_PelAvg_ACSpelvis;
        Lengths_struct Lengths;
        Lig_struct Lig;
        double Mass;
        Eigen::Matrix3d O6_R_SCS_Tib;
        O6_r_O6_struct O6_r_O6;
        P1_r_P1_struct P1_r_P1;
        double PatMovRad;
        Plane4Bar_struct Plane4Bar;
        Eigen::Vector3d SCS_Fem_rg;
        Eigen::Vector3d SCS_Tib_CPL_Tib;
        Eigen::Vector3d SCS_Tib_CPM_Tib;
        Eigen::Vector3d SCS_Tib_FibColLigTib;
        Eigen::Vector3d SCS_Tib_TibColLigTib;
        Eigen::Vector3d SCS_Tib_rg;
        Eigen::Matrix3d SCSpie_Ig;
        Eigen::Vector3d SCSpie_r_SCSpie_Gpie;
        ViaPtsViaContours_struct ViaPtsViaContours;
        Widths_struct Widths;
        Eigen::Matrix<double,6,1> a;
        double a4;
        Eigen::Matrix<double,5,1> alpha;
        double ang_cad_AbdAduc_i;
        double ang_cad_FlxExt_i;
        double ang_pie_FlxExt_i;
        double ang_rod_FlxExt_i;
        Eigen::Matrix<double,6,1> d;
        Eigen::Matrix<double,6,1> m;
        double m_pie;
        Eigen::Matrix<double,3,6> rg;

        bool correct_reading = true;
        std::vector<std::string> non_correct;

        // Constructor. Receives a path and extracts the JSON content
        Calibration_data_struct(std::string path);

        // Prints all the data
        void print_data();

        // Generic function that reads from a rapidjson document or value (substructure) and a generic var
        template<typename D, typename T>
        void read_member(const D &doc, const std::string member, T &var);

        // Function overloading if the var is int
        template<typename D>
        void assign_value(const D &doc, const Value::ConstMemberIterator &itr, int &var);

        // Function overloading if the var is double
        template<typename D>
        void assign_value(const D &doc, const Value::ConstMemberIterator &itr, double &var);

        template<typename D>
        void assign_value(const D &doc, const Value::ConstMemberIterator &itr, std::string &var);

        // Function overloading if the var is a Eigen Matrix
        template<typename D, typename Derived>
        void assign_value(const D &doc, const Value::ConstMemberIterator &itr, Eigen::MatrixBase<Derived> &var);

        // Function overload if the Eigen matrix is of integers
        void assign_value(int &var, const Value &val);

        // Function overload if the Eigen matrix is of doubles
        void assign_value(double &var, const Value &val);

        // Extracts substructures from the document (or another substructure) and places in a Value variable
        template<typename D>
        const Value& struct_value(const D &doc, const std::string member);

    };

}

#endif // PR_LIB__JSON_CALIBRATION_