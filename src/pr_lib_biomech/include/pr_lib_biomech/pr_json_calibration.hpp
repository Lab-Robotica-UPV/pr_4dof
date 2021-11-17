#ifndef PR_LIB__JSON_CALIBARTION_
#define PR_LIB__JSON_CALIBRATION_

#include "pr_json_data.hpp"     // Inherits pr_json_data_struct

using namespace rapidjson;

namespace PRJsonData{

    namespace PRJsonCal{

        struct Calibration_data_struct;
        struct Lig_struct;
        struct TibColLig_struct;
        struct FibColLig_struct;
        struct O6_r_O6_struct;
        struct Plane4Bar_struct;
        struct Widths_struct;



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
            Eigen::Vector3d LM2;
            Eigen::Vector3d MM2;
            Eigen::Vector3d SCS_Tib;
            void print_data(); 
        };


        struct Plane4Bar_struct{
            Eigen::Vector3d LMO4_u;
            Eigen::Matrix<double,6, 1> L;
            Eigen::Vector2d A;
            double theta0;
            double DELTA;
            Eigen::Matrix3d u_R_LMO4;
            void print_data();
        };


        struct Widths_struct{
            double Fem;
            double Tib;
            void print_data();
        };

        struct Calibration_data_struct : public Data_struct{

            Eigen::Vector3d ACSpelvis_r_PelAvg_ACSpelvis;
            Lig_struct Lig;
            Eigen::Matrix3d O6_R_SCS_Tib;
            O6_r_O6_struct O6_r_O6;
            Plane4Bar_struct Plane4Bar;
            Eigen::Vector3d SCS_Tib_CPL_Tib;
            Eigen::Vector3d SCS_Tib_CPM_Tib;
            Eigen::Vector3d SCS_Tib_rg;
            Eigen::Vector3d SCSpie_r_SCSpie_Gpie;
            Widths_struct Widths;
            double ang_cad_AbdAduc_i;
            double ang_cad_FlxExt_i;
            double ang_pie_FlxExt_i;
            double ang_rod_FlxExt_i;
            Eigen::Matrix<double,6,1> m;
            double m_pie;
            Eigen::Matrix<double,3,6> rg;

            bool correct_reading = true;
            std::vector<std::string> non_correct;

            // Constructor. Receives a path and extracts the JSON content
            Calibration_data_struct(std::string path);

            // Print data
            void print_data();

        };

    }

}

#endif // PR_LIB__JSON_CALIBRATION_