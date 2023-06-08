#ifndef PR_LIB__JSON_CALIBARTION_
#define PR_LIB__JSON_CALIBRATION_

#include "pr_json_data.hpp"     // Inherits pr_json_data_struct

using namespace rapidjson;

namespace PRJsonData{

    namespace PRJsonCal{

        struct Plane4Bar;
        struct r_Local;
        struct R_local;

        struct Plane4Bar_struct {
            Eigen::Vector3d LMO4_u;
            Eigen::Matrix <double,6,1> L;
            double DELTA;
            Eigen::Matrix3d u_R_LMO4;
            double ThetaL1X3;
            double ThetaL3X5;
            void print_data(); 
        };

        struct r_Local_struct {
            struct Fem_struct {
                Eigen::Vector3d CoM;
                Eigen::Vector3d LFE;
                Eigen::Vector3d MFE;
                Eigen::Vector3d O3;
            };
            struct Tib_struct {
                Eigen::Vector3d CoM;
                Eigen::Vector3d LM;
                Eigen::Vector3d MM;
                Eigen::Vector3d FH;
                Eigen::Vector3d O5;
            };
            struct Pie_struct {
                Eigen::Vector3d CoM;
                Eigen::Vector3d MH1;
                Eigen::Vector3d MH5;
                Eigen::Vector3d CAL;
                Eigen::Vector3d O7;
            };
            struct Pel_struct {
                Eigen::Vector3d LASIS;
                Eigen::Vector3d LPSIS;
                Eigen::Vector3d RASIS;
                Eigen::Vector3d RPSIS;
            };

            Fem_struct Fem;
            Tib_struct Tib;
            Pie_struct Pie;
            Pel_struct Pel;            

            void print_data();
        };

        struct R_local_struct{
            Eigen::Matrix3d Femur_O3;
            Eigen::Matrix3d Tibia_O5;
            Eigen::Matrix3d Pie_O7;
            void print_data();
        };

        struct Calibration_data_struct : public Data_struct{

            Eigen::Vector3d ACSpelvis_r_PelAvg_ACSpelvis;
            Plane4Bar_struct Plane4Bar;
            r_Local_struct r_Local;
            R_local_struct R_local;
            Eigen::Matrix <double, 8, 4> DH_parameters;

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