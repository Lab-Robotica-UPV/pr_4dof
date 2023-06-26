#ifndef PR_LIB_BIOMECH__DATA_STRUCTURES_
#define PR_LIB_BIOMECH__DATA_STRUCTURES_

#include "eigen3/Eigen/Dense"
#include <string>
#include <vector>

using namespace Eigen;

namespace PRDataStructures{

    struct G_r_G_struct{

        struct PatLig_struct{
            Vector3d Pat;
        }; 

        struct Ext_struct{
            PatLig_struct PatLig;
        }; 

        struct ViaPtsViaContours_struct{
            Vector3d CylCent;
            Vector3d CylDir;
        }; 
        
        //Marcadores Optitrack
        Vector3d LASIS;
        Vector3d RASIS;
        Vector3d LPSIS;
        Vector3d RPSIS;
        Vector3d LFE;
        Vector3d MFE;
        Vector3d LM;
        Vector3d MM;
        Vector3d FH;
        Vector3d CAL;
        Vector3d MH1;
        Vector3d MH5;
        Vector3d Fext;
        // Posiciones calcualdas
        Vector3d MidFE      ;
        Vector3d MidM       ;
        Vector3d MidDM      ;
        Vector3d MASIS      ;
        Vector3d MPSIS      ;
        Vector3d Pelvis     ;
        Vector3d Femur      ;
        Vector3d Tibia      ;
        Vector3d Pie        ;
        Vector3d IC         ;
        void print_data();
        
    };

    struct R_struct {
        // Matrices de rotacion segmentos
        Matrix3d Pelvis, Femur, Tibia, Pie, G_R_LMO4;
        // Matrices de rotacion intermedios
        Matrix3d O0_O3, Tibia_Pie, O3_O5, O5_O7;
        // Matrices de rotacion Rodilla
        Matrix3d u_R_LMO4;
        void print_data();
    };

    // Struct with relevant data to check after experiment
    struct Data_struct {

        struct input_struct {
            Matrix<double, 6, Dynamic> forces;
            Matrix<double, 3, Dynamic> LASIS;
            Matrix<double, 3, Dynamic> RASIS;
            Matrix<double, 3, Dynamic> LPSIS;
            Matrix<double, 3, Dynamic> RPSIS;
            Matrix<double, 3, Dynamic> RLE;
            Matrix<double, 3, Dynamic> RME;
            Matrix<double, 3, Dynamic> RHF;
            Matrix<double, 3, Dynamic> RLM;
            Matrix<double, 3, Dynamic> RMM;
            Matrix<double, 3, Dynamic> RCA;
            Matrix<double, 3, Dynamic> RFM;
            Matrix<double, 3, Dynamic> RVM;
            Matrix<double, 3, Dynamic> P_Movil_1;
            Matrix<double, 3, Dynamic> P_Movil_2;
            Matrix<double, 3, Dynamic> P_Movil_3;
            Matrix<double, 3, Dynamic> P_Fija_1;
            Matrix<double, 3, Dynamic> P_Fija_2;
            Matrix<double, 3, Dynamic> P_Fija_3;
            // Matrix<double, 3, Dynamic> robot_markers;
            // Matrix<double, 3, Dynamic> human_markers;
        };

        VectorXd Time;
        std::string Date;
        Matrix<double, 3, Dynamic> G_Fext;
        Matrix<double, 3, Dynamic> G_Mext;
        Matrix<double,73,Dynamic> MuscleForce;
        Matrix<double, 3, Dynamic> F_Knee;
        Matrix<double, 3, Dynamic> M_Knee;
        Matrix<double, 8, Dynamic> q;
        VectorXd PatForce, PosKnee;
        Matrix<double, 6, Dynamic> TauMus;
        Matrix<double, 4, Dynamic> muscle_dir;
        input_struct input;

        void print_data();
    };

    // Struct with data from human makers
    struct Piel_struct {
        Matrix<double, 3, Dynamic> LASIS;
        Matrix<double, 3, Dynamic> RASIS;
        Matrix<double, 3, Dynamic> LPSIS;
        Matrix<double, 3, Dynamic> RPSIS;
        Matrix<double, 3, Dynamic> LFE;
        Matrix<double, 3, Dynamic> MFE;
        Matrix<double, 3, Dynamic> FH;
        Matrix<double, 3, Dynamic> LM;
        Matrix<double, 3, Dynamic> MM;
        Matrix<double, 3, Dynamic> CAL;
        Matrix<double, 3, Dynamic> MH1;
        Matrix<double, 3, Dynamic> MH5;
        Matrix<double, 3, Dynamic> Fext;

        void print_data();
    };

    struct forLigForceCal_struct {
        bool FlxActiveOrg;
        std::vector<std::string> FlxLegend;
        Matrix<double, 8, 1> FlxMusForceEstOpt;
        Matrix<double, 8, 1> FlxMaxMusForce;
        std::vector<std::string> ExtLegend;
        Matrix<double, 10, 1> ExtMusForceEstOpt;
        Matrix<double, 10, 1> ExtMaxMusForce;
        double PatLig, RecFem12_1, RecFem12_2, TenFacLat;

        void print_data();
    };

    struct ang_struct {
        struct cad_struct {
            double FlxExt, AbdAduc, IntExt;
        };
        struct rod_struct {
            double FlxExt;
        };
        struct pie_struct {
            double FlxExt;
        };

        cad_struct cad;
        rod_struct rod;
        pie_struct pie;
        
        void print_data();
    };



    // AUXILIARY Struct with input data from force sensor (remove in ROS)
    struct AUXILIAR_ForceSensorin_struct {
        Matrix<double, 3, Dynamic> Force;
        Matrix<double, 3, Dynamic> Torque;

        void print_data();
    };

}

#endif // PR_LIB_BIOMECH__DATA_STRUCTURES_