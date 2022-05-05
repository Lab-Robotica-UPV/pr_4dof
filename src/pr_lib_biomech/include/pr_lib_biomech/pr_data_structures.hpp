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

        Vector3d ACSpelvis;
        Vector3d AJC;
        Vector3d AJC2;
        Vector3d CAL;
        Vector3d CPL;
        Vector3d CPM;
        Vector3d Cal2;
        Ext_struct Ext;
        Vector3d FH;
        Vector3d FH2;
        Vector3d Fem_Cent;
        Vector3d Fext;
        Vector3d Fext_r;
        Vector3d Fext_u;
        Vector3d FibColLigTib;
        Vector3d FibColLigTib_un;
        Vector3d Gpie;
        Vector3d Gpie_r;
        Vector3d HJC;
        Vector3d IC;
        Vector3d KJC;
        Vector3d LASIS;
        Vector3d LFE;
        Vector3d LM;
        Vector3d LM2;
        Vector3d LMO4;
        Vector3d LPSIS;
        Vector3d MASIS;
        Vector3d MFE;
        Vector3d MH1;
        Vector3d MH1_2;
        Vector3d MH5;
        Vector3d MH5_2;
        Vector3d MHmid;
        Vector3d MM;
        Vector3d MM2;
        Vector3d MPSIS;
        Vector3d O1;
        Vector3d O4;
        Vector3d O5;
        Vector3d P1;
        Vector3d P2;
        Vector3d P3;
        Vector3d P4;
        Vector3d Pie_Cent;
        Vector3d RASIS;
        Vector3d RPSIS;
        Vector3d SCS_Fem;
        Vector3d SCS_Tib;
        Vector3d SCSpie;
        Vector3d TibColLigTib;
        Vector3d TibColLigTib_un;
        Vector3d Tib_Cent;
        ViaPtsViaContours_struct ViaPtsViaContours;
        Vector3d g4;
        Vector3d g6;

        void print_data();
        
    };

    struct R_struct {
        Matrix3d u_R_LMO4, LMO4_R_u, G_R_ACSpelvis, G_R_SCSpie, G_R_LMO4, LMO4_R_G, G_R_SCS_Tib;
        void print_data();
    };

    // Struct with relevant data to check after experiment
    struct Data_struct {

        struct cad_struct {
            VectorXd FlxExt, AbdAduc, IntExt;
        };
        struct rod_struct {
            VectorXd FlxExt;
        };
        struct pie_struct {
            VectorXd FlxExt;
        };

        struct ang_struct {
            cad_struct cad;
            rod_struct rod;
            pie_struct pie;
        };

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

        ang_struct ang;
        VectorXd Time;
        std::string Date;
        VectorXd GenForceHipFlxExt, GenForceKneeFlxExtGrav, GenForceKneeFlxExtFext, GenForceKneeFlxExtMext, GenForceKneeFlxExt;
        Matrix<double,6,Dynamic> q;
        VectorXd BicFemCB, GastLat, GastMed, BicFemCL, SemTend, SemMem, Sat, Gra;
        VectorXd VasInt123, VasMedInf12, VasMedMed12, VasMedSup34, VasLatSup12, VasInt456, VasLatInf4;
        VectorXd RecFem12_1, RecFem12_2, TenFacLat;
        Matrix<double,8,Dynamic> FlxCoefMatRed;
        Matrix<double,10,Dynamic> ExtCoefMatRed;
        VectorXd F_OnTibUc, F_OnTibUt, F_ACL, F_PCL;
        VectorXi Frame;
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