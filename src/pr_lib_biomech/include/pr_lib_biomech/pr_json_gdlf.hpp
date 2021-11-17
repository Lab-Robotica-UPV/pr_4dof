#ifndef PR_LIB__JSON_GDLF_
#define PR_LIB__JSON_GDLF_

#include "pr_json_data.hpp"     // Inherits pr_json_data_struct

using namespace rapidjson;
using namespace Eigen;

namespace PRJsonData{

    namespace PRJsonGdlf{

        struct A_r_B_struct;
        struct G_uMatPelvis_struct;
        struct Flx_struct;
        struct SemTend_struct;
        struct SemMem_struct;
        struct BicFemCB_struct;
        struct GastLat_struct;
        struct GastMed_struct;
        struct BicFemCL_struct;
        struct Sat_struct;
        struct Gra_struct;
        struct Ext_struct;
        struct PatLig_struct;
        struct TenFacLat_struct;



        struct A_r_B_struct{
            MatrixXd CPL;
            MatrixXd CPL_BicFemCBTib;
            MatrixXd CPL_BicFemCLTib;
            MatrixXd CPL_GastLatFoot;
            MatrixXd CPL_GastMedFoot;
            MatrixXd CPL_Gpie_r;
            MatrixXd CPL_GraTib;
            MatrixXd CPL_PatLigTib;
            MatrixXd CPL_SatTib;
            MatrixXd CPL_SemMemTib;
            MatrixXd CPL_SemTendTib;
            MatrixXd CPL_TenFacLatTib;
            MatrixXd CPL_g6;
            MatrixXd CPM;
            MatrixXd CPM_BicFemCBTib;
            MatrixXd CPM_BicFemCLTib;
            MatrixXd CPM_GastLatFoot;
            MatrixXd CPM_GastMedFoot;
            MatrixXd CPM_Gpie_r;
            MatrixXd CPM_GraTib;
            MatrixXd CPM_PatLigTib;
            MatrixXd CPM_SatTib;
            MatrixXd CPM_SemMemTib;
            MatrixXd CPM_SemTendTib;
            MatrixXd CPM_TenFacLatTib;
            MatrixXd CPM_g6;
            MatrixXd KJC;
            MatrixXd KJC_BicFemCBTib;
            MatrixXd KJC_BicFemCLTib;
            MatrixXd KJC_GastLatFoot;
            MatrixXd KJC_GastMedFoot;
            MatrixXd KJC_Gpie_r;
            MatrixXd KJC_GraTib;
            MatrixXd KJC_PatLigTib;
            MatrixXd KJC_SatTib;
            MatrixXd KJC_SemMemTib;
            MatrixXd KJC_SemTendTib;
            MatrixXd KJC_TenFacLatTib;
            MatrixXd KJC_g6;

            void print_data();
        };

        struct SemTend_struct{
            MatrixXd Tib_Pel;
        };

        struct SemMem_struct{
            MatrixXd Tib_Pel;
        };

        struct BicFemCB_struct{
            MatrixXd Tib_Fem;
        };

        struct GastLat_struct{
            MatrixXd Foot_Fem;
        };

        struct GastMed_struct{
            MatrixXd Foot_Fem;
        };

        struct BicFemCL_struct{
            MatrixXd Tib_Pel;
        };

        struct Sat_struct{
            MatrixXd Tib_Pel;
        };

        struct Gra_struct{
            MatrixXd Tib_Pel;
        };

        struct Flx_struct{
            SemTend_struct SemTend;
            SemMem_struct SemMem;
            BicFemCB_struct BicFemCB;
            GastLat_struct GastLat;
            GastMed_struct GastMed;
            BicFemCL_struct BicFemCL;
            Sat_struct Sat;
            Gra_struct Gra;

        };

        struct PatLig_struct{
            MatrixXd Tib_Pat;
        };

        struct TenFacLat_struct{
            MatrixXd Tib_Pel;
        };

        struct Ext_struct{
            PatLig_struct PatLig;
            TenFacLat_struct TenFacLat;
        };

        struct G_uMatPelvis_struct{
            Flx_struct Flx;
            Ext_struct Ext;
            void print_data();
        };


        struct Gdlf_data_struct : public Data_struct{

            A_r_B_struct A_r_B;
            MatrixXd AllExtCoefMat2;
            MatrixXd AllFlxCoefMat;
            G_uMatPelvis_struct G_uMatPelvis;
            MatrixXd PatLigCoefMat;
            VectorXd q5;

            // Constructor. Receives a path and extracts the JSON content
            Gdlf_data_struct(std::string path);

            // Print data
            void print_data();

        };

    }

}

#endif // PR_LIB__JSON_GDLF_