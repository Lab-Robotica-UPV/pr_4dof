#ifndef PR_LIB_BIOMECH__DATA_STRUCTURES_
#define PR_LIB_BIOMECH__DATA_STRUCTURES_

#include "eigen3/Eigen/Dense"

using namespace Eigen;

namespace PRDataStructures{

    struct G_r_G_struct{

        struct PatLig_struct{
            Vector3d Pat;
        }; 

        struct Ext_struct{
            PatLig_struct PagLig;
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
        
    };

    

}

#endif // PR_LIB_BIOMECH__DATA_STRUCTURES_