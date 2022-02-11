#ifndef PR_BIOMECH__CALLIGFORCEANTA_PAU_
#define PR_BIOMECH__CALLIGFORCEANTA_PAU_

#include "pr_biomech/streaming_gdlf.hpp"

struct SemTend_struct {
    Vector3d Tib_Pel;
};

struct SemMem_struct {
    Vector3d Tib_Pel;
};

struct BicFemCB_struct {
    Vector3d Tib_Fem;
};

struct GastLat_struct {
    Vector3d Foot_Fem;
};

struct GastMed_struct {
    Vector3d Foot_Fem;
};

struct BicFemCL_struct {
    Vector3d Tib_Pel;
};

struct Sat_struct {
    Vector3d Tib_Pel;
};

struct Gra_struct {
    Vector3d Tib_Pel;
};

struct Flx_struct {
    SemTend_struct SemTend;
    SemMem_struct SemMem;
    BicFemCB_struct BicFemCB;
    GastLat_struct GastLat;
    GastMed_struct GastMed;
    BicFemCL_struct BicFemCL;
    Sat_struct Sat;
    Gra_struct Gra;

};

struct PatLig_struct {
    Vector3d Tib_Pat;
};

struct TenFacLat_struct {
    Vector3d Tib_Pel;
};

struct Ext_struct {
    PatLig_struct PatLig;
    TenFacLat_struct TenFacLat;
};

struct G_u_struct {
    Flx_struct Flx;
    Ext_struct Ext;
    void print_data() {
        std::cout << "--------G_u struct---------" << std::endl;
        std::cout << "Flx.SemTend.Tib_Pel (transposed): " << Flx.SemTend.Tib_Pel.transpose() << std::endl;
        std::cout << "Flx.SemMem.Tib_Pel (transposed): " << Flx.SemMem.Tib_Pel.transpose() << std::endl;
        std::cout << "Flx.BicFemCB.Tib_Fem (transposed): " << Flx.BicFemCB.Tib_Fem.transpose() << std::endl;
        std::cout << "Flx.GastLat.Foot_Fem (transposed): " << Flx.GastLat.Foot_Fem.transpose() << std::endl;
        std::cout << "Flx.GastMed.Foot_Fem (transposed): " << Flx.GastMed.Foot_Fem.transpose() << std::endl;
        std::cout << "Flx.BicFemCL.Tib_Pel (transposed): " << Flx.BicFemCL.Tib_Pel.transpose() << std::endl;
        std::cout << "Flx.Sat.Tib_Pel (transposed): " << Flx.Sat.Tib_Pel.transpose() << std::endl;
        std::cout << "Flx.Gra.Tib_Pel (transposed): " << Flx.Gra.Tib_Pel.transpose() << std::endl;
        std::cout << "Ext.PatLig.Tib_Pat (transposed): " << Ext.PatLig.Tib_Pat.transpose() << std::endl;
        std::cout << "Ext.TenFacLat.Tib_Pel (transposed): " << Ext.TenFacLat.Tib_Pel.transpose() << std::endl;
        std::cout << "--------G_u struct end---------" << std::endl;
    }
};

struct A_r_B_Int_struct {
    VectorXd CPL, CPL_BicFemCBTib, CPL_BicFemCLTib, CPL_GastLatFoot, CPL_GastMedFoot, CPL_Gpie_r,
        CPL_GraTib, CPL_PatLigTib, CPL_SatTib, CPL_SemMemTib, CPL_SemTendTib, CPL_TenFacLatTib, CPL_g6;
    VectorXd CPM, CPM_BicFemCBTib, CPM_BicFemCLTib, CPM_GastLatFoot, CPM_GastMedFoot, CPM_Gpie_r,
        CPM_GraTib, CPM_PatLigTib, CPM_SatTib, CPM_SemMemTib, CPM_SemTendTib, CPM_TenFacLatTib, CPM_g6;
    VectorXd KJC, KJC_BicFemCBTib, KJC_BicFemCLTib, KJC_GastLatFoot, KJC_GastMedFoot, KJC_Gpie_r,
        KJC_GraTib, KJC_PatLigTib, KJC_SatTib, KJC_SemMemTib, KJC_SemTendTib, KJC_TenFacLatTib, KJC_g6;
};

/*Función que calcula la proyeccion de un vector sobre un plano y el otro
    calcula el componente normal*/
Vector3d project(Vector3d V, Vector3d un);

// Este programas calcula el momento de todas las fuerzas alrededor de una dirección u_n
Vector3d Moment2(Matrix<double,3,Dynamic> F, Matrix<double, 3, Dynamic> O_Fp, Vector3d u_n);


#endif // PR_BIOMECH__CALLIGFORCEANTA_PAU_
