#include "pr_biomech/streaming_gdlf.hpp"
#include "pr_biomech/CalLigForceAnta_Pau.hpp"

void pr_biomech::StreamingGDLF::CalLigForceAnta_Pau() {

    //// .........................................................................ELIMINAR....................................................
    //// Vector q para comprobar ..............................................................................................................ELIMINAR
    //q << 0, 0.391625383768560, 1.475558419520632, 2.959497983121001, -0.275691542445527, -3.784116796077701; //..............................ELIMINAR
    //// Estructura forLigForceLag.............................................................................................................ELIMINAR
    //forLigForceCal.FlxMusForceEstOpt << 18.499669310694124, 71.896305603978930, 3.546352498867125e+02, 94.247787068412550,
    //    33.519087228501620, 30.921926442250925, 3.860891124249192, 2.004032473048329;
    //u_n << 0.920101279398723, 0.379961533214781, 0.095094000472722;
    //// Estructura ang
    //ang.cad.FlxExt = 77.209628181188360; ang.cad.AbdAduc = 1.548151532460073; ang.cad.IntExt = 1.720169595529348e+02;
    //ang.rod.FlxExt = 52.010860965418246;
    //ang.pie.FlxExt = 28.257934671802253;
    //// .....................................................................FIN ELIMINAR......................................................

    // ................Interpolamos G_u.......................
    // G_u_Vectores
    G_u_struct G_u;

    // rclcpp::Time init = this->get_clock()->now();

    Vector2d q5i = gdlf_data->q5.segment(q5ind(0), 2);
    double q5_elem = q(4);

    G_u.Flx.BicFemCB.Tib_Fem = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->G_uMatPelvis.Flx.BicFemCB.Tib_Fem.middleCols(q5ind(0), 2));
    G_u.Flx.GastLat.Foot_Fem = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->G_uMatPelvis.Flx.GastLat.Foot_Fem.middleCols(q5ind(0), 2));
    G_u.Flx.GastMed.Foot_Fem = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->G_uMatPelvis.Flx.GastMed.Foot_Fem.middleCols(q5ind(0), 2));
    G_u.Flx.BicFemCL.Tib_Pel = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->G_uMatPelvis.Flx.BicFemCL.Tib_Pel.middleCols(q5ind(0), 2));
    G_u.Flx.Sat.Tib_Pel = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->G_uMatPelvis.Flx.Sat.Tib_Pel.middleCols(q5ind(0), 2));
    G_u.Flx.Gra.Tib_Pel = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->G_uMatPelvis.Flx.Gra.Tib_Pel.middleCols(q5ind(0), 2));
    G_u.Flx.SemTend.Tib_Pel = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->G_uMatPelvis.Flx.SemTend.Tib_Pel.middleCols(q5ind(0), 2));
    G_u.Flx.SemMem.Tib_Pel = R.G_R_ACSpelvis * InterCoef(q5_elem,q5i, gdlf_data->G_uMatPelvis.Flx.SemMem.Tib_Pel.middleCols(q5ind(0), 2));
    G_u.Ext.PatLig.Tib_Pat = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->G_uMatPelvis.Ext.PatLig.Tib_Pat.middleCols(q5ind(0), 2));
    G_u.Ext.TenFacLat.Tib_Pel = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->G_uMatPelvis.Ext.TenFacLat.Tib_Pel.middleCols(q5ind(0), 2));


    //Vectores puntos de contacto
    A_r_B_Int_struct A_r_B_Int;
    A_r_B_Int.CPL = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPL.middleCols(q5ind(0), 2));
    A_r_B_Int.CPL_BicFemCBTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPL_BicFemCBTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPL_BicFemCLTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPL_BicFemCLTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPL_GastLatFoot = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPL_GastLatFoot.middleCols(q5ind(0), 2));
    A_r_B_Int.CPL_GastMedFoot = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPL_GastMedFoot.middleCols(q5ind(0), 2));
    A_r_B_Int.CPL_Gpie_r = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPL_Gpie_r.middleCols(q5ind(0), 2));
    A_r_B_Int.CPL_GraTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPL_GraTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPL_PatLigTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPL_PatLigTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPL_SatTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPL_SatTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPL_SemMemTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPL_SemMemTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPL_SemTendTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPL_SemTendTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPL_TenFacLatTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPL_TenFacLatTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPL_g6 = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPL_g6.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPM.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_BicFemCBTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPM_BicFemCBTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_BicFemCLTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPM_BicFemCLTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_GastLatFoot = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPM_GastLatFoot.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_GastMedFoot = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPM_GastMedFoot.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_Gpie_r = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPM_Gpie_r.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_GraTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPM_GraTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_PatLigTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPM_PatLigTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_SatTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPM_SatTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_SemMemTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPM_SemMemTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_SemTendTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPM_SemTendTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_TenFacLatTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPM_TenFacLatTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_g6 = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.CPM_g6.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_BicFemCBTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_BicFemCBTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_BicFemCLTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_BicFemCLTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_GastLatFoot = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_GastLatFoot.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_GastMedFoot = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_GastMedFoot.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_Gpie_r = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_Gpie_r.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_GraTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_GraTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_PatLigTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_PatLigTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_SatTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_SatTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_SemMemTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_SemMemTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_SemTendTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_SemTendTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_TenFacLatTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_TenFacLatTib.middleCols(q5ind(0), 2));
    A_r_B_Int.CPM_g6 = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_g6.middleCols(q5ind(0), 2));
    A_r_B_Int.KJC = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC.middleCols(q5ind(0), 2));
    A_r_B_Int.KJC_BicFemCBTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_BicFemCBTib.middleCols(q5ind(0), 2));
    A_r_B_Int.KJC_BicFemCLTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_BicFemCLTib.middleCols(q5ind(0), 2));
    A_r_B_Int.KJC_GastLatFoot = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_GastLatFoot.middleCols(q5ind(0), 2));
    A_r_B_Int.KJC_GastMedFoot = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_GastMedFoot.middleCols(q5ind(0), 2));
    A_r_B_Int.KJC_Gpie_r = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_Gpie_r.middleCols(q5ind(0), 2));
    A_r_B_Int.KJC_GraTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_GraTib.middleCols(q5ind(0), 2));
    A_r_B_Int.KJC_PatLigTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_PatLigTib.middleCols(q5ind(0), 2));
    A_r_B_Int.KJC_SatTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_SatTib.middleCols(q5ind(0), 2));
    A_r_B_Int.KJC_SemMemTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_SemMemTib.middleCols(q5ind(0), 2));
    A_r_B_Int.KJC_SemTendTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_SemTendTib.middleCols(q5ind(0), 2));
    A_r_B_Int.KJC_TenFacLatTib = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_TenFacLatTib.middleCols(q5ind(0), 2));
    A_r_B_Int.KJC_g6 = R.G_R_ACSpelvis * InterCoef(q5_elem, q5i, gdlf_data->A_r_B.KJC_g6.middleCols(q5ind(0), 2));


    // G_u.Flx.BicFemCB.Tib_Fem = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->G_uMatPelvis.Flx.BicFemCB.Tib_Fem.middleCols(q5ind(0), 2));
    // G_u.Flx.GastLat.Foot_Fem = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->G_uMatPelvis.Flx.GastLat.Foot_Fem.middleCols(q5ind(0), 2));
    // G_u.Flx.GastMed.Foot_Fem = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->G_uMatPelvis.Flx.GastMed.Foot_Fem.middleCols(q5ind(0), 2));
    // G_u.Flx.BicFemCL.Tib_Pel = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->G_uMatPelvis.Flx.BicFemCL.Tib_Pel.middleCols(q5ind(0), 2));
    // G_u.Flx.Sat.Tib_Pel = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->G_uMatPelvis.Flx.Sat.Tib_Pel.middleCols(q5ind(0), 2));
    // G_u.Flx.Gra.Tib_Pel = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->G_uMatPelvis.Flx.Gra.Tib_Pel.middleCols(q5ind(0), 2));
    // G_u.Flx.SemTend.Tib_Pel = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->G_uMatPelvis.Flx.SemTend.Tib_Pel.middleCols(q5ind(0), 2));
    // G_u.Flx.SemMem.Tib_Pel = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->G_uMatPelvis.Flx.SemMem.Tib_Pel.middleCols(q5ind(0), 2));
    // G_u.Ext.PatLig.Tib_Pat = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->G_uMatPelvis.Ext.PatLig.Tib_Pat.middleCols(q5ind(0), 2));
    // G_u.Ext.TenFacLat.Tib_Pel = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->G_uMatPelvis.Ext.TenFacLat.Tib_Pel.middleCols(q5ind(0), 2));


    // //Vectores puntos de contacto
    // A_r_B_Int_struct A_r_B_Int;
    // A_r_B_Int.CPL = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPL.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPL_BicFemCBTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPL_BicFemCBTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPL_BicFemCLTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPL_BicFemCLTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPL_GastLatFoot = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPL_GastLatFoot.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPL_GastMedFoot = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPL_GastMedFoot.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPL_Gpie_r = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPL_Gpie_r.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPL_GraTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPL_GraTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPL_PatLigTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPL_PatLigTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPL_SatTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPL_SatTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPL_SemMemTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPL_SemMemTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPL_SemTendTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPL_SemTendTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPL_TenFacLatTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPL_TenFacLatTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPL_g6 = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPL_g6.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPM.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_BicFemCBTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPM_BicFemCBTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_BicFemCLTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPM_BicFemCLTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_GastLatFoot = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPM_GastLatFoot.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_GastMedFoot = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPM_GastMedFoot.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_Gpie_r = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPM_Gpie_r.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_GraTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPM_GraTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_PatLigTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPM_PatLigTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_SatTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPM_SatTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_SemMemTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPM_SemMemTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_SemTendTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPM_SemTendTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_TenFacLatTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPM_TenFacLatTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_g6 = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.CPM_g6.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_BicFemCBTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_BicFemCBTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_BicFemCLTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_BicFemCLTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_GastLatFoot = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_GastLatFoot.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_GastMedFoot = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_GastMedFoot.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_Gpie_r = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_Gpie_r.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_GraTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_GraTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_PatLigTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_PatLigTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_SatTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_SatTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_SemMemTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_SemMemTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_SemTendTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_SemTendTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_TenFacLatTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_TenFacLatTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.CPM_g6 = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_g6.middleCols(q5ind(0), 2));
    // A_r_B_Int.KJC = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC.middleCols(q5ind(0), 2));
    // A_r_B_Int.KJC_BicFemCBTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_BicFemCBTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.KJC_BicFemCLTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_BicFemCLTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.KJC_GastLatFoot = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_GastLatFoot.middleCols(q5ind(0), 2));
    // A_r_B_Int.KJC_GastMedFoot = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_GastMedFoot.middleCols(q5ind(0), 2));
    // A_r_B_Int.KJC_Gpie_r = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_Gpie_r.middleCols(q5ind(0), 2));
    // A_r_B_Int.KJC_GraTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_GraTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.KJC_PatLigTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_PatLigTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.KJC_SatTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_SatTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.KJC_SemMemTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_SemMemTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.KJC_SemTendTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_SemTendTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.KJC_TenFacLatTib = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_TenFacLatTib.middleCols(q5ind(0), 2));
    // A_r_B_Int.KJC_g6 = R.G_R_ACSpelvis * InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->A_r_B.KJC_g6.middleCols(q5ind(0), 2));

    // rclcpp::Time time1 = this->get_clock()->now();
    // double lig1 = time1.nanoseconds()*1e-6 - init.nanoseconds()*1e-6;
    // std::cout << "lig1 " << lig1 << std::endl;

    double m6 = cal_data->m(5);

    double dist_CPM_CPL = (G_r_G.CPM - G_r_G.CPL).norm();
    double dist_CPM_FCL = (G_r_G.CPM - G_r_G.FibColLigTib_un).norm();
    double dist_CPL_TCL = (G_r_G.CPL - G_r_G.TibColLigTib_un).norm();

    // Calculation process
    // Pasa la fuerza de los musculos a las variables de "FlxLegend"
    double BicFemCB = forLigForceCal.FlxMusForceEstOpt(0);
    double GastLat = forLigForceCal.FlxMusForceEstOpt(1);
    double GastMed = forLigForceCal.FlxMusForceEstOpt(2);
    double BicFemCL = forLigForceCal.FlxMusForceEstOpt(3);
    double SemTend = forLigForceCal.FlxMusForceEstOpt(4);
    double SemMem = forLigForceCal.FlxMusForceEstOpt(5);
    double Sat = forLigForceCal.FlxMusForceEstOpt(6);
    double Gra = forLigForceCal.FlxMusForceEstOpt(7);

    Vector3d SumF = project(
        m6 * cal_data->G_g + cal_data->m_pie * cal_data->G_g +
        BicFemCB * G_u.Flx.BicFemCB.Tib_Fem + GastLat * G_u.Flx.GastLat.Foot_Fem +
        GastMed * G_u.Flx.GastMed.Foot_Fem + BicFemCL * G_u.Flx.BicFemCL.Tib_Pel + //Flexion muscles
        SemTend * G_u.Flx.SemTend.Tib_Pel + SemMem * SemMem * G_u.Flx.SemMem.Tib_Pel +       //Flexion muscles
        Sat * G_u.Flx.Sat.Tib_Pel + Gra * G_u.Flx.Gra.Tib_Pel +                    //Flexion muscles
        forLigForceCal.PatLig * G_u.Ext.PatLig.Tib_Pat + forLigForceCal.TenFacLat * G_u.Ext.TenFacLat.Tib_Pel +   //Extension muscles
        G_Fext, u_n
    );

    Vector3d G_F_Reaccion = -SumF; // Knee's reactions?

    /*Nuevo, 21 / 05 / 2008
     calcular las fuerzas en los puntos de de contacto que hacen el equilibrio a nevel de momentos
     -------------------------------------------------------------------------------------------- -
     al rededor de[u_c](la linea de contacto difinida por la intersecci�n de
     un plano que contiene G_F_Reaccion y el plano de proyecci�n
     u_c = cross2(G_F_Reaccion, u_n) / norm(cross2(G_F_Reaccion, u_n));*/

    Vector3d u_c = unit((G_r_G.AJC - G_r_G.KJC).cross(u_n)); // definido normal al plano que contiene KJC y AJC y u_n
    Vector3d u_t = u_n.cross(u_c);
    double mag_F_Reaccion_u_c = G_F_Reaccion.transpose() * u_c;
    double mag_F_Reaccion_u_t = G_F_Reaccion.transpose() * u_t;
    double mag_F_Reaccion = G_F_Reaccion.norm();
    Matrix<double, 3, 13> F;
    F << m6 * cal_data->G_g, cal_data->m_pie* cal_data->G_g, BicFemCB* G_u.Flx.BicFemCB.Tib_Fem,
        GastLat* G_u.Flx.GastLat.Foot_Fem, GastMed* G_u.Flx.GastMed.Foot_Fem, BicFemCL* G_u.Flx.BicFemCL.Tib_Pel,
        SemTend* G_u.Flx.SemTend.Tib_Pel, SemMem* G_u.Flx.SemMem.Tib_Pel, Sat* G_u.Flx.Sat.Tib_Pel,
        Gra* G_u.Flx.Gra.Tib_Pel, forLigForceCal.PatLig* G_u.Ext.PatLig.Tib_Pat, forLigForceCal.TenFacLat* G_u.Ext.TenFacLat.Tib_Pel, G_Fext;
    Matrix<double, 3, 13> O_Fp_CPM, O_Fp_CPL, O_Fp_KJC;
    O_Fp_CPM << A_r_B_Int.CPM_g6, A_r_B_Int.CPM_Gpie_r, A_r_B_Int.CPM_BicFemCBTib, A_r_B_Int.CPM_GastLatFoot, A_r_B_Int.CPM_GastMedFoot,
        A_r_B_Int.CPM_BicFemCLTib, A_r_B_Int.CPM_SemTendTib, A_r_B_Int.CPM_SemMemTib, A_r_B_Int.CPM_SatTib, A_r_B_Int.CPM_GraTib,
        A_r_B_Int.CPM_PatLigTib, A_r_B_Int.CPM_TenFacLatTib, G_r_G.CPM - G_r_G.Fext_r;
    O_Fp_CPL << A_r_B_Int.CPL_g6, A_r_B_Int.CPL_Gpie_r, A_r_B_Int.CPL_BicFemCBTib, A_r_B_Int.CPL_GastLatFoot, A_r_B_Int.CPL_GastMedFoot,
        A_r_B_Int.CPL_BicFemCLTib, A_r_B_Int.CPL_SemTendTib, A_r_B_Int.CPL_SemMemTib, A_r_B_Int.CPL_SatTib, A_r_B_Int.CPL_GraTib,
        A_r_B_Int.CPL_PatLigTib, A_r_B_Int.CPL_TenFacLatTib, G_r_G.CPL - G_r_G.Fext_r;
    O_Fp_KJC << A_r_B_Int.KJC_g6, A_r_B_Int.KJC_Gpie_r, A_r_B_Int.KJC_BicFemCBTib, A_r_B_Int.KJC_GastLatFoot, A_r_B_Int.KJC_GastMedFoot,
        A_r_B_Int.KJC_BicFemCLTib, A_r_B_Int.KJC_SemTendTib, A_r_B_Int.KJC_SemMemTib, A_r_B_Int.KJC_SatTib, A_r_B_Int.KJC_GraTib,
        A_r_B_Int.KJC_PatLigTib, A_r_B_Int.KJC_TenFacLatTib, G_r_G.KJC - G_r_G.Fext_r;

    // Calculo Offline

    Vector3d M_F_uc_CPM = Moment2(F, O_Fp_CPM, u_c) + (G_Mext.transpose() * u_c).norm() * u_c;
    Vector3d M_F_uc_CPL = Moment2(F, O_Fp_CPL, u_c) + (G_Mext.transpose() * u_c).norm() * u_c;
    Vector3d M_F_uc_KJC = Moment2(F, O_Fp_KJC, u_c) + (G_Mext.transpose() * u_c).norm() * u_c;
    Vector3d M_F_un_KJC = Moment2(F, O_Fp_KJC, u_n) + (G_Mext.transpose() * u_n).norm() * u_n;
    Vector3d M_F_ut_KJC = Moment2(F, O_Fp_KJC, u_t) + (G_Mext.transpose() * u_t).norm() * u_t;

    Data.F_OnTibUc(GlobalCnt) = mag_F_Reaccion_u_c;

    /* Los �ngulos que definen la orientaci�n de los ligamentos cruzados(thetaACL, thetaPCL)
         (W.Herzogand L.J.Read 1993, Lines of actionand moments arm ...)*/
    double thetaACL = 227.0 - 0.448 * Data.ang.rod.FlxExt(GlobalCnt);
    double thetaPCL = -66.0 + 0.737 * Data.ang.rod.FlxExt(GlobalCnt) - 0.00496 * pow(Data.ang.rod.FlxExt(GlobalCnt), 2);
    thetaACL = 360.0 - thetaACL;
    thetaPCL = -thetaPCL;

    // Las fuerzas rectificadas seg�n los �ngulos de los ligamentos
    double F_ACL = 0, F_PCL = 0, F_n;
    if (mag_F_Reaccion_u_c <= 0) {
        F_ACL = mag_F_Reaccion_u_c / cosd(thetaACL);
        F_n = mag_F_Reaccion_u_t + F_ACL * sind(thetaACL);
    }
    else {
        F_PCL = mag_F_Reaccion_u_c / cosd(thetaPCL);
        F_n = mag_F_Reaccion_u_t + F_PCL * sind(thetaPCL);
    }
    Data.F_OnTibUt(GlobalCnt) = F_n;
    Data.F_ACL(GlobalCnt) = F_ACL;
    Data.F_PCL(GlobalCnt) = F_PCL;

}


Vector3d project(Vector3d V, Vector3d un) {
    /*Funci�n que calcula la proyeccion de un vector sobre un plano y el otro
    calcula el componente normal*/
    Vector3d proj = V - (V.transpose() * un) * un;
    return proj;
}

// Este programa calcula el momento de todas las fuerzas alrededor de una direcci�n u_n
Vector3d Moment2(Matrix<double, 3, Dynamic> F, Matrix<double, 3, Dynamic> O_Fp, Vector3d u_n) {
    int n = F.cols();
    Vector3d result = Vector3d::Zero();
    for (int i = 0; i < n; i++) {
        result += F.col(i).cross(O_Fp.col(i));
    }
    result = (result.transpose() * u_n) * u_n;
    return result;
}