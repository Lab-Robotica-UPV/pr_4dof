#ifndef PR_LIB__JSON_GDLF_
#define PR_LIB__JSON_GDLF_

#include "pr_json_data.hpp"     // Inherits pr_json_data_struct

using namespace rapidjson;
using namespace Eigen;

namespace PRJsonData{

    namespace PRJsonGdlf{

		struct Lig;
		struct Plane4Bar;
		struct r_Local;
		struct R_local;
		struct MusPar;

        struct Plane4Bar_struct {
            Eigen::Vector3d LMO4_u;
            Eigen::Matrix <double, 6, 1> L;
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

        struct R_local_struct {
            Eigen::Matrix3d Femur_O3;
            Eigen::Matrix3d Tibia_O5;
            Eigen::Matrix3d Pie_O7;
            void print_data();
        };

		struct MusPar_struct {

			struct AddBrevDist_struct
			{
				double Area;
				double Ang;
			};
			struct AddBrevMid_struct
			{
				double Area;
				double Ang;
			};
			struct AddBrevPro_struct
			{
				double Area;
				double Ang;
			};
			struct AddLong12_struct
			{
				double Area;
				double Ang;
			};
			struct AddLong34_struct
			{
				double Area;
				double Ang;
			};
			struct AddLong56_struct
			{
				double Area;
				double Ang;
			};
			struct AddMagnDist_struct
			{
				double Area;
				double Ang;
			};
			struct AddMagnMid12_struct
			{
				double Area;
				double Ang;
			};
			struct AddMagnMid34_struct
			{
				double Area;
				double Ang;
			};
			struct AddMagnMid56_struct
			{
				double Area;
				double Ang;
			};
			struct AddMagnProx12_struct
			{
				double Area;
				double Ang;
			};
			struct AddMagnProx34_struct
			{
				double Area;
				double Ang;
			};
			struct BicFemCB_struct
			{
				double Area;
				double Ang;
			};
			struct BicFemCL_struct
			{
				double Area;
				double Ang;
			};
			struct ExtDigLong_struct
			{
				double Area;
				double Ang;
			};
			struct ExtHalLong_struct
			{
				double Area;
				double Ang;
			};
			struct FlxDigLong_struct
			{
				double Area;
				double Ang;
			};
			struct FlxHalLong_struct
			{
				double Area;
				double Ang;
			};
			struct GastLat_struct
			{
				double Area;
				double Ang;
			};
			struct GastMed_struct
			{
				double Area;
				double Ang;
			};
			struct GemellusInf_struct
			{
				double Area;
				double Ang;
			};
			struct GemellusSup_struct
			{
				double Area;
				double Ang;
			};
			struct GlutMaxInf12_struct
			{
				double Area;
				double Ang;
			};
			struct GlutMaxInf34_struct
			{
				double Area;
				double Ang;
			};
			struct GlutMaxInf5_struct
			{
				double Area;
				double Ang;
			};
			struct GlutMaxInf6_struct
			{
				double Area;
				double Ang;
			};
			struct GlutMaxSup14_struct
			{
				double Area;
				double Ang;
			};
			struct GlutMaxSup25_struct
			{
				double Area;
				double Ang;
			};
			struct GlutMaxSup36_struct
			{
				double Area;
				double Ang;
			};
			struct GlutMedAnt123_struct
			{
				double Area;
				double Ang;
			};
			struct GlutMedAnt456_struct
			{
				double Area;
				double Ang;
			};
			struct GlutMedPost123_struct
			{
				double Area;
				double Ang;
			};
			struct GlutMedPost345_struct
			{
				double Area;
				double Ang;
			};
			struct GlutMedPost6_struct
			{
				double Area;
				double Ang;
			};
			struct GlutMinLat_struct
			{
				double Area;
				double Ang;
			};
			struct GlutMinMed_struct
			{
				double Area;
				double Ang;
			};
			struct GlutMinMid_struct
			{
				double Area;
				double Ang;
			};
			struct Gra_struct
			{
				double Area;
				double Ang;
			};
			struct IliacusLat_struct
			{
				double Area;
				double Ang;
			};
			struct IliacusMed_struct
			{
				double Area;
				double Ang;
			};
			struct IliacusMid_struct
			{
				double Area;
				double Ang;
			};
			struct ObtExtInf_struct
			{
				double Area;
				double Ang;
			};
			struct ObtExtSup_struct
			{
				double Area;
				double Ang;
			};
			struct ObtInt_struct
			{
				double Area;
				double Ang;
			};
			struct Pectineus12_struct
			{
				double Area;
				double Ang;
			};
			struct Pectineus34_struct
			{
				double Area;
				double Ang;
			};
			struct PeroBrev_struct
			{
				double Area;
				double Ang;
			};
			struct PeroLong_struct
			{
				double Area;
				double Ang;
			};
			struct PeroTert_struct
			{
				double Area;
				double Ang;
			};
			struct Piriformis_struct
			{
				double Area;
				double Ang;
			};
			struct Plantaris_struct
			{
				double Area;
				double Ang;
			};
			struct PsoasMajor1_struct
			{
				double Area;
				double Ang;
			};
			struct PsoasMajor23_struct
			{
				double Area;
				double Ang;
			};
			struct QuadFem12_struct
			{
				double Area;
				double Ang;
			};
			struct QuadFem34_struct
			{
				double Area;
				double Ang;
			};
			struct RecFem12_1_struct
			{
				double Area;
				double Ang;
			};
			struct RecFem12_2_struct
			{
				double Area;
				double Ang;
			};
			struct Sat_struct
			{
				double Area;
				double Ang;
			};
			struct SemMem_struct
			{
				double Area;
				double Ang;
			};
			struct SemTend_struct
			{
				double Area;
				double Ang;
			};
			struct SoleiusLat_struct
			{
				double Area;
				double Ang;
			};
			struct SoleusMed_struct
			{
				double Area;
				double Ang;
			};
			struct TenFacLat_struct
			{
				double Area;
				double Ang;
			};
			struct TibialAnt_struct
			{
				double Area;
				double Ang;
			};
			struct TibialPostLat_struct
			{
				double Area;
				double Ang;
			};
			struct TibialPostMed_struct
			{
				double Area;
				double Ang;
			};
			struct VasInt123_struct
			{
				double Area;
				double Ang;
			};
			struct VasInt456_struct
			{
				double Area;
				double Ang;
			};
			struct VasLatInf4_struct
			{
				double Area;
				double Ang;
			};
			struct VasLatSup12_struct
			{
				double Area;
				double Ang;
			};
			struct VasMedInf12_struct
			{
				double Area;
				double Ang;
			};
			struct VasMedMed12_struct
			{
				double Area;
				double Ang;
			};
			struct VasMedSup34_struct
			{
				double Area;
				double Ang;
			};
			//struct AddBrevDist_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct AddBrevMid_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct AddBrevPro_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct AddLong12_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct AddLong34_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct AddLong56_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct AddMagnDist_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct AddMagnMid12_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct AddMagnMid34_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct AddMagnMid56_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct AddMagnProx12_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct AddMagnProx34_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct BicFemCB_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct BicFemCL_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct ExtDigLong_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct ExtHalLong_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct FlxDigLong_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct FlxHalLong_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct GastLat_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct GastMed_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct GemellusInf_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct GemellusSup_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct GlutMaxInf12_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct GlutMaxInf34_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct GlutMaxInf5_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct GlutMaxInf6_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct GlutMaxSup14_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct GlutMaxSup25_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct GlutMaxSup36_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct GlutMedAnt123_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct GlutMedAnt456_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct GlutMedPost123_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct GlutMedPost345_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct GlutMedPost6_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct GlutMinLat_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct GlutMinMed_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct GlutMinMid_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct Gra_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct IliacusLat_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct IliacusMed_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct IliacusMid_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct ObtExtInf_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct ObtExtSup_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct ObtInt_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct Pectineus12_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct Pectineus34_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct PeroBrev_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct PeroLong_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct PeroTert_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct Piriformis_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct Plantaris_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct PsoasMajor1_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct PsoasMajor23_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct QuadFem12_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct QuadFem34_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct RecFem12_1_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct RecFem12_2_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct Sat_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct SemMem_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct SemTend_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct SoleiusLat_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct SoleusMed_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct TenFacLat_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct TibialAnt_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct TibialPostLat_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct TibialPostMed_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct VasInt123_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct VasInt456_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct VasLatInf4_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct VasLatSup12_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct VasMedInf12_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct VasMedMed12_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};
			//struct VasMedSup34_struct
			//{
			//	Eigen::Vector3d Area;
			//	Eigen::Vector3d Ang;
			//};

			AddBrevDist_struct 			AddBrevDist;
			AddBrevMid_struct 			AddBrevMid;
			AddBrevPro_struct 			AddBrevPro;
			AddLong12_struct 			AddLong12;
			AddLong34_struct 			AddLong34;
			AddLong56_struct 			AddLong56;
			AddMagnDist_struct 			AddMagnDist;
			AddMagnMid12_struct 		AddMagnMid12;
			AddMagnMid34_struct 		AddMagnMid34;
			AddMagnMid56_struct 		AddMagnMid56;
			AddMagnProx12_struct 		AddMagnProx12;
			AddMagnProx34_struct 		AddMagnProx34;
			BicFemCB_struct 			BicFemCB;
			BicFemCL_struct 			BicFemCL;
			ExtDigLong_struct 			ExtDigLong;
			ExtHalLong_struct 			ExtHalLong;
			FlxDigLong_struct 			FlxDigLong;
			FlxHalLong_struct 			FlxHalLong;
			GastLat_struct 				GastLat;
			GastMed_struct 				GastMed;
			GemellusInf_struct 			GemellusInf;
			GemellusSup_struct 			GemellusSup;
			GlutMaxInf12_struct 		GlutMaxInf12;
			GlutMaxInf34_struct 		GlutMaxInf34;
			GlutMaxInf5_struct 			GlutMaxInf5;
			GlutMaxInf6_struct 			GlutMaxInf6;
			GlutMaxSup14_struct 		GlutMaxSup14;
			GlutMaxSup25_struct 		GlutMaxSup25;
			GlutMaxSup36_struct 		GlutMaxSup36;
			GlutMedAnt123_struct 		GlutMedAnt123;
			GlutMedAnt456_struct 		GlutMedAnt456;
			GlutMedPost123_struct 		GlutMedPost123;
			GlutMedPost345_struct 		GlutMedPost345;
			GlutMedPost6_struct 		GlutMedPost6;
			GlutMinLat_struct 			GlutMinLat;
			GlutMinMed_struct 			GlutMinMed;
			GlutMinMid_struct 			GlutMinMid;
			Gra_struct 					Gra;
			IliacusLat_struct 			IliacusLat;
			IliacusMed_struct 			IliacusMed;
			IliacusMid_struct 			IliacusMid;
			ObtExtInf_struct 			ObtExtInf;
			ObtExtSup_struct 			ObtExtSup;
			ObtInt_struct 				ObtInt;
			Pectineus12_struct 			Pectineus12;
			Pectineus34_struct 			Pectineus34;
			PeroBrev_struct 			PeroBrev;
			PeroLong_struct 			PeroLong;
			PeroTert_struct 			PeroTert;
			Piriformis_struct 			Piriformis;
			Plantaris_struct 			Plantaris;
			PsoasMajor1_struct 			PsoasMajor1;
			PsoasMajor23_struct 		PsoasMajor23;
			QuadFem12_struct 			QuadFem12;
			QuadFem34_struct 			QuadFem34;
			RecFem12_1_struct 			RecFem12_1;
			RecFem12_2_struct 			RecFem12_2;
			Sat_struct 					Sat;
			SemMem_struct 				SemMem;
			SemTend_struct 				SemTend;
			SoleiusLat_struct 			SoleiusLat;
			SoleusMed_struct 			SoleusMed;
			TenFacLat_struct 			TenFacLat;
			TibialAnt_struct 			TibialAnt;
			TibialPostLat_struct 		TibialPostLat;
			TibialPostMed_struct 		TibialPostMed;
			VasInt123_struct 			VasInt123;
			VasInt456_struct 			VasInt456;
			VasLatInf4_struct 			VasLatInf4;
			VasLatSup12_struct 			VasLatSup12;
			VasMedInf12_struct 			VasMedInf12;
			VasMedMed12_struct 			VasMedMed12;
			VasMedSup34_struct 			VasMedSup34;
			void print_data();
		};

		struct Gdlf_data_struct : public Data_struct {
			Eigen::Vector3d ACSpelvis_r_PelAvg_ACSpelvis;
			std::vector <Matrix<double, 6, 11>> AllKneeCoef;
			//MatrixXd AllKneeCoef;
			Matrix <double, 9, Dynamic> AllPatCoef;
			Matrix <double, 6, Dynamic> AllTauGrav;
			Matrix <double, 8, 4 > DH_parameters;
			int n_q5;
			Eigen::Vector3d G_g;
			Eigen::Vector3d Lengths;
			Eigen::Vector3d Widths;
			Matrix <double, 73, Dynamic> MusCoef1;
			Matrix <double, 73, Dynamic> MusCoef2;
			Matrix <double, 73, Dynamic> MusCoef3;
			Matrix <double, 73, Dynamic> MusCoef4;
			Matrix <double, 73, Dynamic> MusCoef5;
			Matrix <double, 73, Dynamic> MusCoef6;
			//std::vector <Eigen:: Matrix <double, 6, 11> > AllKneeCoef;
			//std::vector < std::vector < Eigen::Vector<double, 11> > > AllKneeCoef;
			//std::vector<std::vector<Eigen::Matrix3d>> AllKneeCoef;
			Eigen::Matrix <double, Dynamic,1> q5;
			Eigen::Matrix <double, 8, 1> SegMass;
			
			//Lig_struct       Lig;
			Plane4Bar_struct Plane4Bar;
			r_Local_struct   r_Local;
			R_local_struct   R_local;
			MusPar_struct    MusPar;

			//Area efectiva
			Eigen::Matrix <double, 73, 1> AreaEfect;

			bool correct_reading = true;
			std::vector<std::string> non_correct;

			// Constructor. Receives a path and extracts the JSON content
			Gdlf_data_struct(std::string path);
			
			// Print data
			void print_data();
		};
    }

}

#endif // PR_LIB__JSON_GDLF_