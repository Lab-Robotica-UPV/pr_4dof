#include "pr_biomech/streaming_gdlf.hpp"

void pr_biomech::StreamingGDLF::MusForceOpt2_Anta_Coef_Guard_Red() {
	
	/*----FUCTIONS'S PARAMETERS----*/

	//Effective area of muscles - FLEXORS
	const double BicFemCB_A = 11.8 / 10000;
	const double GastLat_A = 24.0 / 10000;
	const double GastMed_A = 43.8 / 10000;
	const double BicFemCL_A = 27.2 / 10000;
	const double SemTend_A = 14.7 / 10000;
	const double SemMem_A = 17.1 / 10000;
	const double Sat_A = 5.9 / 10000;
	const double Gra_A = 4.9 / 10000;

	//Effective area of muscles - EXTENDERS
	const double TenFacLat_A = 8.8 / 10000;
	const double VasInt123_A = 38.1 / 2 / 10000;
	const double VasMedInf12_A = 9.8 / 10000;
	const double VasMedMed12_A = 23.2 / 10000;
	const double VasMedSup34_A = 26.9 / 10000;
	const double VasLatSup12_A = 59.0 / 10000;
	const double VasInt456_A = 38.1 / 2 / 10000;
	const double VasLatInf4_A = 10.7 / 10000;
	const double RecFem12_1_A = 28.9 / 2 / 10000;
	const double RecFem12_2_A = 28.9 / 2 / 10000;

	//Maximun Teorical stress
	const double MaxEstres = 86.0 * 10000.0;  //N/m^2 

	//Flexor muscle areas joint in a vector
	Matrix<double, 1, 8> FlxMusArea;
	FlxMusArea << BicFemCB_A, GastLat_A, GastMed_A, BicFemCL_A, SemTend_A, SemMem_A, Sat_A, Gra_A;
	//Maximun stress for Flexors
	Matrix<double, 1, 8> FlxMaxMusForce;
	FlxMaxMusForce = FlxMusArea * MaxEstres;
	//Flexor muscle legends
	const std::vector<std::string>  FlxLegend{ "BicFemCB","GastLat","GastMed","BicFemCL","SemTend","SemMem","Sat","Gra" };

	//Extender muscle areas joint in a vector
	Matrix<double, 1, 10> ExtMusArea;
	ExtMusArea << VasInt123_A, VasMedInf12_A, VasMedMed12_A, VasMedSup34_A, VasLatSup12_A, VasInt456_A, VasLatInf4_A, RecFem12_1_A, RecFem12_2_A, TenFacLat_A;
	//Maximun stress for Extenders
	Matrix<double, 1, 10> ExtMaxMusForce;
	ExtMaxMusForce = ExtMusArea * MaxEstres;
	//Extender muscle legends
	const std::vector<std::string>  ExtLegend{ "VasInt123","VasMedInf12","VasMedMed12","VasMedSup34","VasLatSup12","VasInt456","VasLatInf4","RecFem12_1","RecFem12_2","TenFacLat" };


	/*----MUSCLE'S FORCES OPTIMIZATION----*/
	//Transpose of the vector Tau_TOT_7_T
	Matrix<double, 1, 5> Tau_TOT_7_T = Tau_TOT_7.transpose();

	//Calculate the number of rows for Tau_TOT_7_T
	const int Npts = 1;		 //Linea original: Tau_TOT_7_T.rows()

	//Coefficient for the the flexor muscles
	Matrix<double, 1, 4> Tau_TOT_V = Tau_TOT_7_T.middleCols(1, 4);

	//Extraction of the offline coefficients of the muscles
	q5ind = FindLoc(q(4), gdlf_data->q5);
	Matrix<double, 1, 8> FlxCoefMatRed = InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->AllFlxCoefMat.middleCols(q5ind(0), 2));
	Matrix<double, 1, 10> ExtCoefMatRed = InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->AllExtCoefMat2.middleCols(q5ind(0), 2));
	Matrix<double, 1, 9> PatLigCoef = InterCoef(q(4), gdlf_data->q5.segment(q5ind(0), 2), gdlf_data->PatLigCoefMat.middleCols(q5ind(0), 2));
	//Prints for verification --------------------------------------Borrar al final
	// std::cout  << "FlxCoefMatRed:\t" << FlxCoefMatRed << std::endl;
	// std::cout << "ExtCoefMatRed:\t" << ExtCoefMatRed << std::endl;
	// std::cout << "PatLigCoef:\t" << PatLigCoef << std::endl;

	//Number of intervals for Flextion-Extention
	std::string  FrstActive;
	bool FlxActive;
	int TauSign;

	if (Tau_TOT_7_T(4) > 0) {
		FrstActive = "F";
		FlxActive = true;
		TauSign = 1;
	}
	else {
		FrstActive = "E";
		FlxActive = false;
		TauSign = -1;
	}
	//Prints for verification --------------------------------------Borrar al final
	//std::cout  << "FrstActive:\t" << FrstActive << std::endl;


	//Optimization loop
	bool FlxActiveOrg = FlxActive;
	Matrix<double, 1, 8> FlxMusForceEstOpt;
	FlxMusForceEstOpt.fill(0);
	Matrix<double, 1, 10> ExtMusForceEstOpt;
	ExtMusForceEstOpt.fill(0);
	//std::string  prefix;
	//MatrixXd CoefMatRed(1,10), MaxMusForce(1, 10), MusArea(1, 10);
	Matrix<double, 1, Dynamic> CoefMatRed, MaxMusForce, MusArea;  //, ub
	std::vector<std::string> Legend;
	Matrix<double, 1, Dynamic> MusForceEstOpt, MusEstresEstOpt, lb;
	Matrix<double, 1, Dynamic> Aeq, auxnumMusForceEstOpt, auxdenMusForceEstOpt;


	for (int k = 0; k < 2; k++) {
		//Coactivation section
		if (k == 0) {
			Tau_TOT_7_T(4) = Tau_TOT_V(3) * (1 + per);
		}
		else {
			Tau_TOT_7_T(4) = Tau_TOT_V(3) * (-per);
		}
		//Flexion-Extensi�n limits selection
		if (FlxActive) {
			//prefix = "Flx";
			FlxActive = false;
			CoefMatRed.resize(1, FlxCoefMatRed.cols());
			CoefMatRed = FlxCoefMatRed;
			MaxMusForce.resize(1, FlxMaxMusForce.cols());
			MaxMusForce = FlxMaxMusForce;
			MusArea.resize(1, FlxMusArea.cols());
			MusArea = FlxMusArea;
			//Legend = FlxLegend;
			//ub = FlxMaxMusForce;
		}
		else {
			//prefix = "Ext";
			FlxActive = true;
			CoefMatRed.resize(1, ExtCoefMatRed.cols());
			CoefMatRed = ExtCoefMatRed;
			MaxMusForce.resize(1, ExtMaxMusForce.cols());
			MaxMusForce = ExtMaxMusForce;
			MusArea.resize(1, ExtMusArea.cols());
			MusArea = ExtMusArea;
			//Legend = ExtLegend;
			//ub = ExtMaxMusForce;
		}
		//Prints for verification --------------------------------------Borrar al final
		//std::cout << "Tot Cols of CoefMatRed:\t" << CoefMatRed.cols() << std::endl;

		//Variables for optimization results
		int dof = CoefMatRed.cols();
		MusForceEstOpt.resize(1, dof);
		MusForceEstOpt.fill(0);
		MusEstresEstOpt.resize(1, dof);
		MusEstresEstOpt.fill(0);
		lb.resize(1, dof);
		lb.fill(0);
		//Prints for verification --------------------------------------Borrar al final
		// std::cout << "MusForceEstOpt:\t" << MusForceEstOpt << std::endl;
		//Linear constraint equations
		Aeq = CoefMatRed;
		for (int c1 = 0; c1 < Aeq.cols(); c1++) {
			if (Aeq(c1) * Tau_TOT_7_T(0, 4) < 0) {
				Aeq(c1) = 0;
			}
		}

		//Optimization of forces in muscles
		double beq = Tau_TOT_7_T(0, 4);
		auxnumMusForceEstOpt.resize(1, MusArea.cols());
		auxnumMusForceEstOpt = MusArea.array().square();
		auxnumMusForceEstOpt = Aeq.cwiseProduct(beq * auxnumMusForceEstOpt);
		auxdenMusForceEstOpt = (Aeq.cwiseProduct(MusArea)).array().square();
		MusForceEstOpt = auxnumMusForceEstOpt.array() / auxdenMusForceEstOpt.sum();
		MusEstresEstOpt = MusForceEstOpt.array() / MusArea.array();
		//Prints for verification --------------------------------------Borrar al final
		// std::cout << "CalculoMusForceEstOpt:\t" << MusForceEstOpt << std::endl;
		// std::cout << "CalculoMusEstresEstOpt:\t" << MusEstresEstOpt << std::endl;
		//Selection of the Optimal forces
		if (FlxActive) {
			ExtMusForceEstOpt = MusForceEstOpt;
		}
		else {
			FlxMusForceEstOpt = MusForceEstOpt;
		}

	}
	//Prints for verification --------------------------------------Borrar al final
	//std::cout  << "FlxMusForceEstOpt:\t" << FlxMusForceEstOpt << std::endl;
	//std::cout << "ExtMusForceEstOpt:\t" << ExtMusForceEstOpt << std::endl;

	//Force in the patellar ligament
	double PatLig = ExtMusForceEstOpt.segment(0, 9) * PatLigCoef.transpose();
	//Prints for verification --------------------------------------Borrar al final
	// std::cout << "CalPatlig:\t" << PatLig << std::endl;

	//Forces in the flexor muscles
	forLigForceCal.FlxActiveOrg = FlxActiveOrg;
	forLigForceCal.FlxLegend = FlxLegend;
	forLigForceCal.FlxMusForceEstOpt = FlxMusForceEstOpt.transpose();
	forLigForceCal.FlxMaxMusForce = FlxMaxMusForce.transpose();

	//Forces in the extensor muscles
	forLigForceCal.ExtLegend = ExtLegend;
	forLigForceCal.ExtMusForceEstOpt = ExtMusForceEstOpt.transpose();
	forLigForceCal.ExtMaxMusForce = ExtMaxMusForce.transpose();
	forLigForceCal.PatLig = PatLig;
	forLigForceCal.RecFem12_1 = ExtMusForceEstOpt(0, 7);
	forLigForceCal.RecFem12_2 = ExtMusForceEstOpt(0, 8);
	forLigForceCal.TenFacLat = ExtMusForceEstOpt(0, 9);

	//Prints for verification --------------------------------------Borrar al final
	//std::cout <<"RES" << "FUN JOSE" << std::endl;
	//forLigForceCal.print_data();


	//Save the flexor muscles results
	Data.BicFemCB(GlobalCnt) = FlxMusForceEstOpt(0);
	Data.GastLat(GlobalCnt) = FlxMusForceEstOpt(1);
	Data.GastMed(GlobalCnt) = FlxMusForceEstOpt(2);
	Data.BicFemCL(GlobalCnt) = FlxMusForceEstOpt(3);
	Data.SemTend(GlobalCnt) = FlxMusForceEstOpt(4);
	Data.SemMem(GlobalCnt) = FlxMusForceEstOpt(5);
	Data.Sat(GlobalCnt) = FlxMusForceEstOpt(6);
	Data.Gra(GlobalCnt) = FlxMusForceEstOpt(7);

	//Save the extensor muscles results
	Data.VasInt123(GlobalCnt) = ExtMusForceEstOpt(0);
	Data.VasMedInf12(GlobalCnt) = ExtMusForceEstOpt(1);
	Data.VasMedMed12(GlobalCnt) = ExtMusForceEstOpt(2);
	Data.VasMedSup34(GlobalCnt) = ExtMusForceEstOpt(3);
	Data.VasLatSup12(GlobalCnt) = ExtMusForceEstOpt(4);
	Data.VasInt456(GlobalCnt) = ExtMusForceEstOpt(5);
	Data.VasLatInf4(GlobalCnt) = ExtMusForceEstOpt(6);
	Data.RecFem12_1(GlobalCnt) = ExtMusForceEstOpt(7);
	Data.RecFem12_2(GlobalCnt) = ExtMusForceEstOpt(8);
	Data.TenFacLat(GlobalCnt) = ExtMusForceEstOpt(9);

	Data.FlxCoefMatRed.col(GlobalCnt) = FlxCoefMatRed.transpose();
	Data.ExtCoefMatRed.col(GlobalCnt) = ExtCoefMatRed.transpose();
	//Prints for verification --------------------------------------Borrar al final
	//Data.print_data();

	//Prints for verification --------------------------------------Borrar al final
	//std::cout << "---------------Fin Funcion MusForceOpt2_Anta_Coef_Guard_Red---------------" << std::endl;
	//std::cout << "Elapsed time:" << etime.count() << std::endl;
}