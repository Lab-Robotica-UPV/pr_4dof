#include "pr_lib/pr_dmp_reversible.hpp"
#include <chrono>

// #include <eigen3/Eigen/SVD>
// #include <eigen3/Eigen/LU>

DmpReversible::DmpReversible(const Eigen::MatrixXd& trajectory, const double& ts, const int& N, const Eigen::VectorXd& k, const Eigen::VectorXd& d, const Eigen::VectorXd& m, const double& a_x)
	: trajectory(trajectory), ts(ts)
{
	// Initialization of struct params
	LWR.N = N;
	DMP.k = k;
	DMP.d = d;
	DMP.m = m;
	Phase.a_x = a_x;

	// Number of samples
	n_samples = trajectory.rows();
	// Number of DOF
	n_dof = trajectory.cols();

	// Get initial position
	DMP.y0 = trajectory.row(0);
	// Goal position
	DMP.goal = trajectory.row(trajectory.rows() - 1);

	// Struct of variables
	S.x = 1.0;
	S.y = DMP.y0;
	S.z = Eigen::VectorXd::Zero(n_dof);
	S.state.resize(2 * n_dof + 1);

	// Variables for the phase system
	Phase.tau = (n_samples - 1) * ts;
	Phase.tau0 = Phase.tau;

	// Function approximator variables
	Eigen::VectorXd c_lin = Eigen::VectorXd::LinSpaced(LWR.N, 0, 1);
	LWR.c = (-Phase.a_x * c_lin).array().exp();
	Eigen::VectorXd sigma2_aux = (diff(LWR.c) * 0.75).array().square();
	LWR.sigma2.resize(LWR.N);
	LWR.sigma2 << sigma2_aux, sigma2_aux.bottomRows(1);
	// Resizing the rest of variables for LWR
	LWR.psi.resize(LWR.N);
	LWR.dpsi.resize(LWR.N);
	LWR.ddpsi.resize(LWR.N);
	LWR.XX.resize(n_samples, LWR.N);
	LWR.PSI.resize(n_samples, LWR.N);
	LWR.F.resize(n_samples, n_dof);
	ones = Eigen::VectorXd::Ones(LWR.N);

	LWR.W = Eigen::MatrixXd::Zero(LWR.N, n_dof);

	// Integration variables
	Int.f.resize(n_dof);
	Int.df.resize(n_dof);
	Int.ddf.resize(n_dof);
	Int.prealloc_dphi.resize(n_dof);
	Int.prealloc_ddphi.resize(n_dof);
	Int.prealloc_psi_times_W.resize(n_dof);
	Int.prealloc_dpsi_times_W.resize(n_dof);
	Int.prealloc_ddpsi_times_W.resize(n_dof);
	Int.yx.resize(n_dof);
	Int.dyx.resize(n_dof);
	Int.ddyx.resize(n_dof);
	Int.dz.resize(n_dof);
	Int.dy.resize(n_dof);
}

DmpReversible::DmpReversible(const Eigen::MatrixXd& trajectory, const double& ts, const int& N, const double& k, const double& d, const double& m, const double& a_x)
	: DmpReversible(trajectory, ts, N, k* Eigen::VectorXd::Ones(trajectory.cols()), d* Eigen::VectorXd::Ones(trajectory.cols()), m* Eigen::VectorXd::Ones(trajectory.cols()), a_x) {}

void DmpReversible::set_spring_param(const Eigen::VectorXd& spring_param) { DMP.k = spring_param; }
void DmpReversible::set_damping_param(const Eigen::VectorXd& damping_param) { DMP.d = damping_param; }
void DmpReversible::set_mass_param(const Eigen::VectorXd& mass_param) { DMP.k = mass_param; }
void DmpReversible::set_goal(const Eigen::VectorXd& goal) { DMP.goal = goal; }
void DmpReversible::set_speed(const double& speed) { Phase.tau = Phase.tau0 / speed; }

Eigen::VectorXd DmpReversible::get_spring_param() { return DMP.k; };
Eigen::VectorXd DmpReversible::get_damping_param() { return DMP.d; };
Eigen::VectorXd DmpReversible::get_mass_param() { return DMP.m; };
Eigen::VectorXd DmpReversible::get_initial_position() { return DMP.y0; };
Eigen::VectorXd DmpReversible::get_goal() { return DMP.goal; };
double DmpReversible::get_tau() { return Phase.tau; };

void DmpReversible::train() {

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	// Position, velocity and acceleration
	Eigen::MatrixXd dy = Eigen::MatrixXd::Zero(n_samples, n_dof);
	dy.bottomRows(n_samples - 1) = diff(trajectory) / 0.01;
	Eigen::MatrixXd ddy = Eigen::MatrixXd::Zero(n_samples, n_dof);
	ddy.bottomRows(n_samples - 1) = diff(dy) / 0.01;
	// Phase
	double x = 1;
	double dx = 0;


	for (int i = 0; i < n_samples; i++) {

		// Kernel activations
		LWR.PSI.row(i) = (-0.5 * (x * ones - LWR.c).array().square() / (LWR.sigma2.array())).exp();
		LWR.XX.row(i) = LWR.PSI.row(i) / LWR.PSI.row(i).sum();

		// Phase evolution
		dx = -Phase.a_x * x / Phase.tau;
		x = x + dx * ts;
	}

	// Target for fitting
	LWR.F = trajectory.rowwise() - DMP.goal.transpose();

	// Least squares
	//std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	//LWR.W = LWR.XX.colPivHouseholderQr().solve(LWR.F);
	//LWR.W = LWR.XX.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(LWR.F);
	LWR.W = (LWR.XX.transpose() * LWR.XX).ldlt().solve(LWR.XX.transpose() * LWR.F);
	//LWR.W = LWR.XX.householderQr().solve(LWR.F);
	//LWR.W = (LWR.XX.transpose() * LWR.XX).inverse() * LWR.XX.transpose() * LWR.F;
	
	//std::cout << LWR.c << std::endl;
	
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

	std::cout << "Reversible DMP training time = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()/1000000 << "[s]" << std::endl;
};

Eigen::VectorXd DmpReversible::integrate(double speed, double delay,const Eigen::VectorXd &accel_force) {

	Eigen::internal::set_is_malloc_allowed(false);

	// Phase variable update (using speed in the velocity)
	// If speed=1.0, the phase evolves speed in normal speed
	// If speed=-1.0, the phase evolves backwards in normal speed
	// In between we can control the velocity (instead of modifying tau). Specifically, for speed=0.0 the phase stops
	Phase.dx = -speed * Phase.a_x * S.x / ((Phase.tau) * (1 + delay));
	// For ddx, we have to derive with respect to speed and x with the chain rule. 
	// In the first iteration we consider the derivative of speed to be 0.
	// For the rest of iterations, the derivative is calculated with increment divided by ts.
	if (first_iter)
		Phase.ddx = - Phase.a_x / (Phase.tau * (1 + delay))*(speed * Phase.dx);
	else{
		Phase.ddx = - Phase.a_x / (Phase.tau * (1 + delay))*(speed * Phase.dx + (speed - Phase.speed_ant)/ts*S.x);
		first_iter = false;
	}
	Phase.speed_ant = speed;
	S.x = S.x + Phase.dx * ts;

	// To avoid strange behaviour, we clip the phase
	if (S.x > 1) {
		S.x = 1;
		Phase.dx = 0;
		Phase.ddx = 0;
	}

	if (S.x < LWR.c(LWR.N-1)) {
		S.x = LWR.c(LWR.N - 1);
		Phase.dx = 0;
		Phase.ddx = 0;
	}
	
	// The weighted sum of the locally weighted regression models
	LWR.psi = (-0.5 * (S.x * ones - LWR.c).array().square() / (LWR.sigma2.array())).exp();
	LWR.dpsi = -(S.x * ones - LWR.c).cwiseQuotient(LWR.sigma2).cwiseProduct(LWR.psi);
	LWR.ddpsi = -LWR.psi.cwiseQuotient(LWR.sigma2) - (S.x * ones - LWR.c).cwiseQuotient(LWR.sigma2).cwiseProduct(LWR.dpsi);

	// Calculation of f and the derivatives df/dt = df/dx*dx/dt. df/dx has been calculated analytically in Maple
	// for the second derivative: d2f/dt2 = 
	Int.sum_psi = epsilon + LWR.psi.sum();
	Int.sum_psi2 = epsilon + pow(LWR.psi.sum(), 2);
	Int.sum_psi3 = epsilon + pow(LWR.psi.sum(), 3);
	Int.prealloc_psi_times_W.noalias() = (LWR.psi.transpose() * LWR.W);
	Int.prealloc_dpsi_times_W.noalias() = (LWR.dpsi.transpose() * LWR.W);
	Int.prealloc_ddpsi_times_W.noalias() = (LWR.ddpsi.transpose() * LWR.W);

	Int.prealloc_dphi.noalias() = Int.prealloc_dpsi_times_W / Int.sum_psi - Int.prealloc_psi_times_W * LWR.dpsi.sum() / Int.sum_psi2;
	Int.prealloc_ddphi.noalias() = Int.prealloc_ddpsi_times_W / Int.sum_psi -
		2 * Int.prealloc_dpsi_times_W * LWR.dpsi.sum() / Int.sum_psi2 +
		2 * Int.prealloc_psi_times_W * pow(LWR.dpsi.sum(), 2) / Int.sum_psi3 -
		Int.prealloc_psi_times_W * LWR.ddpsi.sum() / Int.sum_psi2;

	Int.f = Int.prealloc_psi_times_W / Int.sum_psi;
	Int.df = Int.prealloc_dphi * Phase.dx;
	Int.ddf = Int.prealloc_ddphi * pow(Phase.dx, 2) + Int.prealloc_dphi * Phase.ddx;

	
	//// Estimated outputs
	Int.yx = Int.f + DMP.goal;
	Int.dyx = Int.df;
	Int.ddyx = Int.ddf;

	// Derivatives for the DMP
	Int.dz = Int.ddyx + (-DMP.d.cwiseProduct(S.z - Int.dyx) - DMP.k.cwiseProduct(S.y - Int.yx)).cwiseQuotient(DMP.m) + accel_force.cwiseQuotient(DMP.m);
	Int.dy = S.z;

	// Update by Euler integration
	S.z += Int.dz * ts;
	S.y += Int.dy * ts;

	//State to return
	S.state << S.x, S.y, S.z;

	Eigen::internal::set_is_malloc_allowed(true);
	return S.state;
}

Eigen::VectorXd DmpReversible::integrate(double speed, double delay){
	return integrate(speed, delay, Eigen::VectorXd::Zero(n_dof));
}	

// diff function from Matlab
Eigen::MatrixXd DmpReversible::diff(const Eigen::MatrixXd& M) {
	Eigen::MatrixXd M1 = M.block(0, 0, M.rows() - 1, M.cols());
	Eigen::MatrixXd M2 = M.block(1, 0, M.rows() - 1, M.cols());
	return M2 - M1;
}