#ifndef PR_LIB__DMP_REVERSIBLE_HPP_
#define PR_LIB__DMP_REVERSIBLE_HPP_

#define EIGEN_RUNTIME_NO_MALLOC  // Enable runtime tests for allocations

// #include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/LU>
#include "pr_utils.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>

#include <math.h>
#include <vector>

//using namespace Eigen;
// Note that de class Matrix is in Eigen and also in GMR, so we not including the Eigen namespace
// to avoid colission
using namespace std;

class DmpReversible {

public:
	/**
	   *  Initialization constructor.
	   *  \param trajectory      Trajectory Matrix
	   *  \param ts				 Sample time
	   *  \param N				 Number of basis functions
	   *  \param k				 Stiffness for the spring (if scalar, it is converted to a vector with the same values for all DOFs)
	   *  \param d				 Damping for the spring (if scalar, it is converted to a vector with the same values for all DOFs)
	   *  \param m				 Mass for the spring (if scalar, it is converted to a vector with the same values for all DOFs)
	   *  \param a_x			 Exponential decay for the phase dynamical system
	*/
	DmpReversible(const Eigen::MatrixXd& trajectory, const double& ts, const int& N, const double& k, const double& d, const double& m, const double& a_x = 2.0);
	DmpReversible(const Eigen::MatrixXd& trajectory, const double& ts, const int& N, const Eigen::VectorXd& k, const Eigen::VectorXd& d, const Eigen::VectorXd& m, const double& a_x = 2.0);

	/**   Set the spring parameter of the mass-spring-damper system  */
	void set_spring_param(const Eigen::VectorXd& spring_param);
	/**   Set the damping parameter of the mass-spring-damper system  */
	void set_damping_param(const Eigen::VectorXd& damping_param);
	/**   Set the mass parameter of the mass-spring-damper system  */
	void set_mass_param(const Eigen::VectorXd& mass_param);
	// Set the goal
	void set_goal(const Eigen::VectorXd& goal);
	// Set the speed
	void set_speed(const double& speed);

	/**   Get the spring parameter of the mass-spring-damper system  */
	Eigen::VectorXd get_spring_param();
	/**   Get the damping parameter of the mass-spring-damper system  */
	Eigen::VectorXd get_damping_param();
	/**   Get the mass parameter of the mass-spring-damper system  */
	Eigen::VectorXd get_mass_param();
	// Get the initial position
	Eigen::VectorXd get_initial_position();
	// Get the goal position
	Eigen::VectorXd get_goal();
	// Get the current tau
	double get_tau();

	// Train function approximators
	void train();
	// Integrate DMP
	Eigen::VectorXd integrate(double speed, double delay, const Eigen::VectorXd &accel_force);
	Eigen::VectorXd integrate(double speed=1.0, double delay=0.0);

	// Gradient of Matrix (Diff of matlab)
	Eigen::MatrixXd diff(const Eigen::MatrixXd& M);



private:
	// epsilon to avoid dividing by 0
	double epsilon = 1.0e-10;
	// Trajectory path name
	string file_path;
	// Position matrix for learning (initial complete trajectory)
	Eigen::MatrixXd trajectory;
	// Sample time
	double ts;
	// Number of samples;
	double n_samples;
	// Number of DOF;
	double n_dof;
	// Vector of ones to use for the phase
	Eigen::VectorXd ones;
	// First iter
	bool first_iter = true;
	// Struct with DMP params
	struct {
		// Spring param
		Eigen::VectorXd k;
		// Damping param
		Eigen::VectorXd d;
		// Mass param
		Eigen::VectorXd m;
		// Initial position
		Eigen::VectorXd y0;
		// Goal
		Eigen::VectorXd goal;
	} DMP;
	// Struct with data resulting from integration
	struct S {
		// phase
		double x;
		// Position
		Eigen::VectorXd y;
		// Velocity
		Eigen::VectorXd z;
		// Stacked state vector to return
		Eigen::VectorXd state;
	} S;
	// Struct for the phase system
	struct {
		// Tau for speed
		double tau;
		// Initial tau to control the speed relative to the initial
		double tau0;
		// speed_ant to calculate speed derivative (1.0 for normal forward, 0.0 to stop, -1.0 for normal backward)
		double speed_ant;
		// Exponential decay term
		double a_x;
		// Velocity
		double dx;
		// Acceleration
		double ddx;
	} Phase;
	// Struct for the function approximators LWR
	struct {
		// Number of basis functions
		int N;
		// Centers of the Gaussian kernels
		Eigen::VectorXd c;
		// Variances of the Gaussian kernels
		Eigen::VectorXd sigma2;
		// Weighted sum of locally weighted regression models
		Eigen::VectorXd psi;
		// Weighted sum of locally weighted regression models for learning
		Eigen::MatrixXd PSI;
		// Derivative of the weighted sum
		Eigen::VectorXd dpsi;
		// Second derivative of the weighted sum
		Eigen::VectorXd ddpsi;
		// Normalized sum of psi for LWR
		Eigen::MatrixXd XX;
		// Weights
		Eigen::MatrixXd W;

		// Function approximator for regression
		Eigen::MatrixXd F;

	} LWR;
	// Struct for the integration steps
	struct {
		// Function approximator and derivatives
		Eigen::VectorXd f, df, ddf;
		// Estimated outputs
		Eigen::VectorXd yx, dyx, ddyx;
		// Auxiliary values for computing df and ddf
		Eigen::VectorXd prealloc_psi_times_W, prealloc_dpsi_times_W, prealloc_ddpsi_times_W;
		Eigen::VectorXd prealloc_dphi, prealloc_ddphi;
		// Current velocity and acceleration
		Eigen::VectorXd dy, dz;
		// Denominators for the derivatives
		double sum_psi, sum_psi2, sum_psi3;
	}  Int;

};

#endif