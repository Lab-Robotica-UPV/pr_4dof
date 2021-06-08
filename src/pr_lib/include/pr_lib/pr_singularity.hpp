#ifndef PR_LIB__SINGULARITY_HPP_
#define PR_LIB__SINGULARITY_HPP_

#include <vector>
#include <array>
#include <cmath>

#include "eigen3/Eigen/Dense"

#include "pr_lib/pr_model.hpp"

namespace PRSingularity
{
    //*** Calculate Ang OTS ***//

    Eigen::Matrix<double,6,1> CalculateAngOts(
        const double &theta, const double &psi,
        const Eigen::Matrix<double,4,3> &q,
		Eigen::Matrix<double,6,4> &OTS_ant,
        const std::vector<double> &RParam,
        const int iter_OTS, const double tol_OTS);

    //*** Ang OTS Jacobian Equations ***//

    void EqOTSJacobian(
        Eigen::Matrix<double,5,5> &J,
        const double &wx, const double &wy, const double &wz, 
        const double &theta, const double &psi, 
        const Eigen::Matrix<double,4,3> &q, 
        const int &op, 
        const double &Rm1, const double &Rm2, const double &Rm3, 
        const double &betaMD, const double &betaMI
    );

    void OTSSolverNR(

    );

    void EqOTS(
       Eigen::Matrix<double,5,1> &f,
       const double &wx, const double &wy, const double &wz, 
       const double &vx, const double &vz, 
       const double &theta, const double &psi, 
       const Eigen::Matrix<double,4,3> &q, 
       const int &op, 
       const double &Rm1, const double &Rm2, const double &Rm3, 
       const double &betaMD, const double &betaMI
    );

    Eigen::VectorXd PosiblesAngOTS(
        const Eigen::Vector4d &q_ind_mod,
        const Eigen::Vector4d &X_cart,
        const Eigen::Matrix<double,6,4> &OTS_med,
        const double &des_qind,
        const Eigen::Vector2i i_qind,
        const Eigen::MatrixXi &minc_des,
        const int &ncomb,
		const double &tol_FK,
		const int &iter_FK,
        const int &iter_OTS,
		const double &tol_OTS,
        const std::vector<double> &RParam,
        const Eigen::Matrix<double,4,2> &Mlim_q_ind,
		const Eigen::Vector4d &Vlim_angp
    );


    Eigen::Vector4d CalculateQindModReleaser(
        const Eigen::Vector4d &X_cart, 
        const Eigen::Vector4d &q_ref, 
        const Eigen::Matrix<double,6,1> &angOTS, 
        const Eigen::Matrix<double,6,4> &solOTS,
		const Eigen::MatrixXi &minc_des,
		const double &fj_det,
		const std::vector<double> &RParam,
		Eigen::Vector4i &vc_des,
		const Eigen::Matrix<double,4,2> &Mlim_q_ind,
		const Eigen::Vector4d &Vlim_angp,
		const double &des_qind,
		const double &lmin_Ang_OTS,
		const double &lmin_FJac,
		const double &tol_FK,
		const int &iter_FK,
		const double &tol_OTS,
		const double &iter_OTS,
		const bool &enable
    );

    Eigen::Vector4d CalculateQindModEvader(
        const Eigen::Vector4d &X_cart, 
        const Eigen::Vector4d &q_ref, 
        const Eigen::Matrix<double,6,1> &angOTS_ref, 
		const Eigen::Matrix<double,6,1> &angOTS_med,
        const Eigen::Matrix<double,6,4> &solOTS_med,
		const Eigen::MatrixXi &minc_des,
		const double &det_JDir_med,
		const double &det_JDir_ref,
		const std::vector<double> &RParam,
		Eigen::Vector4i &vc_des,
		const Eigen::Matrix<double,4,2> &Mlim_q_ind,
		const Eigen::Vector4d &Vlim_angp,
		const double &des_qind,
		const double &lmin_Ang_OTS,
		const double &lmin_JDir,
		const double &tol_FK,
		const int &iter_FK,
		const double &tol_OTS,
		const double &iter_OTS
    );

}


#endif  // PR_LIB__SINGULARITY_HPP_