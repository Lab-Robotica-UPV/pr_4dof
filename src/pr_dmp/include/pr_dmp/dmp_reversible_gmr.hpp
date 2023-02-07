#ifndef PR_DMP__REVERSIBLE_GMR_HPP_
#define PR_DMP__REVERSIBLE_GMR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_float_h.hpp"
#include "std_msgs/msg/bool.hpp"
#include "pr_lib/pr_utils.hpp"
#include "pr_lib/pr_model.hpp"

#include <vector>

#include "pr_lib/pr_dmp_reversible.hpp"


#include <iostream>
#include <fstream>

// For GMR
#include "MathLib.h"
#include "gmr.h"



using namespace std;
// using namespace Eigen;
// Note that de class Matrix is in Eigen and also in GMR, so we not including the Eigen namespace
// to avoid colission

namespace pr_dmp
{
    class DmpRevGMR : public rclcpp::Node
    {
        public:
            //PR_AUX_PUBLIC
            explicit DmpRevGMR(const rclcpp::NodeOptions & options);
            ~DmpRevGMR();

        protected:
            // Callback for the force
            void force_callback(const pr_msgs::msg::PRArrayH::SharedPtr force_msg);
            // Callback for the joint position    
            void pos_callback(const pr_msgs::msg::PRArrayH::SharedPtr pos_msg);

        private:

            // PUBLISHERS AND SUBSCRIBERS
            // Subscriptor for the acceleration force for the DMP
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_force_;
            // Subscription to the joint position
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_pos_;
            // Publisher of joint position
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_q_;
            // Publisher of cart position (will be zeros if isCart and calcCart are both set to false)
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_x_;
            // Publisher of phase
            rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr publisher_phase_;
            // Publisher of phase speed
            rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr publisher_phase_speed_;
            // Publisher of phase speed
            rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr publisher_error_gmr_;
            // Publisher that ends the streaming
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_end_;
            
            // PARAMETERS
            
            // Trajectory file (cart or prism, indicated by the bool "isCart")
            // Where to save the DMP learned parameters
            string ref_path;
            // Sample time
            double ts;
            // Number of basis functions
            int n_basis_functions;
            // isCart True if the file is in cartesian coordinates (false if it is in prismatic)
            // In case of a prismatic file, calcCart express whether to calculate the cartesian (otherwise the topic will send 0)
            // This is useful when we only have the prismatic file (for example for moving disassembled legs)
            // Or when we do not have a cartesian file (since we need ref_x_init to start calculating it)
            bool isCart, calcCart;
            // Init cartesian position. Only needed if calcCart is in true
            std::vector<double> ref_x_init;
            // Robot config params for calculating the Direct/Inverse Kinematics
            std::vector<double> robot_params;
            // Constant values of the DMP:
            std::vector<double> damping_coefficient, spring_constant, mass;
            // Speed
            double speed;

            // TRAJECTORY
            // Consideration! The trajectory must have just one blank space between two values of the same sample
            // and one enter between two lines
            // ref_read=True if we read the trajectory successfully
            bool ref_read = false;
            // Matrices containing trajectories
            Eigen::MatrixXd ref_pos;
            // Number of inputs of the trajectory (columns)
            int n_dims;
            // First iteration of the trajectory
            bool first_iter = true;
            // Number of iterations
            int n_iter = 0;
            // When the task is finished, set end_task to true
            bool end_task = false;


            // DMP
            // Variable for the DMP
            DmpReversible* dmp;
            Eigen::VectorXd damping_coefficient_vec;
            Eigen::VectorXd spring_constant_vec;
            Eigen::VectorXd mass_vec;
            double a_x = 2.0;
            // State of DMP and x, y, z
            Eigen::VectorXd state_dmp;
            double x_dmp = 1.0;
            Eigen::VectorXd y_dmp, z_dmp;
            // Forces to apply (coming from the callbacks)
            Eigen::Vector4d force = Eigen::Vector4d::Zero();
            // Current phase of the (exponential) phase system
            Eigen::VectorXd phase_current = Eigen::VectorXd::Zero(1);
            // Phase limit
            double phase_limit;
            // Gain of the slowdown
            double gain_slowdown;
            // Variable of slowdown of the phase variable
            double slowdown;
             
            
            // FORWARD AND INVERSE KINEMATICS
            // Variables for direct kinematics
            double tol = 0.0000001;
            int iter = 30;
            // Previous value of the iteration of direct kinematics
            std::vector<double> x_dir_kin;
            // Complete solution of the inverse kinematics
            Eigen::Matrix<double, 4, 3> q_inv_kin;

            // For GMR
            GaussianMixture gmr;
            bool gmr_activate;
            // Path with GMR data
            std::string gmr_path;
            // Matrix 1x1 with input (Matrix from GMR library)
            Matrix input_gmr;
            // Matrices to store the mean and variance in each iteration
            Matrix mean_gmr;
            Matrix *sigma_gmr;
            // Columns vector of the regression (the first column is the time input)
            // The rest of columns are the outputs
            Vector inC;
            Vector outC;
            // Norm of the force
            double norm_f;
            // Error to evalaute
            double error = 0.0;
            // How many times to surpass the standard deviation for the error
            double relative_error_limit = 5.0;
            // Scaling factor for the moments to obtain force (F=M/l)
            double scaling_factor = 0.5;

    };
}

#endif // PR_DMP__REVERSIBLE_GMR_HPP_