#ifndef PR_DMP__GEN_REF_HPP_
#define PR_DMP__GEN_REF_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "std_msgs/msg/bool.hpp"
#include "pr_lib/pr_utils.hpp"
#include "pr_lib/pr_model.hpp"

#include <vector>

#include "dmp/Dmp.hpp"
#include "dmp/Gka.hpp"
#include "dmp/Trajectory.hpp"

#include "dynamicalsystems/DynamicalSystem.hpp"
#include "dynamicalsystems/ExponentialSystem.hpp"
#include "dynamicalsystems/SpringDamperSystem.hpp"

#include "functionapproximators/FunctionApproximatorLWR.hpp"
#include "functionapproximators/MetaParametersLWR.hpp"
#include "functionapproximators/ModelParametersLWR.hpp"

#include "dmpbbo_io/EigenFileIO.hpp"

#include <iostream>
#include <fstream>

#include "dmp/serialization.hpp"
#include "dynamicalsystems/serialization.hpp"
#include "functionapproximators/serialization.hpp"


using namespace std;
using namespace Eigen;
using namespace DmpBbo;

namespace pr_dmp
{
    class DmpRefGen : public rclcpp::Node
    {
        public:
            //PR_AUX_PUBLIC
            explicit DmpRefGen(const rclcpp::NodeOptions & options);
            ~DmpRefGen();

        protected:
            // Callback for the acceleration force
            void force_accel_callback(const pr_msgs::msg::PRArrayH::SharedPtr force_accel_msg);
            // Callback for the velocity force
            void force_vel_callback(const pr_msgs::msg::PRArrayH::SharedPtr force_vel_msg);
            // Callback for the joint position    
            void pos_callback(const pr_msgs::msg::PRArrayH::SharedPtr pos_msg);

        private:

            // PUBLISHERS AND SUBSCRIBERS
            // Subscriptor for the acceleration force for the DMP (will be added in the term of the acceleration)
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_force_accel_;
            // Subscriptor for the velocity force for the DMP (will be added in the term of the velocity)    
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_force_vel_;
            // Subscription to the joint position
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_pos_;
            // Publisher of joint position
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_q_;
            // Publisher of cart position (will be zeros if isCart and calcCart are both set to false)
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_x_;
            // Publisher of the GKA trajectory
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_gka_;
            // Publisher that ends the streaming
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_end_;
            
            // PARAMETERS
            
            // Trajectory file (cart or prism, indicated by the bool "isCart")
            // Where to save the DMP learned parameters
            string ref_path, save_directory;
            // Trajectory for the GKA
            string gka_path;
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
            bool ref_read = false, gka_read = false;
            // Number of inputs of the trajectory (columns)
            int n_dims;
            // First iteration of the trajectory
            bool first_iter = true;
            // Number of iterations
            int n_iter = 0;
            // When the task is finished, set end_task to true
            bool end_task = false;
            // Duration
            double tau;

            // DMP and GKA
            // Variable for the DMP and GKA
            Dmp* dmp;
            Gka* gka;
            Eigen::VectorXd damping_coefficient_vec;
            Eigen::VectorXd spring_constant_vec;
            Eigen::VectorXd mass_vec;
            // Forces to apply (coming from the callbacks)
            Vector4d force_accel = Vector4d::Zero();
            Vector4d force_vel = Vector4d::Zero();
            // Current phase of the (exponential) phase system
            VectorXd phase_current;
            // Limit phase so that the exponential system lasts as much as the trajectory file (without stop conditions)
            double phase_limit;
            // Initialization of the states and its derivatives
            VectorXd s, sd, s_updated;
            Vector4d s_pos;
            // Variables for GKA (phase variable and derivative)
            VectorXd x_gka;
            VectorXd xd_gka;
            // Output of the Gka
            VectorXd y_gka;
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
    };
}

#endif // PR_DMP__GEN_REF_HPP_