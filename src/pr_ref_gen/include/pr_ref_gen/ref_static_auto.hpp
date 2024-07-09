#ifndef PR_REF_GEN__REF_STATIC_AUTO_HPP_
#define PR_REF_GEN__REF_STATIC_AUTO_HPP_

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_mocap.hpp"

#include "std_msgs/msg/bool.hpp"

#include "eigen3/Eigen/Dense"

namespace pr_ref_gen
{
    class RefStaticAuto : public rclcpp::Node
    {
        public:
            //PR_REF_GEN_PUBLIC
            explicit RefStaticAuto(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback_encoder(const pr_msgs::msg::PRArrayH::SharedPtr q_msg);
            void topic_callback_cameras(const pr_msgs::msg::PRMocap::SharedPtr x_cam_msg);
            void external_stop_callback(const std_msgs::msg::Bool::SharedPtr external_stop_msg);

        private:
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_external_stop_;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_encoder;
            rclcpp::Subscription<pr_msgs::msg::PRMocap>::SharedPtr subscription_cameras;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_q;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_qini;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_x;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_f_ref;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_end_;
            std::vector<double> robot_params;
            std::string ref_path;
            
            Eigen::MatrixXd ref_matrix_x;
            //Eigen::MatrixXd ref_matrix_q;

            // Robot boolean (if true, is robot_5p)
            bool robot_5p;
            // Iteration for cameras and encodr
            int idx=0;
            // Number of samples
            static const int ndata_cam = 400;
            // Matrix to store readings of position and orientation
            Eigen::Matrix<double,ndata_cam,4> MX_init_estim;
            // Individual sample from cameras, initial and final position
            Eigen::Vector4d x_cam, x_ini;
            // Position arrays
            std::array<double,4> x_ini_arr;
            // Boolean to obey cameras or encoder
            bool read_encoder = false;
            // Boolean for external stop
            bool external_stop = false;
            // Sample time
            double ts;
          
            // Verificadores de factibilidad
            Eigen::Matrix<int,7,1> verif_ini;
            
            // Original samples for the trajectory
            int Nptos_o;
            // Samples to execute in the trajectory
            int Nptos_set;

            // Trayectoria despues del evasor
            Eigen::MatrixXd TrRobot_ns;
            // Trayectoria despues del evasor en espacio de juntas
            Eigen::MatrixXd Trqind_ns;
            
            // Variable for Inverse Kinematics
	        Eigen::Matrix<double,4,3> solq;
            
            // Limite del determinante y del angulo entre OTS
            //double lmin_Ang_OTS, lmin_FJac;

    };
}

#endif // PR_REF_GEN__REF_STATIC_AUTO_HPP_