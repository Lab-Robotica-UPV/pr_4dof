#ifndef PR_MODELLING_ADAPTIVE_RBFG_NN_HPP_
#define PR_MODELLING_ADAPTIVE_RBFG_NN_HPP_

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_mat_h.hpp"
#include "pr_lib/pr_model.hpp"
#include "pr_lib/pr_utils.hpp"

#include "eigen3/Eigen/Dense"

namespace pr_modelling
{
    class AdaptiveRBFGNN : public rclcpp::Node
    {
        public:
            //PR_MODELLING_PUBLIC
            explicit AdaptiveRBFGNN(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback(const pr_msgs::msg::PRArrayH::ConstPtr& q_ref_msg,
                                const pr_msgs::msg::PRArrayH::ConstPtr& q_mea_msg);

        private:
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_q_ref;
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_q_mea;

            typedef message_filters::sync_policies::ApproximateTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH> SyncPolicy;

            typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
            std::shared_ptr<Synchronizer> sync_;

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            
            //*************************
            //Variables definition

            // Boolean to activate printings for debugging
            bool print_debug = false;

            // Number of Hidden neural network layers
            int N_Hlayers;
            // Center points of the Gaussian function
            std::vector<double> c_festo;
            std::vector<double> c_niasa;

            // Center points of the Gaussian function
            Eigen::VectorXd c_festo_mat;
            Eigen::VectorXd c_niasa_mat;

            //Joint position reference and feedback
            Eigen::Vector4d q_ref, q_mea;
            
            //Control actions for the adaptive law based on neural network
            Eigen::Vector4d adaptive_actions;

            //Matrix for center points of the Gaussian function
            double c_gain = 100.0;
            Eigen::MatrixXd c;
            
            //Width value
            double b = 1.0;
            //Eigen::Matrix<double,8,4> B = Eigen::Matrix<double,8,4>::Zero();
            Eigen::Matrix<double,8,4> B;

            //Matrix P
            Eigen::Matrix<double,8,8> P;

            //gama value
            double gama_rbf = 100.0;

            //Sample time
            double ts;

            //Error varible
            Eigen::Vector4d e, e_ant, d_e;

            //xi vector
            Eigen::Matrix<double,8,1> xi = Eigen::Matrix<double,8,1>::Zero();

            //h vector
            Eigen::MatrixXd h;

            //dw matrix
            Eigen::MatrixXd aux_dW, dW, auxX;

            // Adaptive vector
            Eigen::VectorXd y, y_ant, dy, dy_ant;
    };
}

#endif // PR_MODELLING_ADAPTIVE_RBFG_NN_HPP_