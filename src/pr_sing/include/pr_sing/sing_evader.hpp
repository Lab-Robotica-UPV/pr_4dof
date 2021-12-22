#ifndef PR_SING__SING_EVADER_HPP_
#define PR_SING__SING_EVADER_HPP_

#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"


#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/prots.hpp"
#include "pr_msgs/msg/pr_float_h.hpp"
#include "pr_msgs/msg/pr_bool_h.hpp"

#include "pr_lib/pr_utils.hpp"

#include "pr_lib/pr_singularity.hpp"


namespace pr_sing
{
    class SingEvader : public rclcpp::Node
    {
        public:
            //PR_SING_PUBLIC
            explicit SingEvader(const rclcpp::NodeOptions & options);

        protected:

            void topic_callback_ref(const pr_msgs::msg::PRArrayH::SharedPtr ref_msg);
            void topic_callback_x(const pr_msgs::msg::PRArrayH::SharedPtr x_msg);
            void topic_callback_ots_ref(const pr_msgs::msg::PROTS::SharedPtr ots_ref_msg);
            void topic_callback_ots_med(const pr_msgs::msg::PROTS::SharedPtr ots_med_msg);
            void topic_callback_jac_det_ref(const pr_msgs::msg::PRFloatH::SharedPtr for_jac_det_ref);
            void topic_callback_jac_det_med(const pr_msgs::msg::PRFloatH::SharedPtr for_jac_det_med);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr ref_sub;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr x_sub;
            rclcpp::Subscription<pr_msgs::msg::PROTS>::SharedPtr ots_ref_sub;
            rclcpp::Subscription<pr_msgs::msg::PROTS>::SharedPtr ots_med_sub;
            rclcpp::Subscription<pr_msgs::msg::PRFloatH>::SharedPtr jac_det_ref_sub;
            rclcpp::Subscription<pr_msgs::msg::PRFloatH>::SharedPtr jac_det_med_sub;

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_vc_;
            rclcpp::Publisher<pr_msgs::msg::PRBoolH>::SharedPtr publisher_sing_pin;

            Eigen::Matrix<double,6,4> OTS_med = Eigen::Matrix<double,6,4>::Zero();
            Eigen::Matrix<double,6,1> angOTS_ref = Eigen::Matrix<double,6,1>::Zero();
            Eigen::Matrix<double,6,1> angOTS_med = Eigen::Matrix<double,6,1>::Zero();
            Eigen::Vector4d x_coord = Eigen::Vector4d::Zero();
            double jac_det_ref, jac_det_med;

            Eigen::MatrixXi minc_des;
            Eigen::Vector4i vc_des = Eigen::Vector4i::Zero();
            Eigen::Vector4d q_ind_mod = Eigen::Vector4d::Zero();

            Eigen::Vector4d q_ref = Eigen::Vector4d::Zero();
            Eigen::Matrix<double,4,2> Mlim_q_ind = Eigen::Matrix<double,4,2>::Zero();
            Eigen::Vector4d Vlim_angp = Eigen::Vector4d::Zero();
            
            std::vector<double> robot_params;
            int iterations=0, iter_max=30, ncomb, iter_OTS;
            double tol, tol_OTS, ts, des_qind, lmin_Ang_OTS, lmin_FJac;
            double minAng_OTS_ref;
            bool sing_pin = true; // If true, the robot is not in singular position

            // Initializing booleans
            bool init_x = false, init_ots_ref = false, init_ots_med = false, init_jac_det_ref = false, init_jac_det_med = false;

    };
}

#endif // PR_SING__SING_EVADER_HPP_