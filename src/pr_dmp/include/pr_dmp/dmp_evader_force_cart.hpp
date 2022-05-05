#ifndef PR_DMP__EVADER_FORCE_CART_HPP_
#define PR_DMP__EVADER_FORCE_CART_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_mat_h.hpp"
#include "pr_lib/pr_utils.hpp"

#include <vector>
#include <chrono>
#include <memory>
#include <utility>

#include "eigen3/Eigen/Dense"

namespace pr_dmp
{
    class EvaderForceCart : public rclcpp::Node
    {
        public:
            //PR_DMP_PUBLIC
            explicit EvaderForceCart(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback_jac(const pr_msgs::msg::PRMatH::SharedPtr jacobian_dir_msg);
            void topic_callback_force_evader_prism(const pr_msgs::msg::PRArrayH::SharedPtr force_evader_prism_msg);

        private:

            rclcpp::Subscription<pr_msgs::msg::PRMatH>::SharedPtr jacobian_dir_sub;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr force_evader_prism_sub;

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            
            // Initializing booleans
            bool init_jac_dir = false;

            // VALORES DE ENTRADA Y SALIDA
            // Fuerza de salida (cartesiana)
            Eigen::Vector4d force_evader_cart = Eigen::Vector4d::Zero();
            // Fuerza de entrada (prismatica)
            Eigen::Vector4d force_evader_prism;
            // Jacobiano directo
            Eigen::Matrix<double, 4, 4> JDir;

            // PARAMETRO de ganancia
            std::vector<double> gain;

    };
}

#endif // PR_DMP__EVADER_FORCE_CART_HPP_