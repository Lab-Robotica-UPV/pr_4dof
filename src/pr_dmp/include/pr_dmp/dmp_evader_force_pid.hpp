#ifndef PR_DMP__EVADER_FORCE_PID_HPP_
#define PR_DMP__EVADER_FORCE_PID_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_float_h.hpp"
#include "pr_msgs/msg/prots.hpp"
#include "pr_msgs/msg/pr_bool_h.hpp"

#include "pr_lib/pr_utils.hpp"
#include "pr_lib/pr_limits.hpp"
#include "pr_lib/pr_singularity.hpp"

#include "pr_lib/pr_singularity.hpp"

#include <vector>
#include <chrono>
#include <memory>
#include <utility>

#include "eigen3/Eigen/Dense"

namespace pr_dmp
{
    class EvaderForcePID : public rclcpp::Node
    {
        public:
            //PR_DMP_PUBLIC
            explicit EvaderForcePID(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback_ref(const pr_msgs::msg::PRArrayH::SharedPtr ref_msg);
            void topic_callback_x(const pr_msgs::msg::PRArrayH::SharedPtr x_msg);
            void topic_callback_ots_med(const pr_msgs::msg::PROTS::SharedPtr ots_med_msg);
            void topic_callback_jac_det_med(const pr_msgs::msg::PRFloatH::SharedPtr for_jac_det_med);

        private:

            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr ref_sub;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr x_sub;
            rclcpp::Subscription<pr_msgs::msg::PROTS>::SharedPtr ots_med_sub;
            rclcpp::Subscription<pr_msgs::msg::PRFloatH>::SharedPtr jac_det_med_sub;

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_action;
            rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr publisher_proportional;
            rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr publisher_derivative;
            rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr publisher_integral;
            rclcpp::Publisher<pr_msgs::msg::PRBoolH>::SharedPtr publisher_sing_pin;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_sing_pin_vector;
            
            // Initializing booleans
            bool init_x = false, init_ots_med = false, init_jac_det_med = false;

            // VALORES DE ENTRADA
            // Referencia medida
            Eigen::Vector4d q_ref = Eigen::Vector4d::Zero();
            // OTS medidos para la referencia
            Eigen::Matrix<double,6,4> OTS_med = Eigen::Matrix<double,6,4>::Zero();
            Eigen::Matrix<double,6,1> angOTS_med = Eigen::Matrix<double,6,1>::Zero();
            // Coordenadas cartesianas
            Eigen::Vector4d x_coord = Eigen::Vector4d::Zero();
            // Determinante del jacobiano
            double jac_det_med;

            // Matriz de incrementos para cada posible modificacion
            Eigen::MatrixXi minc_des;
            // Matriz que identifica las patas involucradas en el angulo OMEGA
	        Eigen::Matrix<int,6,2> mi_qind_des;
            //Vector para identificr los actuadores a modificar
            Eigen::Vector2i i_qind = Eigen::Vector2i::Zero();
            // Numero de combinaciones posibles
            int ncomb;
            // Posicion modificada
            Eigen::Vector4d q_ind_mod = Eigen::Vector4d::Zero();
            // Minimo angulo entre un par de ejes instantaneos de los OTS medidos y su indice
            double minAng_OTS_med;
            int index_minAng_OTS_med;
            // Indice del maximo angulo de par de ejes instantaneos de los OTS modificados
            double maxAng_OTS_mod;
            int index_maxAng_OTS_mod;

            // Limites de los actuadores
            Eigen::Matrix<double,4,2> Mlim_q_ind = Eigen::Matrix<double,4,2>::Zero();
            Eigen::Vector4d Vlim_angp = Eigen::Vector4d::Zero();
            
            // PARAMETROS
            // Parametros del robot
            std::vector<double> robot_params;
            // Parametros para los calculos
            int iterations=0, iter_max=30, iter_OTS;
            double tol, tol_OTS, ts, des_qind, lmin_Ang_OTS, lmin_FJac;


            // Avoid dividing by 0
            double epsilon = 1e-5;

            // Maximum force applicable
            double max_force;
            // Integrators
            double integrator_OTS=0, integrator_jac=0;
            // Values for  decay
            double decay;
            // Constants of PID Kp_evader, Kd_evader, Ki_evader
            double Kp_evader, Kd_evader, Ki_evader;
            double proportional_action_jac, derivative_action_jac, integral_action_jac, e_ant_jac;
            double proportional_action_OTS, derivative_action_OTS, integral_action_OTS, e_ant_OTS;
            // Prior measures
            double ang_OTS_med_ant, jac_med_ant;
            // Fuerza evasor
            Eigen::Vector4d force_OTS = Eigen::Vector4d::Zero();
            Eigen::Vector4d force_jac = Eigen::Vector4d::Zero(); 
            Eigen::Vector4d force_evader = Eigen::Vector4d::Zero(); 
            
            // Repulsive direction of the two legs provoking the singularity
            Eigen::Vector2i repulsive_dir;

            bool sing_pin = true; // If true, the robot is not in singular position
            Eigen::Vector4d sing_pin_vector = Eigen::Vector4d::Ones();

    };
}

#endif // PR_DMP__EVADER_FORCE_PID_HPP_