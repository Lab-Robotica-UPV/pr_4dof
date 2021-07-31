#ifndef PR_REF_GEN__REF_INIT_AUTO_HPP_
#define PR_REF_GEN__REF_INIT_AUTO_HPP_

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_mocap.hpp"

#include "std_msgs/msg/bool.hpp"

#include "eigen3/Eigen/Dense"

namespace pr_ref_gen
{
    class RefInitAuto : public rclcpp::Node
    {
        public:
            //PR_REF_GEN_PUBLIC
            explicit RefInitAuto(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback_encoder(const pr_msgs::msg::PRArrayH::SharedPtr q_msg);
            void topic_callback_cameras(const pr_msgs::msg::PRMocap::SharedPtr x_cam_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_encoder;
            rclcpp::Subscription<pr_msgs::msg::PRMocap>::SharedPtr subscription_cameras;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_q;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_x;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_end_;
            std::vector<double> robot_params;
            std::string ref_path;
            
            Eigen::MatrixXd ref_matrix_x;
            //Eigen::MatrixXd ref_matrix_q;


            // Iteration for cameras and encodr
            int idx=0;
            // Number of samples
            static const int ndata_cam = 400;
            // Matrix to store readings of position and orientation
            Eigen::Matrix<double,ndata_cam,4> MX_init_estim;
            // Individual sample from cameras, initial and final position
            Eigen::Vector4d x_cam, x_ini, x_fin;
            // Position arrays
            std::array<double,4> x_ini_arr, x_fin_arr;
            // Boolean to obey cameras or encoder
            bool read_encoder = false;
            // Sample time
            double ts;
            // Vector of maximum velocities in cartesian coordinates
            Eigen::Vector4d Xp_max = Eigen::Vector4d::Zero();

            // VARIABLES FOR SINGULARITIES
            // Condicion inicial para el contador de desviaciones
            Eigen::Vector4d vc_des_ant = Eigen::Vector4d::Zero();
            // Condicion inicial para resolver los OTS
            Eigen::Matrix<double,6,4> solOTS_ant = Eigen::Matrix<double,6,4>::Zero();
            // Idem para referencias y medicion
            Eigen::Matrix<double,6,4> solOTS_ant_ref;
            Eigen::Matrix<double,6,4> solOTS_ant_med;
            // Verificadores de factibilidad
            Eigen::Matrix<int,7,1> verif_ini, verif_fin;
            // Tiempo del experimento
            double tiempo_tr_o;
            // Actual velocity for each leg
            Eigen::Vector4d Xp_tr_o;
            // Vector de tiempo discretizado para el tramo
            Eigen::VectorXd v_tiempo_tramo;
            // Tiempo adicional para mantener la referencia fija al final
            double tad=3.0;
            // Puntos originales de la trayectoria
            int Nptos_o;
            // Trayectoria original (Espacio de juntas)
            Eigen::MatrixXd Trqind_o;
            // Trayectoria despues del evasor
            Eigen::MatrixXd TrRobot_ns;
            // Trayectoria despues del evasor en espacio de juntas
            Eigen::MatrixXd Trqind_ns;
            // Posicion medida en el espacio cartesiano
            Eigen::Vector4d Pos_carte_med;
            // Determinante del jacobiano ddirecto
            double detJD_med;
            // Variable for Inverse Kinematics
	        Eigen::Matrix<double,4,3> solq;

    };
}

#endif // PR_REF_GEN__REF_INIT_AUTO_HPP_