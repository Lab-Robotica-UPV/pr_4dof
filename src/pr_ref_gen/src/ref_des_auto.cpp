#include "pr_ref_gen/ref_des_auto.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"

#include "pr_lib/pr_model.hpp"
#include "pr_lib/pr_limits.hpp"
#include "pr_lib/pr_utils.hpp"
#include "pr_lib/pr_singularity.hpp"

using std::placeholders::_1;

namespace pr_ref_gen
{
    /**** REFERENCE DISPLASAMENTS AUTO GENERATOR COMPONENT ****/
    RefDesAuto::RefDesAuto(const rclcpp::NodeOptions & options)
    : Node("ref_des_auto", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>("robot_config_params", 
            {0.4, 0.4, 0.4, 0.15, 90*(M_PI/180), 45*(M_PI/180), 0.3, 0.3, 0.3, 50*(M_PI/180), 90*(M_PI/180)});

        this->declare_parameter<double>("ts", 0.01);
        this->declare_parameter<double>("lmin_Ang_OTS",2.0);
        this->declare_parameter<double>("lmin_FJac", 0.015);
        this->declare_parameter<std::vector<double>>("des_x_set", {0.0, 0.0, 0.0, 0.0});
        this->declare_parameter<bool>("robot_5p", false);

        this->get_parameter("robot_config_params", robot_params);
        this->get_parameter("ts", ts);
        this->get_parameter("lmin_Ang_OTS",lmin_Ang_OTS);
        this->get_parameter("lmin_FJac", lmin_FJac);
        this->get_parameter("des_x_set", des_x_set);
        this->get_parameter("robot_5p", robot_5p);

        //Vector of maximum velocities
        Xp_max << 0.025, 0.025, 5*(M_PI/180), 5*(M_PI/180);

        //Condicion inicial para el contador de desviaciones

        //Initial condition for OTS
        solOTS_ant.row(2) = Eigen::Matrix<double, 1, 4>::Ones();
        solOTS_ant.row(5) = Eigen::Matrix<double, 1, 4>::Ones();
        solOTS_ant_ref = solOTS_ant;
        solOTS_ant_med = solOTS_ant;
        iter_max_ots = 30;
        tol_ots = 1.0e-7;

        // Matriz de incrementos para cada posible modificación
        minc_des.resize(2,8);
        minc_des << 1, -1, 1, -1, 1, -1, 0, 0,
		            1, -1, -1, 1, 0,  0, 1, -1;

        // Desplazamiento por cada 10 ms
        des_qind = 0.01*ts;

        //Create communication
        publisher_q = this->create_publisher<pr_msgs::msg::PRArrayH>(
            "ref_pose",
            1
        );

        publisher_x = this->create_publisher<pr_msgs::msg::PRArrayH>(
            "ref_pose_x",
            1
        );

        publisher_qini = this->create_publisher<pr_msgs::msg::PRArrayH>(
            "ref_pose_qini",
            1
        );

        publisher_f_ref = this->create_publisher<pr_msgs::msg::PRArrayH>(
            "ref_force",
            1
        );

        publisher_end_ = this->create_publisher<std_msgs::msg::Bool>(
            "end_flag",
            1
        );

        subscription_external_stop_ = this->create_subscription<std_msgs::msg::Bool>(
            "external_stop",
            1,//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_sensor_data),
            std::bind(&RefDesAuto::external_stop_callback, this, _1)
        );

        subscription_encoder = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "joint_position",
            1,
            std::bind(&RefDesAuto::topic_callback_encoder, this, _1)
        );

        subscription_cameras = this->create_subscription<pr_msgs::msg::PRMocap>(
            "x_coord_mocap",
            1,
            std::bind(&RefDesAuto::topic_callback_cameras, this, _1)
        );

    }

    void RefDesAuto::topic_callback_encoder(const pr_msgs::msg::PRArrayH::SharedPtr q_msg)
    {
        if (read_encoder){
            if((idx<TrRobot_ns.rows()) && !external_stop)
            {
                //Ref message and init time
                auto ref_msg_q = pr_msgs::msg::PRArrayH();
                auto ref_msg_x = pr_msgs::msg::PRArrayH();
                auto ref_msg_qini = pr_msgs::msg::PRArrayH();
                auto fref_msg = pr_msgs::msg::PRArrayH();

                ref_msg_q.init_time = this->get_clock()->now();
                ref_msg_x.init_time = this->get_clock()->now();
                ref_msg_qini.init_time = this->get_clock()->now();
                fref_msg.init_time = this->get_clock()->now();

                //CONVERTIR A FUNCIÓN
                for(int i=0; i<4; i++){
                    ref_msg_q.data[i] = Trqind_ns(idx, i);
                    ref_msg_x.data[i] = TrRobot_ns(idx, i);
                    ref_msg_qini.data[i] = Trqind_ns(0, i);
                    fref_msg.data[i] = 0;
                }

                
                ref_msg_q.header.stamp = q_msg->header.stamp;
                ref_msg_q.header.frame_id = q_msg->header.frame_id;
                
                ref_msg_x.header.stamp = q_msg->header.stamp;
                ref_msg_x.header.frame_id = q_msg->header.frame_id;

                ref_msg_qini.header.stamp = q_msg->header.stamp;
                ref_msg_qini.header.frame_id = q_msg->header.frame_id;

                fref_msg.header.stamp = q_msg->header.stamp;
                fref_msg.header.frame_id = q_msg->header.frame_id;

                ref_msg_q.current_time = this->get_clock()->now();
                publisher_q->publish(ref_msg_q);
                ref_msg_x.current_time = this->get_clock()->now();
                publisher_x->publish(ref_msg_x);
                ref_msg_qini.current_time = this->get_clock()->now();
                publisher_qini->publish(ref_msg_qini);

                fref_msg.current_time = this->get_clock()->now();
                publisher_f_ref->publish(fref_msg);
            }
            else
            {
                auto end_msg = std_msgs::msg::Bool();
                end_msg.data = true;
                publisher_end_->publish(end_msg);
            }

            idx++;
            std::cout << idx << std::endl;
        }
    }

    void RefDesAuto::topic_callback_cameras(const pr_msgs::msg::PRMocap::SharedPtr x_cam_msg)
    {

        if (!read_encoder){

            std::cout << idx << std::endl;

            if (idx<ndata_cam){

                //auto x_mocap_msg = pr_msgs::msg::PRArrayH();
                //x_mocap_msg.data = x_cam_msg->x_coord.data;

                for (int i=0; i<4; i++){
            
                    x_cam(i)=x_cam_msg->x_coord.data[i];
                    
                }

                MX_init_estim.row(idx) = x_cam.transpose();
                idx++;
                std::cout << "Mocap read " << idx << std::endl;
                
            }    

            else{
                // Obtain the mean for initial pose
                x_ini = MX_init_estim.colwise().mean();
                for (int i=0; i<4; i++) x_ini_arr[i] = x_ini(i);
                // Arrays

                //VERIFICATION PRINT "DELETE AT THE END"
                // std::cout << "*******************" << std::endl;
                // std::cout << "Xini " << x_ini(0) << "Zini " << x_ini(1)<< "Thini " << x_ini(2)<< "Psiini " << x_ini(3)<< std::endl;
                // std::cout << "*******************" << std::endl;

                //Final pose definition
                for (int i=0; i<4; i++){ 
                    if (i < 2){
                        x_fin(i)= x_ini(i) + des_x_set[i];
                    }
                    else{
                        x_fin(i)= x_ini(i) + (des_x_set[i]*(M_PI/180));
                    }

                    x_fin_arr[i] = x_fin(i);
                }
                x_med_prev_vec.resize(4);
                
                //VERIFICATION PRINT "DELETE AT THE END"
                // std::cout << "*******************" << std::endl;
                // std::cout << "Xfin " << x_fin(0) << "Zfin " << x_fin(1)<< "Thfin " << x_fin(2)<< "Psifin " << x_fin(3)<< std::endl;
                // std::cout << "*******************" << std::endl;

                // GENERADOR DE TRAYECTORIA LINEAL

                // Determinante del punto inicial y de referencia y medicion
                Eigen::Matrix<double,4,4> ForJ_ini, ForJ_ref, ForJ_med;
                PRModel::ForwardJacobian(ForJ_ini, x_ini_arr, robot_params);
                double detJd_ini = ForJ_ini.determinant();

                // Determinante del punto final
                Eigen::Matrix<double,4,4> ForJ_fin;
                PRModel::ForwardJacobian(ForJ_fin, x_fin_arr, robot_params);
                double detJd_fin = ForJ_fin.determinant();

                // Verificamos que los puntos a unir tienen el mismo signo
                if ((detJd_ini>=0 && detJd_fin>=0) || (detJd_ini<0 && detJd_fin<0)){
                    // Verificadores de factilibidad punto inicial y final
                    verif_ini = PRLimits::VerFactPos(x_ini_arr, robot_params, robot_5p);
                    verif_fin = PRLimits::VerFactPos(x_fin_arr, robot_params, robot_5p);
                    // Verifico que ambos puntos sean factibles
                    if (((verif_ini.cwiseAbs()).sum()==0) && ((verif_fin.cwiseAbs()).sum()==0)){
                        // Puntos de los tramos de diseño;
                        Eigen::Matrix<double, 2,4> X_tr;
                        X_tr << x_ini.transpose(),
                                x_fin.transpose();

                        // Calculo del tiempo de la trayectoria
                        if (x_ini.isApprox(x_fin)){
                            tiempo_tr_o = 1;
                            Xp_tr_o = Eigen::Vector4d::Zero();
                        }
                        else{
                            Eigen::Vector4d auxtiempos = (x_fin-x_ini).cwiseQuotient(Xp_max);
                            tiempo_tr_o = (auxtiempos.cwiseAbs()).maxCoeff();
                            tiempo_tr_o = int(tiempo_tr_o*100)/100.0;
                            Xp_tr_o = (x_fin-x_ini)/tiempo_tr_o;
                        }
                        // Vector de tiempo discretizado para el tramo
                        v_tiempo_tramo = Eigen::VectorXd::LinSpaced(int(tiempo_tr_o/ts)+1,0,tiempo_tr_o);
                        // Calculo de trayectoria de forma discreta (Espacio de configuracion) sin parada
                        Eigen::MatrixXd TrRobot;
                        TrRobot = Eigen::MatrixXd::Ones(v_tiempo_tramo.size(),4).array().rowwise() * Xp_tr_o.transpose().array();
                        TrRobot = TrRobot.array().colwise() * v_tiempo_tramo.array();
                        TrRobot = TrRobot.rowwise() + x_ini.transpose();
                        
                        // Añado las muestras para el punto final
                        Eigen::MatrixXd m_repit = Eigen::MatrixXd::Ones(int(tad/ts),4);
                        Eigen::MatrixXd TrRobot_o(TrRobot.rows()+m_repit.rows(), 4);
                        TrRobot_o.topRows(TrRobot.rows()) = TrRobot;
                        TrRobot_o.bottomRows(m_repit.rows()) = m_repit.array().rowwise() * TrRobot.row(TrRobot.rows()-1).array();
                        
                        // Puntos originales de la trayectoria
                        Nptos_o = TrRobot_o.rows();
                        // Trayectoria original (Espacio de juntas)
                        Trqind_o = Eigen::MatrixXd::Zero(Nptos_o, 4);

                        // ANALISIS DE LA TRAYECTORIA CON EL EVASOR DE SINGULARIDADES
                        // Trayectoria despues del evasor
                        TrRobot_ns = Eigen::MatrixXd::Zero(Nptos_o, 4);
                        // Trayectoria despues del evasor (espacio de juntas)
                        Trqind_ns = Eigen::MatrixXd::Zero(Nptos_o, 4);
                        // Punto inicial del espacio cartesiano medido
                        Pos_carte_med = TrRobot_o.row(0);
                        // Punto inicial para la trayectoria no singular
                        TrRobot_ns.row(0) = TrRobot_o.row(0);
                        // Determinante del jacobiano directo para el punto inicial de las mediciones
                        detJD_med = detJd_ini;
                        // Cinematica inversa punto inicial de las mediciones 
                        PRModel::InverseKinematics(solq, x_ini_arr, robot_params);
                        // Almaceno el punto inicial de las trayectorias en espacio de juntas
                        Trqind_o.row(0) << solq(0,2), solq(1,2), solq(2,2), solq(3,1);
                        // Almaceno el punto inicial de las trayectorias en espacio de juntas (evasor)
                        Trqind_ns.row(0) << solq(0,2), solq(1,2), solq(2,2), solq(3,1);
                        // Angulos OTS para el punto inicial de las mediciones
                        ang_OTS_med = PRSingularity::CalculateAngOts(x_ini(2), x_ini(3),
                                                    solq, solOTS_ant_med,
                                                    robot_params,
                                                    iter_max_ots, tol_ots);

                        // Bucle para el resto de la trayectoria
                        for (int i=1;i<Nptos_o;i++){
                            // Jacobiano directo de la referencia
                            for (int j=0; j<4; j++) x_ref_arr[j] = TrRobot_o(i,j);
                            PRModel::ForwardJacobian(ForJ_ref, x_ref_arr, robot_params);
                            detJD_ref = ForJ_ref.determinant();
                            // Cinematica inversa de referencia
                            PRModel::InverseKinematics(solq, x_ref_arr, robot_params);
                            // Almaceno el punto para las trayectorias en espacio de juntas
                            Trqind_o.row(i) << solq(0,2), solq(1,2), solq(2,2), solq(3,1);
                            // Angulos OTS para las referencias
                            ang_OTS_ref = PRSingularity::CalculateAngOts(TrRobot_o(i,2), TrRobot_o(i,3),
                                                    solq, solOTS_ant_ref,
                                                    robot_params,
                                                    iter_max_ots, tol_ots);
                            // Evasor de singularidades
                            q_ind_mod = PRSingularity::CalculateQindModEvader(
                                Pos_carte_med, Trqind_o.row(i), ang_OTS_ref, ang_OTS_med, solOTS_ant_med,
                                minc_des, detJD_med, detJD_ref, robot_params, vc_des,
                                PRLimits::LimActuators(), PRLimits::LimAngles(), des_qind, lmin_Ang_OTS,
                                lmin_FJac,
                                tol_ots, iter_max_ots,
                                tol_ots, iter_max_ots
                            );
                            // Almaceno el punto para las trayectorias en espacio de juntas despues del evasor
                            Trqind_ns.row(i) = q_ind_mod.transpose();
                            // Calculo de la posicion alcanzada por el robot (cinematica directa)
                            for (int j=0; j<4; j++){
                                q_ind_mod_arr[j] = q_ind_mod(j);
                                x_med_prev_vec[j] = Pos_carte_med(j);
                            }
                            x_med_arr = PRModel::ForwardKinematics(q_ind_mod_arr, x_med_prev_vec, robot_params, tol_ots, iter_max_ots);
                            TrRobot_ns.row(i) << x_med_arr[0], x_med_arr[1], x_med_arr[2], x_med_arr[3];
                            // Actualizo la posicion cartesiana medida
                            Pos_carte_med = TrRobot_ns.row(i);
                            // Determinante del jacobiano directo para las mediciones
                            PRModel::ForwardJacobian(ForJ_med, x_med_arr, robot_params);
                            detJD_med = ForJ_med.determinant();
                            // Cinematica inversa de las mediciones
                            PRModel::InverseKinematics(solq, x_med_arr, robot_params);
                            // Angulos OTS para las mediciones
                            ang_OTS_med = PRSingularity::CalculateAngOts(x_med_arr[2], x_med_arr[3],
                                                    solq, solOTS_ant_med,
                                                    robot_params,
                                                    iter_max_ots, tol_ots);

                        }
                    }
                    else{
                        if ((verif_ini.cwiseAbs()).sum()!=0)
                            std::cout << "El punto inicial no es fisicamente alcanzable: " << verif_ini.transpose() << ", x ini: " << x_ini.transpose() << std::endl;
                        if ((verif_fin.cwiseAbs()).sum()!=0)
                            std::cout << "El punto final no es fisicamente alcanzable: " << verif_fin.transpose() << ", x fin: " << x_fin.transpose() << std::endl;
                    }
                }
                else{
                    std::cout << "No se puede conectar sin pasar por la singularidad" << std::endl;
                }

                idx = 0;
                read_encoder = true;
        
            } 

        }

    }

    void RefDesAuto::external_stop_callback(const std_msgs::msg::Bool::SharedPtr external_stop_msg)
    {
        if (!external_stop)
            external_stop = external_stop_msg->data;

    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_ref_gen::RefDesAuto)