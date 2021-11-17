#include "pr_ref_gen/ref_init_auto.hpp"

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
    /**** REFERENCE INIT AUTO GENERATOR COMPONENT ****/
    RefInitAuto::RefInitAuto(const rclcpp::NodeOptions & options)
    : Node("ref_init_auto", options)
    {
        //Parameter declaration
        this->declare_parameter<std::string>("ref_cart_path", 
            "/home/paralelo4dofnew/ros2_eloquent_ws/pr_4dof/references/ref_cart_TRR0_CF1_IdV1.txt");
        
        this->declare_parameter<std::vector<double>>("robot_config_params", 
            {0.4, 0.4, 0.4, 0.15, 90*(M_PI/180), 45*(M_PI/180), 0.3, 0.3, 0.3, 50*(M_PI/180), 90*(M_PI/180)});

        this->declare_parameter<double>("ts", 0.01);
        this->declare_parameter<double>("lmin_Ang_OTS",2.0);
        this->declare_parameter<double>("lmin_FJac", 0.015);

        this->get_parameter("ref_cart_path", ref_path);
        this->get_parameter("robot_config_params", robot_params);
        this->get_parameter("ts", ts);
        this->get_parameter("lmin_Ang_OTS",lmin_Ang_OTS);
        this->get_parameter("lmin_FJac", lmin_FJac);


        //Read file
        if(PRUtils::read_file(ref_matrix_x, ref_path)==-1){
             RCLCPP_ERROR(this->get_logger(), "Could not open file");
             return;
        }
        RCLCPP_INFO(this->get_logger(), "Pose references file opened");
        x_fin = ref_matrix_x.row(0);
        for (int i=0; i<4; i++){ x_fin_arr[i] = x_fin(i);}
        x_med_prev_vec.resize(4);

        //Vector of maximum velocities
        Xp_max << 0.01, 0.01, 2*M_PI/180, 2*M_PI/180;

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

        publisher_end_ = this->create_publisher<std_msgs::msg::Bool>(
            "end_flag",
            1
        );

        subscription_encoder = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "joint_position",
            1,
            std::bind(&RefInitAuto::topic_callback_encoder, this, _1)
        );

        subscription_cameras = this->create_subscription<pr_msgs::msg::PRMocap>(
            "x_coord_mocap",
            1,
            std::bind(&RefInitAuto::topic_callback_cameras, this, _1)
        );

        //for (int i=0; i< robot_params.size(); i++) std::cout << robot_params[i] << " " << std::endl;

        // // Prueba eliminar
        // //Eigen::Vector4d X_ini;
        // x_ini << 0.1, 0.79, -0.2793, -0.122;
        // Eigen::Vector4d X_fin
        // //Eigen::Vector4d Xp_tr_o;
        // Xp_tr_o << -0.0012, 0.0035, -0.0045, 0.0350;
        // Eigen::VectorXd v_tiempo_tramo;
        // v_tiempo_tramo = Eigen::VectorXd::LinSpaced(65, 0, 0.64);
        // double tiempo_tr_o = 0.64;

        // // Copiar a partir de aqui
        // // Calculo de trayectoria de forma discreta (Espacio de configuracion) sin parada
        // Eigen::MatrixXd TrRobot;
        // TrRobot = Eigen::MatrixXd::Ones(v_tiempo_tramo.size(),4).array().rowwise() * Xp_tr_o.transpose().array();
        // TrRobot = TrRobot.array().colwise() * v_tiempo_tramo.array();
        // TrRobot = TrRobot.rowwise() + x_ini.transpose();
        
        // // Añado las muestras para el punto final
        // Eigen::MatrixXd m_repit = Eigen::MatrixXd::Ones(int(tad/ts),4);
        // Eigen::MatrixXd TrRobot_o(TrRobot.rows()+m_repit.rows(), 4);
        // TrRobot_o.topRows(TrRobot.rows()) = TrRobot;
        // TrRobot_o.bottomRows(m_repit.rows()) = m_repit.array().rowwise() * TrRobot.row(TrRobot.rows()-1).array();
        
        // // Puntos originales de la trayectoria
        // Nptos_o = TrRobot_o.rows();
        // // Trayectoria original (Espacio de juntas)
        // Trqind_o = Eigen::MatrixXd::Zero(Nptos_o, 4);

        // // ANALISIS DE LA TRAYECTORIA CON EL EVASOR DE SINGULARIDADES
        // // Trayectoria despues del evasor
        // TrRobot_ns = Eigen::MatrixXd::Zero(Nptos_o, 4);
        // // Trayectoria despues del evasor (espacio de juntas)
        // Trqind_ns = Eigen::MatrixXd::Zero(Nptos_o, 4);
        // // Punto inicial del espacio cartesiano medido
        // Pos_carte_med = TrRobot_o.row(0);
        // // Punto inicial para la trayectoria no singular
        // TrRobot_ns.row(0) = TrRobot_o.row(0);
        // // Determinante del jacobiano directo para el punto inicial de las mediciones
        // detJD_med = detJD_ini;
        // // Cinematica inversa punto inicial de las mediciones 
        // PRModel::InverseKinematics(solq, x_ini, RParam);
        // // Almaceno el punto inicial de las trayectorias en espacio de juntas
        // Trqind_o.row(0) << solq(0,2), solq(1,2), solq(2,2), solq(3,1);
        // // Almaceno el punto inicial de las trayectorias en espacio de juntas (evasor)
        // Trqind_ns.row(0) << solq(0,2), solq(1,2), solq(2,2), solq(3,1);

        // x_ini << 0.1, 0.79, -0.2793, -0.122;
        // x_fin << 0.0, 0.66, 0.0, -0.0873;
        // for (int i=0; i<4; i++){ x_ini_arr[i] = x_ini(i); x_fin_arr[i] = x_fin(i);}

        // // GENERADOR DE TRAYECTORIA LINEAL

        //         // Determinante del punto inicial y de referencia y medicion
        //         Eigen::Matrix<double,4,4> ForJ_ini, ForJ_ref, ForJ_med;
        //         PRModel::ForwardJacobian(ForJ_ini, x_ini_arr, robot_params);
        //         double detJd_ini = ForJ_ini.determinant();

        //         // Determinante del punto final
        //         Eigen::Matrix<double,4,4> ForJ_fin;
        //         PRModel::ForwardJacobian(ForJ_fin, x_fin_arr, robot_params);
        //         double detJd_fin = ForJ_fin.determinant();

        //         // Verificamos que los puntos a unir tienen el mismo signo
        //         if (detJd_ini>=0 && detJd_fin>=0 || detJd_ini<0 && detJd_fin<0){
        //             // Verificadores de factilibidad punto inicial y final
        //             verif_ini = PRLimits::VerFactPos(x_ini_arr, robot_params);
        //             verif_fin = PRLimits::VerFactPos(x_fin_arr, robot_params);
        //             // Verifico que ambos puntos sean factibles
        //             if ((verif_ini.cwiseAbs()).sum()==0 && (verif_fin.cwiseAbs()).sum()==0){
        //                 // Puntos de los tramos de diseño;
        //                 Eigen::Matrix<double, 2,4> X_tr;
        //                 X_tr << x_ini.transpose(),
        //                         x_fin.transpose();
        //             }

        //             // Calculo del tiempo de la trayectoria
        //             if (x_ini.isApprox(x_fin)){
        //                 tiempo_tr_o = 1;
        //                 Xp_tr_o = Eigen::Vector4d::Zero();
        //             }
        //             else{
        //                 Eigen::Vector4d auxtiempos = (x_fin-x_ini).cwiseQuotient(Xp_max);
        //                 tiempo_tr_o = (auxtiempos.cwiseAbs()).maxCoeff();
        //                 tiempo_tr_o = int(tiempo_tr_o*100)/100.0;
        //                 Xp_tr_o = (x_fin-x_ini)/tiempo_tr_o;
        //             }
        //             // Vector de tiempo discretizado para el tramo
        //             v_tiempo_tramo = Eigen::VectorXd::LinSpaced(int(tiempo_tr_o/ts)+1,0,tiempo_tr_o);
        //             // Calculo de trayectoria de forma discreta (Espacio de configuracion) sin parada
        //             Eigen::MatrixXd TrRobot;
        //             TrRobot = Eigen::MatrixXd::Ones(v_tiempo_tramo.size(),4).array().rowwise() * Xp_tr_o.transpose().array();
        //             TrRobot = TrRobot.array().colwise() * v_tiempo_tramo.array();
        //             TrRobot = TrRobot.rowwise() + x_ini.transpose();
                    
        //             // Añado las muestras para el punto final
        //             Eigen::MatrixXd m_repit = Eigen::MatrixXd::Ones(int(tad/ts),4);
        //             Eigen::MatrixXd TrRobot_o(TrRobot.rows()+m_repit.rows(), 4);
        //             TrRobot_o.topRows(TrRobot.rows()) = TrRobot;
        //             TrRobot_o.bottomRows(m_repit.rows()) = m_repit.array().rowwise() * TrRobot.row(TrRobot.rows()-1).array();
                    
        //             // Puntos originales de la trayectoria
        //             Nptos_o = TrRobot_o.rows();
        //             // Trayectoria original (Espacio de juntas)
        //             Trqind_o = Eigen::MatrixXd::Zero(Nptos_o, 4);

        //             // ANALISIS DE LA TRAYECTORIA CON EL EVASOR DE SINGULARIDADES
        //             // Trayectoria despues del evasor
        //             TrRobot_ns = Eigen::MatrixXd::Zero(Nptos_o, 4);
        //             // Trayectoria despues del evasor (espacio de juntas)
        //             Trqind_ns = Eigen::MatrixXd::Zero(Nptos_o, 4);
        //             // Punto inicial del espacio cartesiano medido
        //             Pos_carte_med = TrRobot_o.row(0);
        //             // Punto inicial para la trayectoria no singular
        //             TrRobot_ns.row(0) = TrRobot_o.row(0);
        //             // Determinante del jacobiano directo para el punto inicial de las mediciones
        //             detJD_med = detJd_ini;
        //             // Cinematica inversa punto inicial de las mediciones 
        //             PRModel::InverseKinematics(solq, x_ini_arr, robot_params);
        //             // Almaceno el punto inicial de las trayectorias en espacio de juntas
        //             Trqind_o.row(0) << solq(0,2), solq(1,2), solq(2,2), solq(3,1);
        //             // Almaceno el punto inicial de las trayectorias en espacio de juntas (evasor)
        //             Trqind_ns.row(0) << solq(0,2), solq(1,2), solq(2,2), solq(3,1);
        //             // Angulos OTS para el punto inicial de las mediciones
        //             ang_OTS_med = PRSingularity::CalculateAngOts(x_ini(2), x_ini(3),
        //                                          solq, solOTS_ant_med,
        //                                          robot_params,
        //                                          iter_max_ots, tol_ots);

        //             // Bucle para el resto de la trayectoria
        //             for (int i=1;i<Nptos_o;i++){
        //                 // Jacobiano directo de la referencia
        //                 for (int j=0; j<4; j++) x_ref_arr[j] = TrRobot_o(i,j);
        //                 PRModel::ForwardJacobian(ForJ_ref, x_ref_arr, robot_params);
        //                 detJD_ref = ForJ_ref.determinant();
        //                 // Cinematica inversa de referencia
        //                 PRModel::InverseKinematics(solq, x_ref_arr, robot_params);
        //                 // Almaceno el punto para las trayectorias en espacio de juntas
        //                 Trqind_o.row(i) << solq(0,2), solq(1,2), solq(2,2), solq(3,1);
        //                 // Angulos OTS para las referencias
        //                 ang_OTS_ref = PRSingularity::CalculateAngOts(TrRobot_o(i,2), TrRobot_o(i,3),
        //                                          solq, solOTS_ant_ref,
        //                                          robot_params,
        //                                          iter_max_ots, tol_ots);
        //                 // Evasor de singularidades
        //                 q_ind_mod = PRSingularity::CalculateQindModEvader(
        //                     Pos_carte_med, Trqind_o.row(i), ang_OTS_ref, ang_OTS_med, solOTS_ant_med,
        //                     minc_des, detJD_med, detJD_ref, robot_params, vc_des,
        //                     PRLimits::LimActuators(), PRLimits::LimAngles(), des_qind, lmin_Ang_OTS,
        //                     lmin_FJac,
        //                     tol_ots, iter_max_ots,
        //                     tol_ots, iter_max_ots
        //                 );
        //                 // Almaceno el punto para las trayectorias en espacio de juntas despues del evasor
        //                 Trqind_ns.row(i) = q_ind_mod.transpose();
        //                 // Calculo de la posicion alcanzada por el robot (cinematica directa)
        //                 for (int j=0; j<4; j++){
        //                     q_ind_mod_arr[j] = q_ind_mod(j);
        //                     x_med_prev_vec[j] = Pos_carte_med(j);
        //                 }
        //                 x_med_arr = PRModel::ForwardKinematics(q_ind_mod_arr, x_med_prev_vec, robot_params, tol_ots, iter_max_ots);
        //                 TrRobot_ns.row(i) << x_med_arr[0], x_med_arr[1], x_med_arr[2], x_med_arr[3];
        //                 // Actualizo la posicion cartesiana medida
        //                 Pos_carte_med = TrRobot_ns.row(i);
        //                 // Determinante del jacobiano directo para las mediciones
        //                 PRModel::ForwardJacobian(ForJ_med, x_med_arr, robot_params);
        //                 detJD_med = ForJ_med.determinant();
        //                 // Cinematica inversa de las mediciones
        //                 PRModel::InverseKinematics(solq, x_med_arr, robot_params);
        //                 // Angulos OTS para las mediciones
        //                 ang_OTS_med = PRSingularity::CalculateAngOts(x_med_arr[2], x_med_arr[3],
        //                                          solq, solOTS_ant_med,
        //                                          robot_params,
        //                                          iter_max_ots, tol_ots);

        //             }
        //         }

        // std::cout << "Terminado" << std::endl;
        // std::cout << Trqind_ns << std::endl;
        // // Guardar contenido de las matrices
        // std::ofstream file("/home/paralelo4dofnew/ros2_eloquent_ws/pr_4dof/references/pruebaq.txt");
        // Eigen::IOFormat spaceFormat(Eigen::StreamPrecision, Eigen::DontAlignCols);//(4, 0, " ", "\n", "", "", "", "", "");
        // if (file.is_open())
        // {
        //     file << Trqind_ns.format(spaceFormat);
        //     file.close();
        // }

    }

    void RefInitAuto::topic_callback_encoder(const pr_msgs::msg::PRArrayH::SharedPtr q_msg)
    {
        if (read_encoder){
            if(idx<TrRobot_ns.rows())
            {
                //Ref message and init time
                auto ref_msg_q = pr_msgs::msg::PRArrayH();
                auto ref_msg_x = pr_msgs::msg::PRArrayH();
                auto ref_msg_qini = pr_msgs::msg::PRArrayH();

                ref_msg_q.init_time = this->get_clock()->now();
                ref_msg_x.init_time = this->get_clock()->now();
                ref_msg_qini.init_time = this->get_clock()->now();
                //CONVERTIR A FUNCIÓN
                for(int i=0; i<4; i++){
                    ref_msg_q.data[i] = Trqind_ns(idx, i);
                    ref_msg_x.data[i] = TrRobot_ns(idx, i);
                    ref_msg_qini.data[i] = Trqind_ns(0, i);
                }

                
                ref_msg_q.header.stamp = q_msg->header.stamp;
                ref_msg_q.header.frame_id = q_msg->header.frame_id;
                
                ref_msg_x.header.stamp = q_msg->header.stamp;
                ref_msg_x.header.frame_id = q_msg->header.frame_id;

                ref_msg_qini.header.stamp = q_msg->header.stamp;
                ref_msg_qini.header.frame_id = q_msg->header.frame_id;

                ref_msg_q.current_time = this->get_clock()->now();
                publisher_q->publish(ref_msg_q);
                ref_msg_x.current_time = this->get_clock()->now();
                publisher_x->publish(ref_msg_x);
                ref_msg_qini.current_time = this->get_clock()->now();
                publisher_qini->publish(ref_msg_qini);
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

    void RefInitAuto::topic_callback_cameras(const pr_msgs::msg::PRMocap::SharedPtr x_cam_msg)
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
                
            }    

            else{
                // Obtain the mean
                x_ini = MX_init_estim.colwise().mean();
                for (int i=0; i<4; i++) x_ini_arr[i] = x_ini(i);
                // Arrays

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
                if (detJd_ini>=0 && detJd_fin>=0 || detJd_ini<0 && detJd_fin<0){
                    // Verificadores de factilibidad punto inicial y final
                    verif_ini = PRLimits::VerFactPos(x_ini_arr, robot_params);
                    verif_fin = PRLimits::VerFactPos(x_fin_arr, robot_params);
                    // Verifico que ambos puntos sean factibles
                    if ((verif_ini.cwiseAbs()).sum()==0 && (verif_fin.cwiseAbs()).sum()==0){
                        // Puntos de los tramos de diseño;
                        Eigen::Matrix<double, 2,4> X_tr;
                        X_tr << x_ini.transpose(),
                                x_fin.transpose();
                    }

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

                idx = 0;
                read_encoder = true;
        
            } 

        }

    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_ref_gen::RefInitAuto)