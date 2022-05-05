#include "pr_dmp/dmp_evader_force_pid.hpp"


using std::placeholders::_1;

namespace pr_dmp
{
    /**** EvaderForce COMPONENT ****/
    EvaderForcePID::EvaderForcePID(const rclcpp::NodeOptions & options)
    : Node("dmp_evader_force_pid", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>(
            "robot_config_params", 
            {0.4, 0.4, 0.4, 0.15, 90*(M_PI/180), 45*(M_PI/180), 0.3, 0.3, 0.3, 50*(M_PI/180), 90*(M_PI/180)});
        this->declare_parameter<int>("iter_fk",30);
        this->declare_parameter<double>("tol_fk",1e-7);
        this->declare_parameter<int>("iter_OTS",30);
        this->declare_parameter<double>("tol_OTS",1e-7);
        this->declare_parameter<double>("lmin_Ang_OTS",2.0);
        this->declare_parameter<double>("ts", 0.01);
        this->declare_parameter<double>("lmin_FJac", 0.015);
        this->declare_parameter<double>("max_force", 150.0);
        this->declare_parameter<double>("Kp_evader", 5.0);
        this->declare_parameter<double>("Kd_evader", 0.5);
        this->declare_parameter<double>("Ki_evader", 10.0);
        this->declare_parameter<double>("decay", 0.0025);

        this->get_parameter("robot_config_params", robot_params);
        this->get_parameter("iter_fk", iter_max);
        this->get_parameter("tol_fk", tol);
        this->get_parameter("iter_OTS", iter_OTS);
        this->get_parameter("tol_OTS", tol_OTS);
        this->get_parameter("lmin_Ang_OTS",lmin_Ang_OTS);
        this->get_parameter("ts", ts);
        this->get_parameter("lmin_FJac", lmin_FJac);
        this->get_parameter("max_force", max_force);
        this->get_parameter("Kp_evader", Kp_evader);
        this->get_parameter("Kd_evader", Kd_evader);
        this->get_parameter("Ki_evader", Ki_evader);
        this->get_parameter("decay", decay);

        // Matriz de incrementos
        minc_des.resize(2,8);
        minc_des << 1, -1, 1, -1, 1, -1, 0, 0,
		            1, -1, -1, 1, 0,  0, 1, -1;

        // Matriz que identifica las patas involucradas en el angulo OMEGA
        mi_qind_des << 0, 1,
				       0, 2,
				       0, 3,
				       1, 2,
				       1, 3,
				       2, 3;

        // Limites de los actuadores
        Mlim_q_ind = PRLimits::LimActuators();
        Vlim_angp = PRLimits::LimAngles();

        // Desplazamiento base para elegir dirección
        des_qind = 0.02*0.01;

        //Numero de posibles modificaciones
        ncomb = minc_des.cols();

        // Inicializar integradores y variables de control
        integrator_jac = 0;
        integrator_OTS = 0;
        ang_OTS_med_ant = lmin_Ang_OTS;
        jac_med_ant = lmin_FJac;
        
        ref_sub = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "ref_pose",
            1,
            std::bind(&EvaderForcePID::topic_callback_ref, this, std::placeholders::_1)
        );

        x_sub = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "x_coord",
            1,
            std::bind(&EvaderForcePID::topic_callback_x, this, std::placeholders::_1)
        );

        ots_med_sub = this->create_subscription<pr_msgs::msg::PROTS>(
            "ang_ots_med",
            1,
            std::bind(&EvaderForcePID::topic_callback_ots_med, this, std::placeholders::_1)
        );

        jac_det_med_sub = this->create_subscription<pr_msgs::msg::PRFloatH>(
            "for_jac_det_med",
            1,
            std::bind(&EvaderForcePID::topic_callback_jac_det_med, this, std::placeholders::_1)
        );

        publisher_action = this->create_publisher<pr_msgs::msg::PRArrayH>("evader_force", 1);
        publisher_proportional = this->create_publisher<pr_msgs::msg::PRFloatH>("evader_force_proportional", 1);
        publisher_derivative = this->create_publisher<pr_msgs::msg::PRFloatH>("evader_force_derivative", 1);
        publisher_integral = this->create_publisher<pr_msgs::msg::PRFloatH>("evader_force_integral", 1);

        publisher_sing_pin = this->create_publisher<pr_msgs::msg::PRBoolH>("sing_pin", 1);
        publisher_sing_pin_vector = this->create_publisher<pr_msgs::msg::PRArrayH>("sing_pin_vector", 1);
    }

    void EvaderForcePID::topic_callback_ref(const pr_msgs::msg::PRArrayH::SharedPtr ref_msg)
    {
        if (init_x && init_ots_med && init_jac_det_med){

            // Force evader total message
            auto force_evader_msg = pr_msgs::msg::PRArrayH();
            force_evader_msg.init_time = this->get_clock()->now();

            // Force evader proportional
            auto force_evader_proportional_msg = pr_msgs::msg::PRFloatH();
            force_evader_proportional_msg.init_time = this->get_clock()->now();

            // Force evader derivative
            auto force_evader_derivative_msg = pr_msgs::msg::PRFloatH();
            force_evader_derivative_msg.init_time = this->get_clock()->now();

            // Force evader integral
            auto force_evader_integral_msg = pr_msgs::msg::PRFloatH();
            force_evader_integral_msg.init_time = this->get_clock()->now();

            //Publish bool sing_pin
            auto sing_pin_msg = pr_msgs::msg::PRBoolH();
            sing_pin_msg.init_time = this->get_clock()->now();

            //Publish vector sing_pin_vector
            auto sing_pin_vector_msg = pr_msgs::msg::PRArrayH();
            sing_pin_vector_msg.init_time = this->get_clock()->now();

            // Convert the to Eigen
            for(int i=0;i<(int)ref_msg->data.size();i++) {
                q_ref(i) = ref_msg->data[i];
            }

            // Minimo angulo entre un par de ejes instantaneos de los OTS medidos
            minAng_OTS_med = angOTS_med.minCoeff(&index_minAng_OTS_med);

            // Angulos para todas las posibles modificaciones
            Eigen::VectorXd solAngOTS_mod;

            // Control actions for jacobian
            proportional_action_jac = 0.0; //Kp_evader*(1-abs(jac_det_med)/lmin_FJac);
            derivative_action_jac = 0.0; //Kd_evader*(-abs(jac_det_med)+jac_med_ant)/lmin_FJac/ts;
            e_ant_jac = 1 - jac_med_ant/lmin_FJac;
            integrator_jac = std::max(0.0, integrator_jac+ts*e_ant_jac);
            integral_action_jac = Ki_evader*integrator_jac;

            // Control actions for OTS
            proportional_action_OTS = 0.0; //Kp_evader*(1-minAng_OTS_med/lmin_Ang_OTS);
            derivative_action_OTS = 0.0; //Kd_evader*(-minAng_OTS_med+ang_OTS_med_ant)/lmin_Ang_OTS/ts;
            e_ant_OTS = 1 - ang_OTS_med_ant/lmin_Ang_OTS;
            integrator_OTS = std::max(0.0, integrator_OTS+ts*e_ant_OTS);
            integral_action_OTS = Ki_evader*integrator_OTS;


	        if (minAng_OTS_med<lmin_Ang_OTS || abs(jac_det_med) < lmin_FJac){
                // El PIN se pone a false
                sing_pin = false;
                sing_pin_vector = Eigen::Vector4d::Zero();

                // Identifico las patas que causan el minimo angulo de OTS
                i_qind = mi_qind_des.row(index_minAng_OTS_med);

                // MODIFICACIONES POSIBLES SOBRE LA REFERENCIA
                // Calculo de los angulos para todas las posibles modificaciones
                solAngOTS_mod = PRSingularity::PosiblesAngOTS(q_ref, x_coord, OTS_med, des_qind, i_qind, minc_des, 
				ncomb, tol, iter_max, iter_OTS, tol_OTS, robot_params, Mlim_q_ind, Vlim_angp);

                // REFERENCIA MODIFICADA QUE PRODUCIRÁ EL MAXIMO ANGULO OMEGA
                // Determino el maximo angulo OMEGA
                maxAng_OTS_mod = solAngOTS_mod.maxCoeff(&index_maxAng_OTS_mod);

                // Sentido (positivo o negativo) de la fuerza
                repulsive_dir = minc_des.col(index_maxAng_OTS_mod);

                // Fuerza debida al jacobiano (se normaliza referencia a 1)
                if (abs(jac_det_med) < lmin_FJac){
                    proportional_action_jac = Kp_evader*(1-abs(jac_det_med)/lmin_FJac);
                    derivative_action_jac = Kd_evader*(-abs(jac_det_med)+jac_med_ant)/lmin_FJac/ts;
                    force_jac(i_qind(0)) = repulsive_dir(0)*(proportional_action_jac+derivative_action_jac+integral_action_jac);
                    force_jac(i_qind(1)) = repulsive_dir(1)*(proportional_action_jac+derivative_action_jac+integral_action_jac);
                }

                // Fuerza debida al angulo OTS
                if (minAng_OTS_med < lmin_Ang_OTS){
                    proportional_action_OTS = Kp_evader*(1-minAng_OTS_med/lmin_Ang_OTS);
                    derivative_action_OTS = Kd_evader*(-minAng_OTS_med+ang_OTS_med_ant)/lmin_Ang_OTS/ts;
                    force_OTS(i_qind(0)) = repulsive_dir(0)*(proportional_action_OTS+derivative_action_OTS+integral_action_OTS);
                    force_OTS(i_qind(1)) = repulsive_dir(1)*(proportional_action_OTS+derivative_action_OTS+integral_action_OTS);
                }
                 
            }

            else{
                // El pin se pone a true
                sing_pin = true;
                sing_pin_vector=Eigen::Vector4d::Ones();
            }

            // Rebajar la fuerza del jacobiano
            if (abs(jac_det_med) >= lmin_FJac){
                // integrator_jac *= 0*exp(-decay);
                // force_jac *= 0*exp(-decay);
                for (int i=0; i<4; i++){
                    if (force_jac(i)>0)
                        force_jac(i) = integral_action_jac;
                    else if (force_jac(i)<0)
                        force_jac(i) = -integral_action_jac;
                }
            }

            // Rebajar la fuerza del OTS
            if (minAng_OTS_med >= lmin_Ang_OTS){
                // integrator_OTS *= 0*exp(-decay);
                // force_OTS *= 0*exp(-decay);
                for (int i=0; i<4; i++){
                    if (force_OTS(i)>0)
                        force_OTS(i) = integral_action_OTS;
                    else if (force_OTS(i)<0)
                        force_OTS(i) = -integral_action_OTS;
                }
            }

            for (int i=0; i<4; i++){
                if (force_jac(i) > max_force) force_jac(i) = max_force;
                if (force_jac(i) < -max_force) force_jac(i) = -max_force;
                if (force_OTS(i) > max_force) force_OTS(i) = max_force;
                if (force_OTS(i) < -max_force) force_OTS(i) = -max_force;
            }

            // Fuerza para enviar al DMP
            force_evader = force_jac + force_OTS;

            jac_med_ant = jac_det_med;
            ang_OTS_med_ant = minAng_OTS_med;


            for(int i=0;i<4;i++){
                force_evader_msg.data[i] = force_evader(i);
                sing_pin_vector_msg.data[i] = sing_pin_vector(i);
            }

            force_evader_proportional_msg.data = proportional_action_jac+proportional_action_OTS;
            force_evader_derivative_msg.data = derivative_action_jac+derivative_action_OTS;
            force_evader_integral_msg.data = integral_action_jac+integral_action_OTS;


            force_evader_msg.header.stamp = ref_msg->header.stamp;
            force_evader_msg.header.frame_id = ref_msg->header.frame_id;

            force_evader_proportional_msg.header.stamp = ref_msg->header.stamp;
            force_evader_proportional_msg.header.frame_id = ref_msg->header.frame_id;

            force_evader_derivative_msg.header.stamp = ref_msg->header.stamp;
            force_evader_derivative_msg.header.frame_id = ref_msg->header.frame_id;

            force_evader_integral_msg.header.stamp = ref_msg->header.stamp;
            force_evader_integral_msg.header.frame_id = ref_msg->header.frame_id;

            sing_pin_msg.data = sing_pin;
            sing_pin_msg.header.frame_id = ref_msg->header.frame_id;
            sing_pin_msg.header.stamp = ref_msg->header.stamp;

            sing_pin_vector_msg.header.frame_id = ref_msg->header.frame_id;
            sing_pin_vector_msg.header.stamp = ref_msg->header.stamp;

            force_evader_msg.current_time = this->get_clock()->now();
            publisher_action->publish(force_evader_msg);

            force_evader_proportional_msg.current_time = this->get_clock()->now();
            publisher_proportional->publish(force_evader_proportional_msg);

            force_evader_derivative_msg.current_time = this->get_clock()->now();
            publisher_derivative->publish(force_evader_derivative_msg);

            force_evader_integral_msg.current_time = this->get_clock()->now();
            publisher_integral->publish(force_evader_integral_msg);

            sing_pin_msg.current_time = this->get_clock()->now();
            publisher_sing_pin->publish(sing_pin_msg);

            sing_pin_vector_msg.current_time = this->get_clock()->now();
            publisher_sing_pin_vector->publish(sing_pin_vector_msg);

        }
    }

    void EvaderForcePID::topic_callback_x(const pr_msgs::msg::PRArrayH::SharedPtr x_msg)
    {
        init_x = true;
        PRUtils::ArRMsg2Eigen(x_msg, x_coord);
    }

    void EvaderForcePID::topic_callback_ots_med(const pr_msgs::msg::PROTS::SharedPtr ots_med_msg)
    {
        for(int i=0;i<(int)ots_med_msg->ots_ang.size();i++)
            angOTS_med(i) = ots_med_msg->ots_ang[i];

        for(int i=0; i<(int)ots_med_msg->ots.data.size(); i++) {
            int row = i/OTS_med.cols();
            int col = i%OTS_med.cols();
            OTS_med(row,col) = ots_med_msg->ots.data[i];
        }
        init_ots_med = true;
    }

    void EvaderForcePID::topic_callback_jac_det_med(const pr_msgs::msg::PRFloatH::SharedPtr for_jac_det_med)
    {
        jac_det_med = for_jac_det_med->data;
        init_jac_det_med = true;
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_dmp::EvaderForcePID)