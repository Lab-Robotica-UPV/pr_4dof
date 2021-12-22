#include "pr_sing/sing_evader.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_lib/pr_utils.hpp"
#include "pr_lib/pr_limits.hpp"


namespace pr_sing
{
    /**** Singularity CONTROLLER COMPONENT ****/
    SingEvader::SingEvader(const rclcpp::NodeOptions & options)
    : Node("sing_evader", options)
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

        this->get_parameter("robot_config_params", robot_params);
        this->get_parameter("iter_fk", iter_max);
        this->get_parameter("tol_fk", tol);
        this->get_parameter("iter_OTS", iter_OTS);
        this->get_parameter("tol_OTS", tol_OTS);
        this->get_parameter("lmin_Ang_OTS",lmin_Ang_OTS);
        this->get_parameter("ts", ts);
        this->get_parameter("lmin_FJac", lmin_FJac);

        minc_des.resize(2,8);
        minc_des << 1, -1, 1, -1, 1, -1, 0, 0,
		            1, -1, -1, 1, 0,  0, 1, -1;

        des_qind = 0.01*ts;

        Mlim_q_ind = PRLimits::LimActuators();
        Vlim_angp = PRLimits::LimAngles();

        RCLCPP_INFO(this->get_logger(), "Creating communication");

        ref_sub = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "ref_pose",
            1,
            std::bind(&SingEvader::topic_callback_ref, this, std::placeholders::_1)
        );

        x_sub = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "x_coord",
            1,
            std::bind(&SingEvader::topic_callback_x, this, std::placeholders::_1)
        );

        ots_ref_sub = this->create_subscription<pr_msgs::msg::PROTS>(
            "ang_ots_ref",
            1,
            std::bind(&SingEvader::topic_callback_ots_ref, this, std::placeholders::_1)
        );

        ots_med_sub = this->create_subscription<pr_msgs::msg::PROTS>(
            "ang_ots_med",
            1,
            std::bind(&SingEvader::topic_callback_ots_med, this, std::placeholders::_1)
        );

        jac_det_ref_sub = this->create_subscription<pr_msgs::msg::PRFloatH>(
            "for_jac_det_ref",
            1,
            std::bind(&SingEvader::topic_callback_jac_det_ref, this, std::placeholders::_1)
        );

        jac_det_med_sub = this->create_subscription<pr_msgs::msg::PRFloatH>(
            "for_jac_det_med",
            1,
            std::bind(&SingEvader::topic_callback_jac_det_med, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>("ref_pose_mod", 1);

        publisher_vc_ = this->create_publisher<pr_msgs::msg::PRArrayH>("vc_des", 1);

        publisher_sing_pin = this->create_publisher<pr_msgs::msg::PRBoolH>("sing_pin", 1);


    }

    void SingEvader::topic_callback_ref(const pr_msgs::msg::PRArrayH::SharedPtr ref_msg)
    {
        if (init_x && init_ots_ref && init_ots_med && init_jac_det_ref && init_jac_det_med)
        {
            // Ref mod message and init time
            auto q_ref_mod_msg = pr_msgs::msg::PRArrayH();
            q_ref_mod_msg.init_time = this->get_clock()->now();
            //Publish also vc_des data
            auto vc_des_msg = pr_msgs::msg::PRArrayH();
            vc_des_msg.init_time = this->get_clock()->now();
            //Publish bool sing_pin
            auto sing_pin_msg = pr_msgs::msg::PRBoolH();
            sing_pin_msg.init_time = this->get_clock()->now();

            //Convert to Eigen
            for(int i=0;i<(int)ref_msg->data.size();i++) {
                q_ref(i) = ref_msg->data[i];
            }

            q_ind_mod = PRSingularity::CalculateQindModEvader(
                x_coord, q_ref, angOTS_ref, angOTS_med, OTS_med,
                minc_des, jac_det_med, jac_det_ref, robot_params, vc_des,
                Mlim_q_ind, Vlim_angp, des_qind, lmin_Ang_OTS,
                lmin_FJac,
                tol, iter_max,
                tol_OTS, iter_OTS
            );

            // sing_pin se activa si la referencia no se encuentra en una singularidad
            sing_pin = false;
            minAng_OTS_ref = angOTS_ref.minCoeff();
            if (minAng_OTS_ref >= lmin_Ang_OTS && abs(jac_det_ref)>=lmin_FJac) sing_pin = true;

            iterations++;
            //std::cout << iterations << " " << vc_des.transpose() << std::endl;

            for(int i=0;i<4;i++)
                q_ref_mod_msg.data[i] = q_ind_mod(i);

            q_ref_mod_msg.header.frame_id = ref_msg->header.frame_id;
            q_ref_mod_msg.header.stamp = ref_msg->header.stamp;

            for(int i=0;i<4;i++)
                vc_des_msg.data[i] = vc_des(i);

            vc_des_msg.header.frame_id = ref_msg->header.frame_id;
            vc_des_msg.header.stamp = ref_msg->header.stamp;

            sing_pin_msg.data = sing_pin;
            sing_pin_msg.header.frame_id = ref_msg->header.frame_id;
            sing_pin_msg.header.stamp = ref_msg->header.stamp;

            q_ref_mod_msg.current_time = this->get_clock()->now();
            publisher_->publish(q_ref_mod_msg);
            vc_des_msg.current_time = this->get_clock()->now();
            publisher_vc_->publish(vc_des_msg);
            sing_pin_msg.current_time = this->get_clock()->now();
            publisher_sing_pin->publish(sing_pin_msg);
        }
    }

    void SingEvader::topic_callback_x(const pr_msgs::msg::PRArrayH::SharedPtr x_msg)
    {
        init_x = true;
        PRUtils::ArRMsg2Eigen(x_msg, x_coord);
    }
    void SingEvader::topic_callback_ots_ref(const pr_msgs::msg::PROTS::SharedPtr ots_ref_msg)
    {
        for(int i=0;i<(int)ots_ref_msg->ots_ang.size();i++)
            angOTS_ref(i) = ots_ref_msg->ots_ang[i];
        init_ots_ref = true;
    }
    void SingEvader::topic_callback_ots_med(const pr_msgs::msg::PROTS::SharedPtr ots_med_msg)
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
    void SingEvader::topic_callback_jac_det_ref(const pr_msgs::msg::PRFloatH::SharedPtr for_jac_det_ref)
    {
        jac_det_ref = for_jac_det_ref->data;
        init_jac_det_ref = true;
    }
    void SingEvader::topic_callback_jac_det_med(const pr_msgs::msg::PRFloatH::SharedPtr for_jac_det_med)
    {
        jac_det_med = for_jac_det_med->data;
        init_jac_det_med = true;
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_sing::SingEvader)