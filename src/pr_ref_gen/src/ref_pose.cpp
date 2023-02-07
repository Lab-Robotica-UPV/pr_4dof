#include "pr_ref_gen/ref_pose.hpp"

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
#include "pr_lib/pr_utils.hpp"

using std::placeholders::_1;

namespace pr_ref_gen
{
    /**** REFERENCE POSE GENERATOR COMPONENT ****/
    RefPose::RefPose(const rclcpp::NodeOptions & options)
    : Node("ref_gen", options)
    {
        //Parameter declaration
        this->declare_parameter<std::string>("ref_path", 
            "/home/paralelo4dofnew/parallel_robot_ws/references/ref_cart_TRR0_CF1_IdV1.txt");
        
        this->declare_parameter<bool>("is_cart", true);
        this->declare_parameter<std::vector<double>>("robot_config_params", 
            {0.4, 0.4, 0.4, 0.15, 90*(M_PI/180), 45*(M_PI/180), 0.3, 0.3, 0.3, 50*(M_PI/180), 90*(M_PI/180)});

        this->get_parameter("ref_path", ref_path);
        this->get_parameter("is_cart", is_cart);
        this->get_parameter("robot_config_params", robot_params);

        if(is_cart)
        {

            //Read file
            if(PRUtils::read_file(ref_matrix_x, ref_path)==-1){
                RCLCPP_ERROR(this->get_logger(), "Could not open file");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Pose references file opened");
            n_ref = ref_matrix_x.rows();

            ref_matrix_q = Eigen::MatrixXd::Zero(n_ref, 4);
            
            //Cartesian to joint conversion (Inverse kinematics)
            for(int i=0; i<n_ref; i++)
            {
                Eigen::RowVector4d q_row;
                Eigen::RowVector4d x_row = ref_matrix_x.row(i);
                PRModel::InverseKinematicsPrism(q_row, x_row, robot_params);
                ref_matrix_q.row(i) = q_row; 
            }
        }

        else{
            //Read file
            if(PRUtils::read_file(ref_matrix_q, ref_path)==-1){
                RCLCPP_ERROR(this->get_logger(), "Could not open file");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Pose references file opened");
            n_ref = ref_matrix_q.rows();

            //In this case, ref_matrix_x have zeros and should not be used
            ref_matrix_x = Eigen::MatrixXd::Zero(n_ref, 4);
        }

        //Create communication
        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
            "ref_pose",
            1//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_default)
        );

        publisher_x_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
            "ref_pose_x",
            1//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_default)
        );

        publisher_end_ = this->create_publisher<std_msgs::msg::Bool>(
            "end_flag",
            1//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_default)
        );

        subscription_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "joint_position",
            1,//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_sensor_data),
            std::bind(&RefPose::topic_callback, this, _1)
        );

        subscription_external_stop_ = this->create_subscription<std_msgs::msg::Bool>(
            "external_stop",
            1,//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_sensor_data),
            std::bind(&RefPose::external_stop_callback, this, _1)
        );

        
    }

    void RefPose::topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr q_msg)
    {
        if ((idx<n_ref) && !external_stop)
        {
            //Ref message and init time
            auto ref_msg_q = pr_msgs::msg::PRArrayH();
            auto ref_msg_x = pr_msgs::msg::PRArrayH();

            ref_msg_q.init_time = this->get_clock()->now();
            ref_msg_x.init_time = this->get_clock()->now();
            //CONVERTIR A FUNCIÓN
            for(int i=0; i<4; i++){
                ref_msg_q.data[i] = ref_matrix_q(idx, i);
                ref_msg_x.data[i] = ref_matrix_x(idx, i);
            }

            
            ref_msg_q.header.stamp = q_msg->header.stamp;
            ref_msg_q.header.frame_id = q_msg->header.frame_id;
            
            ref_msg_x.header.stamp = q_msg->header.stamp;
            ref_msg_x.header.frame_id = q_msg->header.frame_id;

            ref_msg_q.current_time = this->get_clock()->now();
            publisher_->publish(ref_msg_q);
            ref_msg_x.current_time = this->get_clock()->now();
            publisher_x_->publish(ref_msg_x);
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

    void RefPose::external_stop_callback(const std_msgs::msg::Bool::SharedPtr external_stop_msg)
    {
        if (!external_stop)
            external_stop = external_stop_msg->data;
    }

}

    

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_ref_gen::RefPose)