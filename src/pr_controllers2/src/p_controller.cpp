#include "pr_controllers2/p_controller.hpp"

namespace pr_controllers2
{
    /**** P CONTROLLER COMPONENT ****/
    PController::PController(const rclcpp::NodeOptions & options)
    : Node("pid_controller", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>("kp_gain", {27190.0, 27190.0, 27190.0, 361023.0});
        this->declare_parameter<double>("ts", 0.01);

        this->get_parameter("kp_gain", Kp);
        this->get_parameter("ts", ts);

        //Conversión, hacer función
        Kp_mat = Eigen::Matrix4d::Zero(4,4);
        for(int i=0; i<4; i++){
            Kp_mat(i,i) = Kp[i];
        }

        RCLCPP_INFO(this->get_logger(), "Creating communication");

        sub_ref = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "ref_pos",
            1,//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_sensor_data),
            std::bind(&PController::ref_callback, this, std::placeholders::_1)
        );

        sub_pos = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "pos",
            1,//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_sensor_data),
            std::bind(&PController::controller_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>("control_action", 1// rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_default)
        );

    }

    PController::~PController(){
    }

    void PController::ref_callback(const pr_msgs::msg::PRArrayH::SharedPtr ref_msg)
    {
        PRUtils::ArRMsg2Eigen(ref_msg, ref);
        init_ref = true;
    }

    void PController::controller_callback(const pr_msgs::msg::PRArrayH::SharedPtr pos_msg)
    {
        if (init_ref){
            // Control action message and init time
            auto control_action_msg = pr_msgs::msg::PRArrayH();
            control_action_msg.init_time = this->get_clock()->now();

            PRUtils::ArRMsg2Eigen(pos_msg, pos);

            e = pos-ref;

            //Calculate control action
            control_action = -Kp_mat*e;


            PRUtils::Eigen2ArMsg(control_action, control_action_msg);

            control_action_msg.header.stamp = pos_msg->header.stamp;
            control_action_msg.header.frame_id = pos_msg->header.frame_id;

            control_action_msg.current_time = this->get_clock()->now();
            publisher_->publish(control_action_msg);

            /*RCLCPP_INFO(this->get_logger(), "I heard: %f, %f, %f, %f", control_action_msg.data[0],
                                                                    control_action_msg.data[1],
                                                                    control_action_msg.data[2],
                                                                    control_action_msg.data[3]);
            */
        }
        
    }


}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_controllers2::PController)