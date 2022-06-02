#include "pr_controllers/gus_discretized_controller.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_lib/pr_utils.hpp"


namespace pr_controllers
{
    /**** GUS CONTROLLER COMPONENT ****/
    GusDiscretizedController::GusDiscretizedController(const rclcpp::NodeOptions & options)
    : Node("gus_discretized_controller", options)
    {
        //Parameter declaration
        this->declare_parameter<double>("kxc", 35.0);
        this->declare_parameter<double>("k2c", 40.0);
        this->declare_parameter<double>("ts", 0.01);

        this->declare_parameter<std::vector<double>>("initial_position", {0.679005, 0.708169, 0.684298, 0.637145});
        this->declare_parameter<std::vector<double>>("initial_reference", {0.679005, 0.708169, 0.684298, 0.637145});

        this->get_parameter("kxc", kxc);
        this->get_parameter("k2c", k2c);
        this->get_parameter("ts", ts);
        
        std::vector<double> init_pos, init_ref;

        this->get_parameter("initial_position", init_pos);
        this->get_parameter("initial_reference", init_ref);

        //Conversión, hacer función
        for(int i=0; i<4; i++){
            ref_ant(i) = init_ref[i];
        }

        RCLCPP_INFO(this->get_logger(), "Creating communication");

        //sub_ref.subscribe(this, "ref_pose");
        sub_pos.subscribe(this, "joint_position");
        sub_vel.subscribe(this, "joint_velocity");

        sub_ref = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "ref_pose",
            1,
            std::bind(&GusDiscretizedController::ref_callback, this, std::placeholders::_1)
        );

        sync_.reset(new Synchronizer(SyncPolicy(1), sub_pos, sub_vel));
        sync_->registerCallback(std::bind(&GusDiscretizedController::controller_callback, this, std::placeholders::_1, std::placeholders::_2));
        //sync_->setMaxIntervalDuration(rclcpp::Duration(0, 1000000));

        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>("control_action", 1);


    }

    void GusDiscretizedController::ref_callback(const pr_msgs::msg::PRArrayH::SharedPtr ref_msg)
    {
        PRUtils::ArRMsg2Eigen(ref_msg, ref);
        init_ref = true;
    }   

    void GusDiscretizedController::controller_callback(const pr_msgs::msg::PRArrayH::ConstPtr& pos_msg,
                                            const pr_msgs::msg::PRArrayH::ConstPtr& vel_msg)
    {
        if (init_ref){
            // Control action message and init time
            auto control_action_msg = pr_msgs::msg::PRArrayH();
            control_action_msg.init_time = this->get_clock()->now();
            //Conversión, crear función para esto:

            
            PRUtils::ArRMsg2Eigen(pos_msg, pos);
            PRUtils::ArRMsg2Eigen(vel_msg, vel);
            
            //Calculate control action
            e_x1 = ref-pos;
            derivada_qref = (ref-ref_ant)/ts;
            x_ez = derivada_qref+e_x1*kxc;
            e_x2 = x_ez-vel;
            derivada_x_ez = (x_ez-x_ez_ant)/ts;
            ca = derivada_x_ez+e_x2*k2c;
            
            //Update variables
            ref_ant = ref;
            x_ez_ant = x_ez;

            PRUtils::Eigen2ArMsg(ca, control_action_msg);

            control_action_msg.header.stamp = pos_msg->header.stamp;
            control_action_msg.header.frame_id = pos_msg->header.frame_id + ", " + vel_msg->header.frame_id;

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
RCLCPP_COMPONENTS_REGISTER_NODE(pr_controllers::GusDiscretizedController)