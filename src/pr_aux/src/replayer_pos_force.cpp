#include "pr_aux/replayer_pos_force.hpp"

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

namespace pr_aux
{
    /**** REPLAYER DATA GENERATOR COMPONENT ****/
    ReplayerPosForce::ReplayerPosForce(const rclcpp::NodeOptions & options)
    : Node("replayer", options)
    {
        //Parameter declaration
        this->declare_parameter<float>("ts_ms", 10.0);
        this->declare_parameter<std::string>("data_path_pos", 
            "/home/paralelo4dofnew/ros2_eloquent_ws/pr_4dof/replay/med_qinde_TRR18_CF1_V1.txt");
        this->declare_parameter<std::string>("data_path_force", 
            "/home/paralelo4dofnew/ros2_eloquent_ws/pr_4dof/replay/med_Fent_TRR18_CF1_V1.txt");
        
        this->get_parameter("ts_ms", ts_ms);
        this->get_parameter("data_path_pos", data_path_pos);
        this->get_parameter("data_path_force", data_path_force);

        //Read pos file
        std::cout << "About to read the pos file" << std::endl;
        if(PRUtils::read_file(data_matrix_pos, data_path_pos)==-1){
            RCLCPP_ERROR(this->get_logger(), "Could not open pos file");
            std::cout << "Could not read the pos file" << std::endl;
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Pos data file opened");
        n_data = data_matrix_pos.rows();

        //Read force file
        std::cout << "About to read the force file" << std::endl;
        if(PRUtils::read_file(data_matrix_force, data_path_force)==-1){
            RCLCPP_ERROR(this->get_logger(), "Could not open force file");
            std::cout << "Could not read the force file" << std::endl;
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Force data file file opened");

        //Create communication
        publisher_pos = this->create_publisher<pr_msgs::msg::PRArrayH>(
            "data_pos",
            1
        );
        publisher_force = this->create_publisher<pr_msgs::msg::PRForceState>(
            "data_force",
            1
        );

        //End signal subscription
        subscription_end_ = this->create_subscription<std_msgs::msg::Bool>(
            "end_flag",
            1,
            std::bind(&ReplayerPosForce::end_callback, this, _1));

        // Pause before start
		std::this_thread::sleep_for(std::chrono::milliseconds(10000));

        //Create timer
        timer_ = this->create_wall_timer(
            std::chrono::duration<float, std::milli>(ts_ms), 
            std::bind(&ReplayerPosForce::on_timer, this));

        
    }

    void ReplayerPosForce::on_timer()
    {
        if(!is_finished) {
        
            // Pos data msg and init time
            auto pos_data_msg = pr_msgs::msg::PRArrayH();
            pos_data_msg.init_time = this->get_clock()->now();

            // Force data msg and init time
            auto force_data_msg = pr_msgs::msg::PRForceState();
            force_data_msg.init_time = this->get_clock()->now();

            //CONVERTIR A FUNCIÓN
            for(int i=0; i<4; i++)
                pos_data_msg.data[i] = data_matrix_pos(idx, i);
            
            force_data_msg.force[0] = data_matrix_force(idx,0);
            force_data_msg.force[1] = 0;
            force_data_msg.force[2] = data_matrix_force(idx,1);
            force_data_msg.momentum[0] = 0;
            force_data_msg.momentum[1] = data_matrix_force(idx,2);
            force_data_msg.momentum[2] = data_matrix_force(idx,3);

            pos_data_msg.header.frame_id = std::to_string(idx);
            force_data_msg.header.frame_id = std::to_string(idx);

            if(idx<n_data-1)
                idx++;

            pos_data_msg.current_time = this->get_clock()->now();
            pos_data_msg.header.stamp = pos_data_msg.current_time;
            publisher_pos->publish(pos_data_msg);

            force_data_msg.header.stamp = pos_data_msg.current_time;
            publisher_force->publish(force_data_msg); 

        } else {
            //End flag activted
			RCLCPP_INFO(this->get_logger(), "Experiment finished");
		}
	}

    void ReplayerPosForce::end_callback(const std_msgs::msg::Bool::SharedPtr end_msg)
    {
        //End callback looking for the final event
        if(end_msg->data)
			is_finished = true;
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_aux::ReplayerPosForce)