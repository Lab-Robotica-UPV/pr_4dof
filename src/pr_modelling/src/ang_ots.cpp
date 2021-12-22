#include "pr_modelling/ang_ots.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_lib/pr_utils.hpp"

using std::placeholders::_1;

namespace pr_modelling
{
    /**** ANG OTS COMPONENT ****/
    AngOTS::AngOTS(const rclcpp::NodeOptions & options)
    : Node("ang_ots", options)
    {
        //Declare params
        
        this->declare_parameter<std::vector<double>>("robot_config_params", {0.4, 0.4, 0.4, 0.15, 90*(M_PI/180), 45*(M_PI/180), 0.3, 0.3, 0.3, 50*(M_PI/180), 90*(M_PI/180)});
        this->declare_parameter<int>("iter_max_ots", 30);
        this->declare_parameter<double>("tol_ots", 1e-7);
        this->declare_parameter<std::vector<double>>("initial_ots", {0.0, 0.0, 1.0, 0.0, 0.0, 1.0});
        this->declare_parameter<std::vector<double>>("initial_x_coord", {0.038310, 0.640040, 0.114319, 0.063530});

        this->get_parameter("robot_config_params", robot_params);
        this->get_parameter("initial_ots", initial_ots);
        this->get_parameter("iter_max_ots", iter_max_ots);
        this->get_parameter("tol_ots", tol_ots);
        this->get_parameter("initial_x_coord", initial_x_coord);

        for(int i=0; i<OTS.cols(); i++) {
            for(int j=0; j<OTS.rows(); j++)
                OTS(j,i) = initial_ots[j];
        }

        // OTS << 0.0582, 0.0692, 0.0673, 0.0683,
        //        0.5508, 0.1229, -0.2639, 0.2031,
        //        0.8326, 0.9900, 0.9622, 0.9768,
        //        0.1577, -0.1636, 0.3513, 0.1070,
        //        0, 0, 0, 0,
        //        0.0119, -0.0124, 0.0266, -0.0487;

        // std::vector to std::array
        std::copy(initial_x_coord.begin(), initial_x_coord.begin() + 4, initial_x_coord_array.begin());

        //Calculate inverse kinematics
        PRModel::InverseKinematics(q_sol, initial_x_coord_array, robot_params);

        //Calculate initial ots angles
        sol_OTS = PRSingularity::CalculateAngOts(initial_x_coord[2], initial_x_coord[3],
                                                 q_sol, OTS,
                                                 robot_params,
                                                 iter_max_ots, tol_ots);

        publisher_ = this->create_publisher<pr_msgs::msg::PROTS>(
            "ang_ots", 
            1);

        subscription_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "x_coord", 
            1, 
            std::bind(&AngOTS::topic_callback, this, _1));
    }

    void AngOTS::topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr x_msg)
    {

        //OTS msg and initial time
        auto ots_msg = pr_msgs::msg::PROTS();
        ots_msg.init_time = this->get_clock()->now();

        //Calculate inverse kinematics
        PRModel::InverseKinematics(q_sol, x_msg->data, robot_params);

        //Calculate ots angles
        sol_OTS = PRSingularity::CalculateAngOts(x_msg->data[2], x_msg->data[3],
                                                 q_sol, OTS,
                                                 robot_params,
                                                 iter_max_ots, tol_ots);        

        ots_msg.header.stamp = x_msg->header.stamp;

        for(int i=0; i<ots_msg.ots_ang.size(); i++)
            ots_msg.ots_ang[i] = sol_OTS(i);

        for(int i=0; i<OTS.rows(); i++)
        {
            for(int j=0; j<OTS.cols(); j++)
                ots_msg.ots.data.push_back(OTS(i,j));
        }
        ots_msg.ots.rows = OTS.rows();
        ots_msg.ots.cols = OTS.cols();

        ots_msg.header.frame_id = x_msg->header.frame_id;
        ots_msg.current_time = this->get_clock()->now();

        publisher_->publish(ots_msg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_modelling::AngOTS)