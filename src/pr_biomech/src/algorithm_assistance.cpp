#include "pr_biomech/algorithm_assistance.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace pr_biomech
{
    /**** TEST COMPONENT ****/
    AlgorithmAssistance::AlgorithmAssistance(const rclcpp::NodeOptions & options)
    : Node("algorithm_assistance", options)
    {

        //Parameter declaration
        this->declare_parameter<double>("ts", 0.01);
        this->declare_parameter<double>("kp", 1);
        this->declare_parameter<double>("ki", 1);
        this->declare_parameter<double>("kd", 1);

        this->get_parameter("ts", ts);
        this->get_parameter("kp", kp);
        this->get_parameter("ki", ki);
        this->get_parameter("kd", kd);

        // Subscriptor of force state
        sub_force_state = this->create_subscription<pr_msgs::msg::PRForceState>(
            "force_state",
            1,
            std::bind(&AlgorithmAssistance::force_callback, this, _1)
        );

        // Subscriptor of gen force knee
        sub_gen_force_knee = this->create_subscription<pr_msgs::msg::PRFloatH>(
            "gen_force_knee",
            1,
            std::bind(&AlgorithmAssistance::gen_force_knee_callback, this, _1)
        );

        // Subscriptor of muscle force
       sub_muscle_force = this->create_subscription<pr_msgs::msg::PRFloatH>(
            "muscle_force",
            1,
            std::bind(&AlgorithmAssistance::muscle_force_callback, this, _1)
        );

        // Subscriptor of ref of the muscle force
        sub_ref_muscle_force = this->create_subscription<pr_msgs::msg::PRFloatH>(
            "ref_muscle_force",
            1,
            std::bind(&AlgorithmAssistance::ref_muscle_force_callback, this, _1)
        );

        // Subscriptor of muscle dir
        sub_muscle_dir = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "muscle_dir",
            1,
            std::bind(&AlgorithmAssistance::muscle_dir_callback, this, _1)
        );

        pub_ass_force = this->create_publisher<pr_msgs::msg::PRArrayH>("ass_force", 1);
        //publisher_F_opt_ref = this->create_publisher<pr_msgs::msg::PRArrayH>("f_opt_ref", 1);

    }

    void AlgorithmAssistance::force_callback(const pr_msgs::msg::PRForceState::SharedPtr force_state_msg){
        force_state[0] = force_state_msg->force[0];
        force_state[1] = force_state_msg->force[2];
        force_state[2] = force_state_msg->momentum[1];
        force_state[3] = force_state_msg->momentum[2];
        read_force_state = true;

        if (read_muscle_force && read_ref_muscle_force && read_muscle_dir){
            auto ass_force_msg = pr_msgs::msg::PRArrayH();
            ass_force_msg.init_time = this->get_clock()->now();

            // Here goes the algorithm


            PRUtils::Eigen2ArMsg(ass_force, ass_force_msg);

            ass_force_msg.header.stamp = force_state_msg->header.stamp;
            ass_force_msg.header.frame_id = force_state_msg->header.frame_id;

            ass_force_msg.current_time = this->get_clock()->now();
            pub_ass_force->publish(ass_force_msg);
        }
    }

    void AlgorithmAssistance::gen_force_knee_callback(const pr_msgs::msg::PRFloatH::SharedPtr gen_force_knee_msg){
        gen_force_knee = gen_force_knee_msg->data;
        read_gen_force_knee = true;
    }

    void AlgorithmAssistance::muscle_force_callback(const pr_msgs::msg::PRFloatH::SharedPtr muscle_force_msg){
        muscle_force = muscle_force_msg->data;
        read_muscle_force = true;
    }

    void AlgorithmAssistance::ref_muscle_force_callback(const pr_msgs::msg::PRFloatH::SharedPtr ref_muscle_force_msg){
        ref_muscle_force = ref_muscle_force_msg->data;
        read_ref_muscle_force = true;
    }

    void AlgorithmAssistance::muscle_dir_callback(const pr_msgs::msg::PRArrayH::SharedPtr muscle_dir_msg){
        PRUtils::ArRMsg2Eigen(muscle_dir_msg, muscle_dir);
        read_muscle_dir = true;
    }

    AlgorithmAssistance::~AlgorithmAssistance() {
    }    
}



#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_biomech::AlgorithmAssistance)