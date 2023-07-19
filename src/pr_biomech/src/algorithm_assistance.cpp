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
        this->declare_parameter<double>("ki", 0);
        this->declare_parameter<double>("kd", 0);
        this->declare_parameter<double>("c_filt", 0);
        this->declare_parameter<double>("length_tibia", 0.5);
        this->declare_parameter<double>("length_foot", 0.25);

        this->get_parameter("ts", ts);
        this->get_parameter("kp", kp);
        this->get_parameter("ki", ki);
        this->get_parameter("kd", kd);
        this->get_parameter("c_filt", c_filt);
        this->get_parameter("length_tibia", length_tibia);
        this->get_parameter("length_foot", length_foot);

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
        pub_ass_force_prop = this->create_publisher<pr_msgs::msg::PRFloatH>("ass_force_prop", 1);
        pub_ass_force_der = this->create_publisher<pr_msgs::msg::PRFloatH>("ass_force_der", 1);
        pub_ass_force_filt = this->create_publisher<pr_msgs::msg::PRFloatH>("ass_force_filt", 1);
        pub_ass_alpha = this->create_publisher<pr_msgs::msg::PRFloatH>("ass_alpha", 1);
    }

    void AlgorithmAssistance::force_callback(const pr_msgs::msg::PRForceState::SharedPtr force_state_msg){
        force_state[0] = force_state_msg->force[0];
        force_state[1] = force_state_msg->force[2];
        force_state[2] = force_state_msg->momentum[1]/length_tibia;
        force_state[3] = 0*force_state_msg->momentum[2]/length_foot;
        read_force_state = true;

        if (read_muscle_force && read_ref_muscle_force && read_muscle_dir){
            auto ass_force_msg = pr_msgs::msg::PRArrayH();
            ass_force_msg.init_time = this->get_clock()->now();

            auto ass_force_prop_msg = pr_msgs::msg::PRFloatH();
            ass_force_prop_msg.init_time = this->get_clock()->now();

            auto ass_force_der_msg = pr_msgs::msg::PRFloatH();
            ass_force_der_msg.init_time = this->get_clock()->now();

            auto ass_force_filt_msg = pr_msgs::msg::PRFloatH();
            ass_force_filt_msg.init_time = this->get_clock()->now();

            auto ass_alpha_msg = pr_msgs::msg::PRFloatH();
            ass_alpha_msg.init_time = this->get_clock()->now();

            // Here goes the algorithm
            force_state_norm = force_state.norm();
            if (force_state_norm>0){// && muscle_force>0){
                force_state_dir = force_state/force_state_norm;
                force_angle = abs(acos(force_state_dir.dot(muscle_dir)));

                if (force_angle > M_PI/2) {
                    alpha_desired = 0;
                    // e_ant = 0;
                    // ud_ant = 0;
                }
                else if (force_angle < M_PI/2 && force_angle > force_angle_lim){
                    alpha_desired = 1/(force_angle_lim-M_PI/2)*(force_angle-M_PI/2);
                }
                else {
                    alpha_desired = 1;
                }

                if (alpha_desired > alpha){
                    alpha = std::min(alpha_desired, alpha+alpha_inc);
                }
                else if (alpha_desired < alpha){
                    alpha = std::max(alpha_desired, alpha-alpha_inc);
                } 

                // Assistive force
                e = alpha*(ref_muscle_force - muscle_force);
                
                // Update derivative according to the presence of filter
                
                ass_force_prop = kp*e;
                ass_force_der = kd*(e-e_ant)/ts;

                ass_force_filt = ud_ant*c_filt + (ass_force_prop+ass_force_der)*(1-c_filt);
                ass_force_scalar = ass_force_filt; // + ass_force_prop;
                //ass_force_filt = ud_ant*c_filt + ass_force_scalar*(1-c_filt);

                // Update (and weight?)
                e_ant = e;
                ud_ant = ass_force_filt;

                // Clip the maximum force
                // ass_force_scalar = std::min(ass_force_scalar, force_state_norm);

                // Add direction (opposite to the scalar)
                ass_force = ass_force_scalar*force_state_dir;

                // Clip the maximum force and readapt forces to torques
                for (int i=0; i<4; i++){
                    if (force_state[i] > 0){
                        ass_force[i] = std::min(ass_force[i], force_state[i]);
                    }
                    else {
                        ass_force[i] = std::max(ass_force[i], force_state[i]);
                    }
                }
                ass_force[2] *= length_tibia;
                ass_force[3] *= length_foot;
                
                // ass_force[0] = std::min(ass_force[0], force_state[0]);
                // ass_force[1] = std::min(ass_force[1], force_state[1]);
                // ass_force[2] = std::min(ass_force[2], force_state[2])*length_tibia;
                // ass_force[3] = std::min(ass_force[3], force_state[3])*length_foot;

                // Change direction
                ass_force *= -1;
            }
            else{
                alpha = 0.0;
                e_ant = 0;
                ud_ant = 0;
                ass_force[0] = 0;
                ass_force[1] = 0;
                ass_force[2] = 0;
                ass_force[3] = 0;
                ass_force_prop = 0.0;
                ass_force_der = 0.0;
                ass_force_filt = 0.0;
            }



            // Fill messages
            PRUtils::Eigen2ArMsg(ass_force, ass_force_msg);
            ass_force_prop_msg.data = ass_force_prop;
            ass_force_der_msg.data = ass_force_der;
            ass_force_filt_msg.data = ass_force_filt;
            ass_alpha_msg.data = alpha;

            // Send messages
            ass_force_msg.header.stamp = force_state_msg->header.stamp;
            ass_force_msg.header.frame_id = force_state_msg->header.frame_id;

            ass_force_msg.current_time = this->get_clock()->now();
            pub_ass_force->publish(ass_force_msg);

            ass_force_prop_msg.header.stamp = force_state_msg->header.stamp;
            ass_force_prop_msg.header.frame_id = force_state_msg->header.frame_id;

            ass_force_prop_msg.current_time = this->get_clock()->now();
            pub_ass_force_prop->publish(ass_force_prop_msg);

            ass_force_der_msg.header.stamp = force_state_msg->header.stamp;
            ass_force_der_msg.header.frame_id = force_state_msg->header.frame_id;

            ass_force_der_msg.current_time = this->get_clock()->now();
            pub_ass_force_der->publish(ass_force_der_msg);

            ass_force_filt_msg.header.stamp = force_state_msg->header.stamp;
            ass_force_filt_msg.header.frame_id = force_state_msg->header.frame_id;

            ass_force_filt_msg.current_time = this->get_clock()->now();
            pub_ass_force_filt->publish(ass_force_filt_msg);

            ass_alpha_msg.header.stamp = force_state_msg->header.stamp;
            ass_alpha_msg.header.frame_id = force_state_msg->header.frame_id;

            ass_alpha_msg.current_time = this->get_clock()->now();
            pub_ass_alpha->publish(ass_alpha_msg);
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