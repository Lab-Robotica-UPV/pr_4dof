#include "pr_ref_gen/ref_static_auto.hpp"

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
    /**** STATIC REFERENCE AUTO GENERATOR COMPONENT ****/
    RefStaticAuto::RefStaticAuto(const rclcpp::NodeOptions & options)
    : Node("ref_static_auto", options)
    {
        //Parameter declaration
        this->declare_parameter<std::string>("ref_cart_path", 
            "/home/paralelo4dofnew/ros2_eloquent_ws/pr_4dof/references/ref_cart_TRR0_CF1_IdV1.txt");
        
        this->declare_parameter<std::vector<double>>("robot_config_params", 
            {0.4, 0.4, 0.4, 0.15, 90*(M_PI/180), 45*(M_PI/180), 0.3, 0.3, 0.3, 50*(M_PI/180), 90*(M_PI/180)});

        this->declare_parameter<double>("ts", 0.01);
        this->declare_parameter<int>("Nptos_set", 12000);
        this->declare_parameter<bool>("robot_5p", false);

        this->get_parameter("ref_cart_path", ref_path);
        this->get_parameter("robot_config_params", robot_params);
        this->get_parameter("ts", ts);
        this->get_parameter("Nptos_set", Nptos_set);
        this->get_parameter("robot_5p", robot_5p);

        //Selection final sample for the trajectory
        // If Nptos_set=0 the number of sample is equal to the rows in the reference file
        // Else the number of samples are defined by the user

        if (Nptos_set <= 0){
            //Read file
            if(PRUtils::read_file(ref_matrix_x, ref_path)==-1){
                RCLCPP_ERROR(this->get_logger(), "Could not open file");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Pose references file opened");
            
            // Original samples for the trajectory
            Nptos_o = ref_matrix_x.rows();
        }
        else{
            // Original samples for the trajectory
            Nptos_o = Nptos_set;
        }
        
        // PRINTING VERIFICATION ************ERASE LATER***************
        //std::cout << "*******************************" << std::endl;
        //std::cout << "Points for trajectory" << Nptos_o << std::endl;
        //std::cout << "*******************************" << std::endl;

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

        publisher_f_ref = this->create_publisher<pr_msgs::msg::PRArrayH>(
            "ref_force",
            1
        );

        publisher_end_ = this->create_publisher<std_msgs::msg::Bool>(
            "end_flag",
            1
        );

        subscription_external_stop_ = this->create_subscription<std_msgs::msg::Bool>(
            "external_stop",
            1,//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_sensor_data),
            std::bind(&RefStaticAuto::external_stop_callback, this, _1)
        );

        subscription_encoder = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "joint_position",
            1,
            std::bind(&RefStaticAuto::topic_callback_encoder, this, _1)
        );

        subscription_cameras = this->create_subscription<pr_msgs::msg::PRMocap>(
            "x_coord_mocap",
            1,
            std::bind(&RefStaticAuto::topic_callback_cameras, this, _1)
        );

    }

    void RefStaticAuto::topic_callback_encoder(const pr_msgs::msg::PRArrayH::SharedPtr q_msg)
    {
        if (read_encoder){
            if((idx<TrRobot_ns.rows()) && !external_stop)
            {
                //Ref message and init time
                auto ref_msg_q = pr_msgs::msg::PRArrayH();
                auto ref_msg_x = pr_msgs::msg::PRArrayH();
                auto ref_msg_qini = pr_msgs::msg::PRArrayH();
                auto fref_msg = pr_msgs::msg::PRArrayH();

                ref_msg_q.init_time = this->get_clock()->now();
                ref_msg_x.init_time = this->get_clock()->now();
                ref_msg_qini.init_time = this->get_clock()->now();
                fref_msg.init_time = this->get_clock()->now();

                //CONVERTIR A FUNCIÓN
                for(int i=0; i<4; i++){
                    ref_msg_q.data[i] = Trqind_ns(idx, i);
                    ref_msg_x.data[i] = TrRobot_ns(idx, i);
                    ref_msg_qini.data[i] = Trqind_ns(0, i);
                    fref_msg.data[i] = 0;
                }

                
                ref_msg_q.header.stamp = q_msg->header.stamp;
                ref_msg_q.header.frame_id = q_msg->header.frame_id;
                
                ref_msg_x.header.stamp = q_msg->header.stamp;
                ref_msg_x.header.frame_id = q_msg->header.frame_id;

                ref_msg_qini.header.stamp = q_msg->header.stamp;
                ref_msg_qini.header.frame_id = q_msg->header.frame_id;

                fref_msg.header.stamp = q_msg->header.stamp;
                fref_msg.header.frame_id = q_msg->header.frame_id;

                ref_msg_q.current_time = this->get_clock()->now();
                publisher_q->publish(ref_msg_q);
                ref_msg_x.current_time = this->get_clock()->now();
                publisher_x->publish(ref_msg_x);
                ref_msg_qini.current_time = this->get_clock()->now();
                publisher_qini->publish(ref_msg_qini);

                fref_msg.current_time = this->get_clock()->now();
                publisher_f_ref->publish(fref_msg);
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

    void RefStaticAuto::topic_callback_cameras(const pr_msgs::msg::PRMocap::SharedPtr x_cam_msg)
    {

        if (!read_encoder){

            if (idx<ndata_cam){

                for (int i=0; i<4; i++){
            
                    x_cam(i)=x_cam_msg->x_coord.data[i];
                    
                }

                MX_init_estim.row(idx) = x_cam.transpose();
                idx++;
                std::cout << "Mocap read " << idx << std::endl;
                
            }    

            else{
                // Obtain the mean
                x_ini = MX_init_estim.colwise().mean();
                for (int i=0; i<4; i++) x_ini_arr[i] = x_ini(i);
                // Arrays

                // TRAJECTORY GENERATION
                
                // Factibility verification initial point
                verif_ini = PRLimits::VerFactPos(x_ini_arr, robot_params, robot_5p);
                // Verificamos que los puntos a unir tienen el mismo signo
                if ((verif_ini.cwiseAbs()).sum()==0){

                    // Non singular Trajectory in configuration space
                    TrRobot_ns = Eigen::MatrixXd::Zero(Nptos_o, 4);
                    // Non singular Trajectory in joint space
                    Trqind_ns = Eigen::MatrixXd::Zero(Nptos_o, 4);
                    
                    // Initial pose for the actuators based on CAMERAS 
                    PRModel::InverseKinematics(solq, x_ini_arr, robot_params);

                    // Bucle for repeting the same position
                    for (int i=0;i<Nptos_o;i++){
                        // Store initial pose for trajectorey in configuration space
                        TrRobot_ns.row(i) << x_ini_arr[0], x_ini_arr[1], x_ini_arr[2], x_ini_arr[3];                       
                    
                        // Store initial pose for trajectorey in joint space
                        Trqind_ns.row(i) << solq(0,2), solq(1,2), solq(2,2), solq(3,1);
                    }
                    std::cout << "Ref. generation complete " << std::endl;

                    idx = 0;
                    read_encoder = true;
                }
                else{
                    std::cout << "Initial point is physically no factible: " << verif_ini.transpose() << ", x ini: " << x_ini.transpose() << std::endl;
                    
                    auto end_msg = std_msgs::msg::Bool();
                    end_msg.data = true;
                    publisher_end_->publish(end_msg);
                }
        
            } 

        }

    }
    
    void RefStaticAuto::external_stop_callback(const std_msgs::msg::Bool::SharedPtr external_stop_msg)
    {
        if (!external_stop)
            external_stop = external_stop_msg->data;

    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_ref_gen::RefStaticAuto)