#include "pr_modelling/force_fixed_frame.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"



namespace pr_modelling
{
    /**** ForceFixedFrame COMPONENT ****/
    ForceFixedFrame::ForceFixedFrame(const rclcpp::NodeOptions & options)
    : Node("force_fixed_frame", options)
    {
        //Parameter declaration
        this->declare_parameter<double>("boot_mass", 2.2816);
        this->declare_parameter<std::vector<double>>("boot_cdg", {0.1201, 0.0007, 0.0453});
        this->declare_parameter<bool>("boot_compensation", true);
        this->declare_parameter<bool>("fixed_frame_noise_threshold", true);

        this->get_parameter("boot_mass", boot_mass);
        this->get_parameter("boot_cdg", boot_cdg);
        this->get_parameter("boot_compensation", boot_compensation);
        this->get_parameter("fixed_frame_noise_threshold", fixed_frame_noise_threshold);

        // Subscribers of position and force
        sub_x.subscribe(this, "x_coord");
        sub_f.subscribe(this, "force_state");
        sync_.reset(new Synchronizer(SyncPolicy(1), sub_x, sub_f));
        sync_->registerCallback(std::bind(&ForceFixedFrame::topic_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Publisher for the compensated force in the fixed platform
        publisher_f_fixed = this->create_publisher<pr_msgs::msg::PRForceState>("force_state_fixed", 1);

        // Initialization of std_noise
        std_noise.resize(6);
        std_noise[0] = 0.5; //0.036; //Original basado en experimento ruido
        std_noise[1] = 0.0675;
        std_noise[2] = 0.9;
        std_noise[3] = 0.03; //0.0045; //Original basado en experimento ruido
        std_noise[4] = 0.03; //0.0040; //Original basado en experimento ruido
        std_noise[5] = 0.03;

        // Vector of boot cdg
        boot_cdg_vector << boot_cdg[0], boot_cdg[1], boot_cdg[2];

    }

    void ForceFixedFrame::topic_callback(const pr_msgs::msg::PRArrayH::ConstPtr& x_msg, 
                                        const pr_msgs::msg::PRForceState::ConstPtr& f_msg)
    {  

        // fixed frame force msg and init time
        auto force_state_fixed_msg = pr_msgs::msg::PRForceState();
        force_state_fixed_msg.init_time = this->get_clock()->now();

        if (first_iter){
            // Initial orientation
            theta_ini = x_msg->data[2];
            psi_ini = x_msg->data[3];
            // Rotation matrix from the fixed to the mobile coordinate system
            Eigen::Matrix3d R_mf_ini_Boot = PRUtils::RotationMatrices3UPE_RPU(theta_ini,psi_ini).transpose();
            // Initial weight of the boot in mobile frame
            Fb_ini_m = boot_mass*R_mf_ini_Boot*g;
            // Initial torque of the boot in mobile coordinate
            Tb_ini_m = boot_cdg_vector.cross(Fb_ini_m);
            first_iter = false;
        }

        // Data from msgs to vectors
        Eigen::Vector3d Fent_m, Tent_m;
        Fent_m(0) = f_msg->force[0];
        Fent_m(1) = f_msg->force[1];
        Fent_m(2) = f_msg->force[2];
        Tent_m(0) = f_msg->momentum[0];
        Tent_m(1) = f_msg->momentum[1];
        Tent_m(2) = f_msg->momentum[2];

        double theta, psi;
        theta = x_msg->data[2];
        psi = x_msg->data[3];

        // CALCULATION OF BOOT COMPENSATION IN MOBILE COORDINATES
        Eigen::Vector3d DeltaFb_m, DeltaTb_m;
        if (boot_compensation){
            // Rotation Matrix from the fixed tot he mobile coordinate system
            Eigen::Matrix3d R_mf = PRUtils::RotationMatrices3UPE_RPU(theta,psi).transpose();

            // Calculation of the weight of the boot in the current measurement
            Eigen::Vector3d Fb_m = boot_mass*R_mf*g;

            //Calculo de la diferencia de fuerza aplicada por la bota (repecto a una medición inicial) en coordenadas moviles.
            DeltaFb_m =Fb_m-Fb_ini_m;

            //Calculo de la diferencia de momento aplicado por la bota (repecto a una medición inicial) en coordenadas moviles.
            DeltaTb_m = boot_cdg_vector.cross(Fb_m)-Tb_ini_m;
           
        }
        else{
            DeltaFb_m = Eigen::Vector3d::Zero();
            DeltaTb_m = Eigen::Vector3d::Zero();
        }

        // CHANGE OF COORDINATES TO FIXED FRAME
        // A partir de la orientación (Theta y Psi) del centro de la plataforma
        // del robot paralelo, y de las fuerzas del sensor de la plataforma movil
        // se calcula las fuerzas en el sistema de coordenadas fijo.
        Eigen::Vector3d Fent_f, Tent_f;
        // Rotation matrix from mobile to fixed coordinate system in the actual measurement
        Eigen::Matrix3d R_fm = PRUtils::RotationMatrices3UPE_RPU(theta,psi);
        //Ecuaciones para el vector de fuerzas de entorno en el sistema de coordenadas fijas ( Se compensa el peso de la bota)
        Fent_f = R_fm*(Fent_m-DeltaFb_m);
        //Ecuaciones para el vector de momentos de entorno en el sistema de coordenadas fijas ( Se compensa el peso de la bota)
        Tent_f = R_fm*(Tent_m-DeltaTb_m);
        
        if (fixed_frame_noise_threshold) apply_threshold(Fent_f, Tent_f);

        // Force message construction
        force_state_fixed_msg.force[0] = Fent_f(0);
        force_state_fixed_msg.force[1] = Fent_f(1);
        force_state_fixed_msg.force[2] = Fent_f(2);
        force_state_fixed_msg.momentum[0] = Tent_f(0);
        force_state_fixed_msg.momentum[1] = Tent_f(1);
        force_state_fixed_msg.momentum[2] = Tent_f(2);

        // Header configuration
        force_state_fixed_msg.header.frame_id = f_msg->header.frame_id;
        force_state_fixed_msg.header.stamp = f_msg->header.stamp;

        // Current time of the topic
        force_state_fixed_msg.current_time = this->get_clock()->now();
        // Publication of the topic
        publisher_f_fixed->publish(force_state_fixed_msg);

    }

    void ForceFixedFrame::apply_threshold(Eigen::Vector3d &force, Eigen::Vector3d &torque){
        for (int i=0; i<3; i++){
          if (abs(force(i)) < 4*std_noise[i]){
            force(i) = 0;
          }
          if (abs(torque[i]) < 4*std_noise[i+3]){
            torque(i) = 0;
          } 
        }
    }   
    

    
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_modelling::ForceFixedFrame)