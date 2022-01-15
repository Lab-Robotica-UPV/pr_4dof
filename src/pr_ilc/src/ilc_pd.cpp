#include "pr_ilc/ilc_pd.hpp"

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


namespace pr_ilc
{
    /**** ILC PD COMPONENT ****/
    IlcPD::IlcPD(const rclcpp::NodeOptions & options)
    : Node("ilc_pd", options)
    {
        //Parameter declaration
        this->declare_parameter<std::string>("ref_path", 
            "/home/paralelo4dofnew/parallel_robot_ws/references/ref_cart_TRR0_CF1_IdV1.txt");
        this->declare_parameter<std::string>("load_path", "");
        this->declare_parameter<std::string>("save_path", "");
        this->declare_parameter<std::vector<double>>("kp_gain", {10000.0, 10000.0, 10000.0, 50000.0});
        this->declare_parameter<std::vector<double>>("kd_gain", {100.0, 100.0, 100.0, 500.0});
        this->declare_parameter<double>("traj_repetitions", 1.0);
        

        this->get_parameter("ref_path", ref_path);
        this->get_parameter("load_path", load_path);
        this->get_parameter("save_path", save_path);
        this->get_parameter("kp_gain", Kp);
        this->get_parameter("kd_gain", Kd);
        this->get_parameter("traj_repetitions", traj_repetitions);

        //Apertura fichero de referencia para obtener el numero de muestras
        if(PRUtils::read_file(ref_matrix, ref_path)==-1){
                RCLCPP_ERROR(this->get_logger(), "Could not open file");
                return;
        }
        RCLCPP_INFO(this->get_logger(), "Pose references file opened");
        n_ref = ref_matrix.rows();

        // Abrir fichero de errores y acciones que se corresponda con la referencia
        if (load_path != ""){
            if(PRUtils::read_file(e_ant, load_path+"_e.txt")==-1){
                    RCLCPP_ERROR(this->get_logger(), "Could not open error file");
                    return;
            }
            if(PRUtils::read_file(u_ant, load_path+"_u.txt")==-1){
                    RCLCPP_ERROR(this->get_logger(), "Could not open action file");
                    return;
            }
            // Comprobar que tienen el mismo numero de elementos
            if(n_ref!=e_ant.rows() || n_ref!=u_ant.rows()){
                RCLCPP_ERROR(this->get_logger(), "The number of samples of the files does not match");
                return;
            }
            std::cout << e_ant.rows() << std::endl;
            std::cout << u_ant.rows() << std::endl;
        }

        else{
            // Inicializar a ceros
            e_ant = Eigen::MatrixXd::Zero(n_ref, 4);
            u_ant = Eigen::MatrixXd::Zero(n_ref, 4);
        }

        e_init = e_ant.row(0);
        iter = 0;
        alpha = pow(2.0, 10.0/(traj_repetitions-1.0+10.0))-1.0;

        // Create communication
        sub_pos.subscribe(this, "joint_position");
        sub_ref.subscribe(this, "ref_pose");


        sync_.reset(new Synchronizer(SyncPolicy(1), sub_pos, sub_ref));
        sync_->registerCallback(std::bind(&IlcPD::controller_callback, this, std::placeholders::_1, std::placeholders::_2));
        //sync_->setMaxIntervalDuration(rclcpp::Duration(0, 1000000));

        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>("control_action_ilc", 1);


    }

    IlcPD::~IlcPD(){
        if (save_path!= "" && iter>=n_ref){
            // Guardar contenido de las matrices
            std::ofstream file_e(save_path+"_e.txt");
            std::ofstream file_u(save_path+"_u.txt");
            Eigen::IOFormat spaceFormat(Eigen::StreamPrecision, Eigen::DontAlignCols);//(4, 0, " ", "\n", "", "", "", "", "");
            if (file_e.is_open())
            {
                file_e << e_ant.format(spaceFormat);
                file_e.close();
            }
            if (file_u.is_open())
            {
                file_u << u_ant.format(spaceFormat);
                file_u.close();
            }
        }
    }
  

    void IlcPD::controller_callback(const pr_msgs::msg::PRArrayH::ConstPtr& pos_msg,
                                            const pr_msgs::msg::PRArrayH::ConstPtr& ref_msg)
    {
    
        // Control action message and init time
        auto control_action_msg = pr_msgs::msg::PRArrayH();
        control_action_msg.init_time = this->get_clock()->now();
            
        PRUtils::ArRMsg2Eigen(pos_msg, pos);
        PRUtils::ArRMsg2Eigen(ref_msg, ref);

        e_ant_k = e_ant.row(iter);
        u_ant_k = u_ant.row(iter); 
            
        // En el ultimo instante de la trayectoria no existe e_ant_k1, por lo que se toma el del
        // primer instante asumiendo trayectoria ciclica
        if(iter<(n_ref-1)) e_ant_k1 = e_ant.row(iter+1);
        else e_ant_k1 = e_init;

        // Current error
        e_k = ref-pos;
        // Current control action
        //Calculate control action
        for(int i=0; i<4; i++)
            u_k(i) = u_ant_k(i) + alpha*Kp[i]*(e_ant_k1(i)) + alpha*Kd[i]*(e_ant_k(i)-e_ant_k1(i));

        e_ant.row(iter) = e_k;
        u_ant.row(iter) = u_k;

        PRUtils::Eigen2ArMsg(u_k, control_action_msg);

        iter++;

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

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_ilc::IlcPD)