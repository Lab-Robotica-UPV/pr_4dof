#include "pr_dmp/dmp_reversible.hpp"

#include <chrono>
#include <memory>

using std::placeholders::_1;


namespace pr_dmp
{
    /**** DmpRev COMPONENT ****/
    DmpRev::DmpRev(const rclcpp::NodeOptions & options)
    : Node("dmp_rev", options)
    {
        // Parameter declaration
        // Trajectory file of DMP (cart or prism, indicated by the bool "isCart")
        this->declare_parameter<std::string>("ref_path", 
        "/home/paralelo4dofnew/ros2_eloquent_ws/pr_4dof/references/ref_cart_TRR0_CF1_IdV1.txt");
        // Trajectory file of GKA (for example, force)
        this->declare_parameter<std::string>("gka_path", "");
        // Sample time
        this->declare_parameter<double>("ts", 0.01);
        // Number of basis functions
        this->declare_parameter<int>("n_basis_functions", 100);
        // True if the file is in cartesian coordinates (false if it is in prismatic)
        this->declare_parameter<bool>("isCart", false);
        // In case of a prismatic file, whether to calculate the cartesian (otherwise the topic will send 0)
        // This is useful when we only have the prismatic file (for example for moving disassembled legs)
        // Or when we do not have a cartesian file (since we need ref_x_init to start calculating it)
        this->declare_parameter<bool>("calcCart", true);
        // Init cartesian position. Only needed if calcCart is in true
        this->declare_parameter<std::vector<double>>("ref_x_init", {0.069190, 0.632000, 0.078714, -0.135787});
        // Robot config params for calculating the Direct/Inverse Kinematics
        this->declare_parameter<std::vector<double>>(
            "robot_config_params", 
            {0.4, 0.4, 0.4, 0.15, 90*(M_PI/180), 45*(M_PI/180), 0.3, 0.3, 0.3, 50*(M_PI/180), 90*(M_PI/180)});
        // DMP Dynamic parameters
        this->declare_parameter<std::vector<double>>("damping_coefficient", {894.0, 894.0, 89.4, 89.4});
        this->declare_parameter<std::vector<double>>("spring_constant", {250.0, 500.0, 25.0, 25.0});
        this->declare_parameter<std::vector<double>>("mass", {200.0, 200.0, 20.0, 20.0});
        // Speed of the execution
        this->declare_parameter<double>("speed", 1.0);
        // Gain of the slowdown
        this->declare_parameter<double>("gain_slowdown", 0.0);

        // Assignment of previous parameters to variables
        this->get_parameter("ref_path", ref_path);
        this->get_parameter("gka_path", gka_path);
        this->get_parameter("ts", ts);
        this->get_parameter("n_basis_functions", n_basis_functions);
        this->get_parameter("isCart", isCart);
        this->get_parameter("calcCart", calcCart);
        this->get_parameter("ref_x_init", ref_x_init);
        this->get_parameter("robot_config_params", robot_params);
        this->get_parameter("damping_coefficient", damping_coefficient);
        this->get_parameter("spring_constant", spring_constant);
        this->get_parameter("mass", mass);
        this->get_parameter("speed", speed);
        this->get_parameter("gain_slowdown", gain_slowdown);


        // Avoid 0 speed
        if (speed==0) speed=1;

        // Publisher of joint position
        publisher_q_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"dmp_ref_gen_q", 
			1);
        // Publisher of cart position (will be zeros if isCart and calcCart are both set to false)
        publisher_x_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"dmp_ref_gen_x", 
			1);
        // Publisher of the GKA trajectory
        publisher_gka_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"gka_traj", 
			1);
        // Publisher of the phase
        publisher_phase_ = this->create_publisher<pr_msgs::msg::PRFloatH>(
			"dmp_phase", 
			1);
        // Publisher that ends the streaming
        publisher_end_ = this->create_publisher<std_msgs::msg::Bool>(
            "end_flag",
            1
        );

        // Subscriptor for the acceleration force for the DMP (will be added in the term of the acceleration)
        subscription_force_accel_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "dmp_force_accel",
            1,
            std::bind(&DmpRev::force_accel_callback, this, _1));
        // Subscriptor for the velocity force for the DMP (will be added in the term of the velocity)
        subscription_force_vel_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "dmp_force_vel",
            1,
            std::bind(&DmpRev::force_vel_callback, this, _1));
        // Subscription to the joint position
        subscription_pos_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "joint_position",
            1,
            std::bind(&DmpRev::pos_callback, this, _1));

        // Trajectory generation for DMP
        //Read file
        if(PRUtils::read_file(ref_pos, ref_path)==-1){
            cerr << "The TXT file " << ref_path << " could not be found. Aborting." << endl << endl;
            return;
        }
        else ref_read = true;
        n_dims = ref_pos.cols();

        // Trajectory generation for GKA
        Trajectory trajectory_gka;
        if (gka_path != ""){
             trajectory_gka = Trajectory::readFromPosFile(gka_path, ts);
             if (trajectory_gka.length()==0)
            {
                cerr << "The TXT file " << gka_path << " could not be found. Aborting." << endl << endl;
                return;
            }
            else gka_read = true; // We read the GKA trajectory succesfully
        } 


        // DEFINITION OF DMP
        // Transformcion de los coeficientes a vectores
        damping_coefficient_vec.resize(n_dims);
        spring_constant_vec.resize(n_dims);
        mass_vec.resize(n_dims);
        for (int i=0; i<n_dims;i++){
            damping_coefficient_vec(i) = damping_coefficient[i];
            spring_constant_vec(i) = spring_constant[i];
            mass_vec[i] = mass[i];
        }

        // CONSTRUCT AND TRAIN THE DMP
  
        // Initialize the DMP
        dmp = new DmpReversible(ref_pos, ts, n_basis_functions, spring_constant_vec, damping_coefficient_vec, mass_vec);

        // Train the DMP. Passing the directory will make sure the results are saved to file.
        bool overwrite = true;
        dmp->train();

        // Adjust the speed
        //dmp->set_speed(speed);

        // DEFINITION OF GKA
        if (gka_read){
            int input_dim = 1;
            //Eigen::Vector4i n_basis_functions{100, 100, 100, 100};
            double intersection = 0.5;
            // alpha for the phase exponential system
            double alpha_phase = 2.0;
            // Vector de aproximadores para el GKA
            vector<FunctionApproximator*> function_approximators_gka(n_dims);
            MetaParametersLWR* meta_parameters_gka = new MetaParametersLWR(input_dim,n_basis_functions,intersection, 0.0, false, alpha_phase);      
            FunctionApproximatorLWR* fa_lwr_gka = new FunctionApproximatorLWR(meta_parameters_gka);

            for (int dd=0; dd<n_dims; dd++)
                function_approximators_gka[dd] = fa_lwr_gka->clone();

            // Initialization
            gka = new Gka(n_dims, function_approximators_gka, alpha_phase);
            // Training
            gka->train(trajectory_gka);
            // Adjust the speed
            //gka->set_tau(gka->tau()/speed);

        }

        // Initialization of the states and its derivatives for the GKA
        y_gka.resize(n_dims);
        x_gka.resize(1);
        xd_gka.resize(1);

        // // Limit phase so that the exponential system lasts as much as the trajectory file (without stop conditions)
        // if (dmp->getAlphaPhase() != 0){
        //     int nn = trajectory_dmp.length();
        //     double t_f = (nn+1)*ts;
        //     phase_limit = exp(-dmp->getAlphaPhase()*t_f/tau);
        // }
        // else phase_limit = 0.01; // If the phase system is not exponential or alpha=0
        phase_limit = 0.0;
        
        // Previous value for direct kinematics
        x_dir_kin = ref_x_init;

        
        
    }

    DmpRev::~DmpRev(){
        delete dmp;
        delete gka;
    }

    // Callback for the acceleration force
    void DmpRev::force_accel_callback(const pr_msgs::msg::PRArrayH::SharedPtr force_accel_msg)
    {
        for(int i=0; i<4; i++)
            force_accel[i] = force_accel_msg->data[i];
        // if (force_accel.norm() < 20.0) forward=1.0;
        // else forward = -1.0;
        // force_accel.fill(0.0);
    }

    // Callback for the velocity force
    void DmpRev::force_vel_callback(const pr_msgs::msg::PRArrayH::SharedPtr force_vel_msg)
    {
        for(int i=0; i<4; i++)
            force_vel[i] = force_vel_msg->data[i];
    }

    // Callback for the joint position
    void DmpRev::pos_callback(const pr_msgs::msg::PRArrayH::SharedPtr pos_msg)
    {
        //DMP message and init time
        auto ref_dmp_q_msg = pr_msgs::msg::PRArrayH();
        ref_dmp_q_msg.init_time = this->get_clock()->now();
        auto ref_dmp_x_msg = pr_msgs::msg::PRArrayH();
        ref_dmp_x_msg.init_time = this->get_clock()->now();
        auto phase_dmp_msg = pr_msgs::msg::PRFloatH();
        phase_dmp_msg.init_time = this->get_clock()->now();
        // GKA message and init time
        auto gka_msg = pr_msgs::msg::PRArrayH();
        gka_msg.init_time = this->get_clock()->now();

        // Make sure we read the trajectory
        if (ref_read){

            // If this is the first iter, we make the first integration of the system
            if (first_iter){
                state_dmp = dmp->integrate();
                x_dmp = state_dmp.head(1)[0]; // Phase
                y_dmp = state_dmp.segment(1,n_dims); // Position
                // z_dmp = state_dmp.segment(n_dims+1,2*n_dims);
                if (gka_read){
                    y_gka = gka->integrateStart(x_gka,xd_gka);
                }
                first_iter = false;
            }
            
            else{
                phase_current(0) = x_dmp;
                //std::cout << phase_current << std::endl;
                
                if (x_dmp > phase_limit){

                    // Ahora mismo se define el slowdown a partir del error en la fuerza de aceleracion 
                    // (lo que le está costado llegar a la referencia)
                    // La dinámica dependerá de alpha_x y tau, por lo que se puede diseñar (modificar)
                    // el gain_slowdown para que la ganancia sea independiente de la dinámica

                    slowdown = gain_slowdown*force_accel.norm();
                    // if (n_iter<5000){ speed=min(speed+0.005, 1.0);}
                    // else{ speed=max(speed-0.005, -1.0);}

                    // Keep integrating as long as the phase is over the limit and update the data
                    state_dmp = dmp->integrate(speed,slowdown,force_accel);
                    x_dmp = state_dmp.head(1)[0]; // Phase
                    y_dmp = state_dmp.segment(1,n_dims); // Position
                    if (gka_read){
                        y_gka = gka->integrateStepWithSlowDown(ts,phase_current,xd_gka,slowdown);
                    }
                }
                else{
                    // End the task and publish it
                    end_task = true;
                    auto end_msg = std_msgs::msg::Bool();
                    end_msg.data = true;
                    publisher_end_->publish(end_msg);
                }
            }

            // Current iteration (is not used anywhere else and is just for visual purposes)
            n_iter++;
            std::cout << n_iter << std::endl;

            if (!end_task){
                // s_pos can be either prismatic or cartesian according to "isCart". Now we obtain the other
                // The fase is obtained via the following formula (borrowed from PHASE macro in DMP lib)
                phase_current(0) = x_dmp;;
                phase_dmp_msg.data = x_dmp;

                if (isCart){
                    // We performe the inverse kinematics

                    for (int i=0; i<4; i++){
                        ref_dmp_x_msg.data[i] = y_dmp(i);
                    }

                    PRModel::InverseKinematics(q_inv_kin, ref_dmp_x_msg.data, robot_params);
                    ref_dmp_q_msg.data[0] = q_inv_kin(0,2);
                    ref_dmp_q_msg.data[1] = q_inv_kin(1,2);
                    ref_dmp_q_msg.data[2] = q_inv_kin(2,2);
                    ref_dmp_q_msg.data[3] = q_inv_kin(3,1);

                }

                else{
                    // We perform the direct kinematics

                    for (int i=0; i<4; i++){
                        ref_dmp_q_msg.data[i] = y_dmp(i);
                    }

                    if (calcCart){
                        ref_dmp_x_msg.data = PRModel::ForwardKinematics(ref_dmp_q_msg.data, x_dir_kin, robot_params, tol, iter);
                        PRUtils::array2vector(ref_dmp_x_msg.data, x_dir_kin);
                    }
                    else{
                        for (int i=0; i<4; i++) ref_dmp_x_msg.data[i] = 0;
                    }

                }

                if (gka_read){
                    for (int i=0; i<4; i++) gka_msg.data[i] = y_gka[i];
                    gka_msg.header.stamp = pos_msg->header.stamp;
                    gka_msg.header.frame_id = pos_msg->header.frame_id;

                    gka_msg.current_time = this->get_clock()->now();
                    publisher_gka_->publish(gka_msg);
                }

                // Prepare messages for publication
                ref_dmp_q_msg.header.stamp = pos_msg->header.stamp;
                ref_dmp_q_msg.header.frame_id = pos_msg->header.frame_id;

                phase_dmp_msg.header.stamp = pos_msg->header.stamp;
                phase_dmp_msg.header.frame_id = pos_msg->header.frame_id;

                ref_dmp_x_msg.header.stamp = pos_msg->header.stamp;
                ref_dmp_x_msg.header.frame_id = pos_msg->header.frame_id;
                
                ref_dmp_q_msg.current_time = this->get_clock()->now();
                publisher_q_->publish(ref_dmp_q_msg);

                ref_dmp_x_msg.current_time = this->get_clock()->now();
                publisher_x_->publish(ref_dmp_x_msg);

                phase_dmp_msg.current_time = this->get_clock()->now();
                publisher_phase_->publish(phase_dmp_msg);
            }
        }
        else
        {
            // If the trajectory was not properly read, publish the end
            end_task = true;
            auto end_msg = std_msgs::msg::Bool();
            end_msg.data = true;
            publisher_end_->publish(end_msg);
        }
        
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_dmp::DmpRev)