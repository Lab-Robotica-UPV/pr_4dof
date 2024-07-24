#include "pr_dmp/dmp_reversible_force_lim.hpp"

#include <chrono>
#include <memory>

using std::placeholders::_1;


namespace pr_dmp
{
    /**** DmpRev COMPONENT ****/
    DmpRevFLim::DmpRevFLim(const rclcpp::NodeOptions & options)
    : Node("dmp_rev_flim", options)
    {
        // Parameter declaration
        // Trajectory file of DMP (cart or prism, indicated by the bool "isCart")
        this->declare_parameter<std::string>("ref_path", 
        "/home/paralelo4dofnew/ros2_eloquent_ws/pr_4dof/references/ref_cart_TRR0_CF1_IdV1.txt");
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
        this->declare_parameter<double>("speed", 0.0);
        // Gain of the slowdown
        this->declare_parameter<double>("gain_slowdown", 0.0);
        // GMR activation
        this->declare_parameter<bool>("gmr_activate", false);
        // Path for GMR
        this->declare_parameter<std::string>("gmr_path",  "/home/paralelo4dofnew/ros2_eloquent_ws/pr_4dof/gmr/Fer_der");

        // Assignment of previous parameters to variables
        this->get_parameter("ref_path", ref_path);
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
        this->get_parameter("gmr_activate", gmr_activate);
        this->get_parameter("gmr_path", gmr_path);

        // Publisher of joint position
        publisher_q_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"dmp_ref_gen_q", 
			1);
        // Publisher of cart position (will be zeros if isCart and calcCart are both set to false)
        publisher_x_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"dmp_ref_gen_x", 
			1);
        // Publisher of the phase
        publisher_phase_ = this->create_publisher<pr_msgs::msg::PRFloatH>(
			"dmp_phase", 
			1);
        // Publisher of the phase speed
        publisher_phase_speed_ = this->create_publisher<pr_msgs::msg::PRFloatH>(
			"dmp_phase_speed", 
			1);
        // Publisher that ends the streaming
        publisher_end_ = this->create_publisher<std_msgs::msg::Bool>(
            "end_flag",
            1
        );
        //Subscriptor for external stop
        subscription_external_stop_ = this->create_subscription<std_msgs::msg::Bool>(
            "external_stop",
            1,//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_sensor_data),
            std::bind(&DmpRevFLim::external_stop_callback, this, _1)
        );
        // Subscriptor for the force for the DMP (will be added in the term of the acceleration)
        subscription_force_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "dmp_force",
            1,
            std::bind(&DmpRevFLim::force_callback, this, _1));
        // Subscription to the joint position
        subscription_pos_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "joint_position",
            1,
            std::bind(&DmpRevFLim::pos_callback, this, _1));

        // Trajectory generation for DMP
        //Read file
        if(PRUtils::read_file(ref_pos, ref_path)==-1){
            cerr << "The TXT file " << ref_path << " could not be found. Aborting." << endl << endl;
            return;
        }
        else ref_read = true;
        n_dims = ref_pos.cols();

        //FINDING THE INITIAL POSITION WHERE THE USER TAKE CONTROL
        ref_pos_init=ref_pos.row(0);
        //std::cout << "Initial reference pose:" << ref_pos_init << std::endl;

        // FINDING THE TIME INSTANT WHERE THE PATIENT START CONTROLLING THE TRAJECTORY
        int i = ref_pos.rows()-1;
        while (t_user_ctrl < 0 && i > 0)
        {
            //pose comparison
            auto aux_pos = ref_pos.row(i);
            bool verf_pos = true;
            int j = 0;
            while (j<n_dims)
            {
                if (ref_pos_init[j] != aux_pos[j]){
                    verf_pos = false;
                    break;
                }
                j++;
            }
            
            if (verf_pos)
            {
                t_user_ctrl=i*ts;
                std::cout << "Time instat where the patient start controlling:" << t_user_ctrl << " iter." << i << std::endl;
                i = 0;
            }
            i--;
        }
        if(t_user_ctrl < 0){
            cerr << " Time instat where the patient start controlling was not found." << endl << endl;
            return;
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
        std::cout << "Creating DMP" << std::endl;
        dmp = new DmpReversible(ref_pos, ts, n_basis_functions, spring_constant_vec, damping_coefficient_vec, mass_vec, a_x);

        // Train the DMP. Passing the directory will make sure the results are saved to file.
        // bool overwrite = true;
        dmp->train();

        // Initialize the GMR
        gmr.loadParams(gmr_path.c_str());
        input_gmr.Resize(1,1);
        sigma_gmr = new Matrix[1];
        inC.Resize(1);
        outC.Resize(gmr.get_dim());
        inC(0)=0.0; // Columns of the input data for regression (here, time)
        for(unsigned int i=0;i<gmr.get_dim()-1;i++) 
            outC(i)=(float)(i+1); // Columns for output : remainings
        //gmr.debug();

        // Previous value for direct kinematics
        x_dir_kin = ref_x_init;

        // Phase limit (if 0, the experiment will never be stopped)
        phase_limit = 0.0;
        
    }

    DmpRevFLim::~DmpRevFLim(){
        delete dmp;
    }

    // Callback for the force
    void DmpRevFLim::force_callback(const pr_msgs::msg::PRArrayH::SharedPtr force_msg)
    {
        for(int i=0; i<4; i++)
            force[i] = force_msg->data[i];
        // if (force_accel.norm() < 20.0) forward=1.0;
        // else forward = -1.0;
        // force_accel.fill(0.0);
        force[2] /= scaling_factor;
        force[3] /= scaling_factor;
        norm_f = force.norm();

        //Force coming from sensor in the frame aligned with the boot
        force_rot[0]=(cos(ang_boot)*force[0])-(sin(ang_boot)*force[1]);
        force_rot[1]=(sin(ang_boot)*force[0])+(cos(ang_boot)*force[1]);
        force_rot[2]=0*force[2];
        force_rot[3]=0*force[3];
        norm_f_rot = force_rot.norm();

        //Printing the original sensor and the rotation
        std::cout << "Sensor: " << force.transpose() << std::endl;
        std::cout << "Sensor rot: " << force_rot.transpose() << std::endl;

        input_gmr(0,0) = -dmp->get_tau()/a_x*log(x_dmp);
        mean_gmr = gmr.doRegression(input_gmr,sigma_gmr,inC,outC);
                // std::cout << mean_gmr(0,gmr.get_dim()-2) << std::endl;
                // std::cout << sqrt(sigma_gmr[0](gmr.get_dim()-2, gmr.get_dim()-2));
                // Se calcula el error para la norma de la fuerza
        error = (norm_f-mean_gmr(0, gmr.get_dim()-2))/sqrt(sigma_gmr[0](gmr.get_dim()-2, gmr.get_dim()-2));
                // std::cout << "Error: " << error << std::endl;
        if (input_gmr(0,0) >= t_user_ctrl)
        {
            //First transition to controlling the robot by user
            if (!flag_user_ctrl){
                speed=0.0;
                flag_user_ctrl=true;
            }
            if ((force[0] > 0) && norm_f_rot > force_limit[0]){  //|| (force[1] < -force_limit[1]
                speed = min(speed+0.025*pow(speed,2)+0.005, 1.0);
            }  
            else if ((force[0] < -0) && norm_f_rot > force_limit[0]){  //&& norm_f_rot > force_limit[0]  || (force[1] > force_limit[1])
                speed = max(speed-0.025*pow(speed,2)-0.005, -1.0);   //speed = min(speed+0.005*pow(speed,2)+0.001, 1.0);
            }
            else{
                if (speed>0){
                    speed = max(speed-0.025*pow(speed,2)-0.005, 0.0);
                }
                else if (speed<0){
                    speed = min(speed+0.025*pow(speed,2)-0.005, 0.0);        
                }
            }   
        }
        else if (speed<1){
            speed = min(speed+0.05*pow(speed,2)+0.01, 1.0);
            flag_user_ctrl=false;
        }
        
        std::cout << "Time: " << input_gmr(0,0) << std::endl;
        //std::cout << "error: " << error << std::endl;

                
    }
    
    // Callback for the joint position
    void DmpRevFLim::pos_callback(const pr_msgs::msg::PRArrayH::SharedPtr pos_msg)
    {
        //DMP message and init time
        auto ref_dmp_q_msg = pr_msgs::msg::PRArrayH();
        ref_dmp_q_msg.init_time = this->get_clock()->now();
        auto ref_dmp_x_msg = pr_msgs::msg::PRArrayH();
        ref_dmp_x_msg.init_time = this->get_clock()->now();
        auto phase_dmp_msg = pr_msgs::msg::PRFloatH();
        phase_dmp_msg.init_time = this->get_clock()->now();
        auto phase_speed_dmp_msg = pr_msgs::msg::PRFloatH();
        phase_speed_dmp_msg.init_time = this->get_clock()->now();

        // Make sure we read the trajectory
        if (ref_read){

            // If this is the first iter, we make the first integration of the system
            if (first_iter){
                state_dmp = dmp->integrate();
                x_dmp = state_dmp.head(1)[0]; // Phase
                y_dmp = state_dmp.segment(1,n_dims); // Position
                // z_dmp = state_dmp.segment(n_dims+1,2*n_dims);
                first_iter = false;
            }
            
            else{
                // // ESTO VA EN EL CALLBACK DE LA FUERZA
                // norm_f = 0.0;
                // input_gmr(0,0) = -dmp->get_tau()/a_x*log(x_dmp);
                // mean_gmr = gmr.doRegression(input_gmr,sigma_gmr,inC,outC);
                // // std::cout << mean_gmr(0,gmr.get_dim()-2) << std::endl;
                // // std::cout << sqrt(sigma_gmr[0](gmr.get_dim()-2, gmr.get_dim()-2));
                // // Se calcula el error para la norma de la fuerza
                // error = (norm_f-mean_gmr(0, gmr.get_dim()-2))/sqrt(sigma_gmr[0](gmr.get_dim()-2, gmr.get_dim()-2));
                // std::cout << "Error: " << error << std::endl;
                // if (error > relative_error_limit){
                //     speed = max(speed-0.005*pow(speed,2)-0.001*(error-relative_error_limit), -1.0);
                // }  
                // else{
                //     speed = min(speed+0.005*pow(speed,2)+0.001, 1.0);
                // }  
                // /////////// HASTA AQUI /////////////

                phase_current(0) = x_dmp;
                //std::cout << phase_current << std::endl;
                
                if (x_dmp > phase_limit && !external_stop){

                    // Ahora mismo se define el slowdown a partir del error en la fuerza de aceleracion 
                    // (lo que le está costado llegar a la referencia)
                    // La dinámica dependerá de alpha_x y tau, por lo que se puede diseñar (modificar)
                    // el gain_slowdown para que la ganancia sea independiente de la dinámica

                    // slowdown = gain_slowdown*force.norm();
                    // if (n_iter<5000){ speed=min(speed+0.005, 1.0);}
                    // else{ speed=max(speed-0.005, -1.0);}

                    // Keep integrating as long as the phase is over the limit and update the data
                    state_dmp = dmp->integrate(speed);
                    x_dmp = state_dmp.head(1)[0]; // Phase
                    y_dmp = state_dmp.segment(1,n_dims); // Position
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
                phase_speed_dmp_msg.data = speed;

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

                
                // Prepare messages for publication
                ref_dmp_q_msg.header.stamp = pos_msg->header.stamp;
                ref_dmp_q_msg.header.frame_id = pos_msg->header.frame_id;

                phase_dmp_msg.header.stamp = pos_msg->header.stamp;
                phase_dmp_msg.header.frame_id = pos_msg->header.frame_id;

                phase_speed_dmp_msg.header.stamp = pos_msg->header.stamp;
                phase_speed_dmp_msg.header.frame_id = pos_msg->header.frame_id;

                ref_dmp_x_msg.header.stamp = pos_msg->header.stamp;
                ref_dmp_x_msg.header.frame_id = pos_msg->header.frame_id;
                
                ref_dmp_q_msg.current_time = this->get_clock()->now();
                publisher_q_->publish(ref_dmp_q_msg);

                ref_dmp_x_msg.current_time = this->get_clock()->now();
                publisher_x_->publish(ref_dmp_x_msg);

                phase_dmp_msg.current_time = this->get_clock()->now();
                publisher_phase_->publish(phase_dmp_msg);

                phase_speed_dmp_msg.current_time = this->get_clock()->now();
                publisher_phase_speed_->publish(phase_speed_dmp_msg);
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

    //External stop fuction
    void DmpRevFLim::external_stop_callback(const std_msgs::msg::Bool::SharedPtr external_stop_msg)
    {
        if (!external_stop)
            external_stop = external_stop_msg->data;

    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_dmp::DmpRevFLim)