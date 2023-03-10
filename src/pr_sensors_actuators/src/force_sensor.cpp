#include "pr_sensors_actuators/force_sensor.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace pr_sensors_actuators
{
    // FORCE SENSOR COMPONENT
    ForceSensor::ForceSensor(const rclcpp::NodeOptions & options)
    : Node("force_sensor", options)
    {
      this->declare_parameter<bool>("calibration", true);
      this->declare_parameter<bool>("noise_threshold", false);
      this->get_parameter("calibration", calibration);
      this->get_parameter("noise_threshold",noise_threshold);

        //Configuración del socket
        RCLCPP_INFO(this->get_logger(), "Configurando sensor");
        socketHandle = socket(AF_INET, SOCK_DGRAM, 0);
        if (socketHandle == -1) {
		    RCLCPP_ERROR(this->get_logger(), "Error 0");
		    exit(1);
	      }

      /* Timeout configuration. */
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1000;  //1 ms

      // Initialization of std_noise
      std_noise.resize(6);
      std_noise[0] = 0.5; //0.036; //Original basado en experimento ruido
      std_noise[1] = 0.0675;
      std_noise[2] = 0.9;
      std_noise[3] = 0.03; //0.0045; //Original basado en experimento ruido
      std_noise[4] = 0.03; //0.0040; //Original basado en experimento ruido
      std_noise[5] = 0.03;


      // Sensor calibration

      if (calibration){
        *(uint16_t*)&request[0] = htons(0x1234); /* standard header. */
        *(uint16_t*)&request[2] = htons(0x0042); /* per table 9.1 in Net F/T user manual. Ese es el valor de reset BIAS */
        *(uint32_t*)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */
        /*for (int i=0; i<8; i++){
          std::cout << static_cast<int>(request[i]) << " " << std::endl;
        }*/

        
        /* Sending the request. */
          he = gethostbyname("192.168.1.1");
          memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
          addr.sin_family = AF_INET;
          addr.sin_port = htons(PORT);

          err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) ); //Faltaba esta línea, por lo que no estaba conectado el socket
          if (err == -1) {
            RCLCPP_ERROR(this->get_logger(), "Error 1");
            exit(2);
          }

          send( socketHandle, request, 8, 0 );

          usleep(1000000);
      }

        // Prepare for reading data

        *(uint16_t*)&request[0] = htons(0x1234); /* standard header. */
        *(uint16_t*)&request[2] = htons(COMMAND); /* per table 9.1 in Net F/T user manual. */
        *(uint32_t*)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */

        /*for (int i=0; i<8; i++){
          std::cout << static_cast<int>(request[i]) << " " << std::endl;
        }*/

        /* Sending the request. */
        he = gethostbyname("192.168.1.1");
        memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
        addr.sin_family = AF_INET;
        addr.sin_port = htons(PORT);

        err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
        if (err == -1) {
          RCLCPP_ERROR(this->get_logger(), "Error 2");
          exit(2);
        }

        setsockopt(socketHandle, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv,sizeof tv);

        // Define bias if calibration is activated
        if (calibration){

          // Samples to take average from
          samples_bias.resize(num_samples_bias, 6);

          int sample_count=0;
          
          // Fill samples_bias with this loop
          while (sample_count<num_samples_bias){
            
            if (send(socketHandle, request, 8, 0 )<0) {
              continue;
            }

            if (recv(socketHandle, response, 36, 0 )<0) {
              continue;
            }

            resp.rdt_sequence = ntohl(*(uint32_t*)&response[0]);
            resp.ft_sequence = ntohl(*(uint32_t*)&response[4]);
            resp.status = ntohl(*(uint32_t*)&response[8]);
            for( i_fuerza = 0; i_fuerza < 6; i_fuerza++ ) {
                resp.FTData[i_fuerza] = ntohl(*(int32_t*)&response[12 + i_fuerza * 4]);
            }


            samples_bias(sample_count,0) = 1.0*resp.FTData[0]/1000000.0;
            samples_bias(sample_count,1) = 1.0*resp.FTData[1]/1000000.0;
            samples_bias(sample_count,2) = 1.0*resp.FTData[2]/1000000.0;
            samples_bias(sample_count,3) = 1.0*resp.FTData[3]/1000000.0;
            samples_bias(sample_count,4) = 1.0*resp.FTData[4]/1000000.0;
            samples_bias(sample_count,5) = 1.0*resp.FTData[5]/1000000.0; 

            sample_count++;
            usleep(1000);
          }
          // Calculate mean
          bias = samples_bias.colwise().mean();
        }

        timer_ = this->create_wall_timer(5ms, std::bind(&ForceSensor::timer_callback, this));
        
        publisher_ = this->create_publisher<pr_msgs::msg::PRForceState>("force_state", 1);
        publisher_end_ = this->create_publisher<std_msgs::msg::Bool>(
            "end_flag",
            1//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_default)
        );
        publisher_wrenchstamped_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("force_state_std", 1);
        publisher_sync_ = this->create_publisher<pr_msgs::msg::PRForceState>("force_state_sync", 1);

        RCLCPP_INFO(this->get_logger(), "Sensor configurado");

        subscription_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "joint_position",
            1,
            std::bind(&ForceSensor::topic_callback, this, _1)
        );

        force_msg = pr_msgs::msg::PRForceState();


    }

    void ForceSensor::timer_callback()
    {
      // ForceSensor message and init time
      //auto force_msg = pr_msgs::msg::PRForceState();
      force_msg.init_time = this->get_clock()->now();

        //RCLCPP_INFO(this->get_logger(), "Sending request");
        if (send(socketHandle, request, 8, 0 )<0) {
          std::cout << "Time out request!!" << std::endl;
          disconnect_callback();
          return;
        }

        if (recv(socketHandle, response, 36, 0 )<0) {
          std::cout << "Time out response!!" << std::endl;
          disconnect_callback();
          return;
        }
        resp.rdt_sequence = ntohl(*(uint32_t*)&response[0]);
        resp.ft_sequence = ntohl(*(uint32_t*)&response[4]);
        resp.status = ntohl(*(uint32_t*)&response[8]);
        for( i_fuerza = 0; i_fuerza < 6; i_fuerza++ ) {
            resp.FTData[i_fuerza] = ntohl(*(int32_t*)&response[12 + i_fuerza * 4]);
        }

        iter_disconnected = 0;

        // Subtract bias. If not calibrated, bias will remain 0
        force_msg.force[0] = 1.0*resp.FTData[0]/1000000.0 - bias(0);
        force_msg.force[1] = 1.0*resp.FTData[1]/1000000.0 - bias(1);
        force_msg.force[2] = 1.0*resp.FTData[2]/1000000.0 - bias(2);
        force_msg.momentum[0] = 1.0*resp.FTData[3]/1000000.0 - bias(3);
        force_msg.momentum[1] = 1.0*resp.FTData[4]/1000000.0 - bias(4);
        force_msg.momentum[2] = 1.0*resp.FTData[5]/1000000.0 - bias(5);

        // Threshold: If activated, filters 4 times the standard deviation
        if (noise_threshold){
          for (int i=0; i<3; i++){
            if (abs(force_msg.force[i]) < 4*std_noise[i]){
              force_msg.force[i] = 0;
            }
            if (abs(force_msg.momentum[i]) < 4*std_noise[i+3]){
              force_msg.momentum[i] = 0;
            } 
          }
        }
        force_msg.header.stamp = this->get_clock()->now();
        force_msg.current_time = force_msg.header.stamp;

        publisher_->publish(force_msg);

        auto force_msg_ws = geometry_msgs::msg::WrenchStamped();

        // Subtract bias. If not calibrated, bias will remain 0
        force_msg_ws.wrench.force.x = 1.0*resp.FTData[0]/1000000.0 - bias(0);
        force_msg_ws.wrench.force.y = 1.0*resp.FTData[1]/1000000.0 - bias(1);
        force_msg_ws.wrench.force.z = 1.0*resp.FTData[2]/1000000.0 - bias(2);
        force_msg_ws.wrench.torque.x = 1.0*resp.FTData[3]/1000000.0 - bias(3);
        force_msg_ws.wrench.torque.y = 1.0*resp.FTData[4]/1000000.0 - bias(4);
        force_msg_ws.wrench.torque.z = 1.0*resp.FTData[5]/1000000.0 - bias(5);

        force_msg_ws.header.stamp = this->get_clock()->now();
        publisher_wrenchstamped_->publish(force_msg_ws);

        // RCLCPP_INFO(this->get_logger(), "Sensor: %f %f %f %f %f %f",force_msg.force[0], 
        //                                                             force_msg.force[1], 
        //                                                             force_msg.force[2], 
        //                                                             force_msg.momentum[0],
        //                                                             force_msg.momentum[1],
        //                                                             force_msg.momentum[2]);
    }

    void ForceSensor::topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr q_msg)
    {
      auto force_msg_sync = pr_msgs::msg::PRForceState();
      force_msg_sync.init_time = this->get_clock()->now();

      force_msg_sync.force[0] = force_msg.force[0];
      force_msg_sync.force[1] = force_msg.force[1];
      force_msg_sync.force[2] = force_msg.force[2];
      force_msg_sync.momentum[0] = force_msg.momentum[0];
      force_msg_sync.momentum[1] = force_msg.momentum[1];
      force_msg_sync.momentum[2] = force_msg.momentum[2];

      force_msg_sync.header.stamp = q_msg->header.stamp;
      force_msg_sync.header.frame_id = q_msg->header.frame_id;

      force_msg_sync.current_time = this->get_clock()->now();
      publisher_sync_->publish(force_msg_sync);

    }

    void ForceSensor::disconnect_callback(){
      iter_disconnected++;
      if (iter_disconnected >= 5){
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
RCLCPP_COMPONENTS_REGISTER_NODE(pr_sensors_actuators::ForceSensor)