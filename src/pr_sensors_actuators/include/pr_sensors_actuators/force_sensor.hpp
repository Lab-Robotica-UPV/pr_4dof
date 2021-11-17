#ifndef PR_SENSORS_ACTUATORS__FORCE_SENSOR_HPP_
#define PR_SENSORS_ACTUATORS__FORCE_SENSOR_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <cstdint>
#include <unistd.h>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_array.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_force_state.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"

#define PORT 49152 /* Port the Net F/T always uses */
#define COMMAND 2 /* Command code 2 starts streaming */
#define NUM_SAMPLES 1 /* Will send 1 sample before stopping */


namespace pr_sensors_actuators
{
    class ForceSensor : public rclcpp::Node
    {
        public:
            //PR_SENSORS_ACTUATORS_PUBLIC
            explicit ForceSensor(const rclcpp::NodeOptions & options);

        protected:
            //Change for a topic callback to sync with encoder
            void timer_callback();
            //Synchronize with the encoder
            void topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr q_msg);

        private:
            typedef struct response_struct {
	            uint32_t rdt_sequence;
	            uint32_t ft_sequence;
	            uint32_t status;
	            int32_t FTData[6];
            } RESPONSE;

            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_;
            rclcpp::Publisher<pr_msgs::msg::PRForceState>::SharedPtr publisher_;
            rclcpp::Publisher<pr_msgs::msg::PRForceState>::SharedPtr publisher_sync_;
            rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr publisher_accelstamped_;
            rclcpp::TimerBase::SharedPtr timer_;
            int socketHandle;
            uint8_t request[8];	/* The request data sent to the Net F/T. */
            struct hostent *he;
            struct sockaddr_in addr;	/* Address of Net F/T. */
            int err;
            RESPONSE resp;
            uint8_t response[36];
            int i_fuerza,j_fuerza;

            bool calibration;
            bool noise_threshold;
            std::vector<double> std_noise;
            pr_msgs::msg::PRForceState force_msg;
    };

}   // Namespace pr_sensors_actuators


#endif // PR_SENSORS_ACTUATORS__FORCE_SENSOR_HPP_