#ifndef PR_ILC_PD_HPP_
#define PR_ILC_PD_HPP_

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"

#include "eigen3/Eigen/Dense"

#include "pr_msgs/msg/pr_array_h.hpp"


namespace pr_ilc
{
    class IlcPD : public rclcpp::Node
    {
        public:
            explicit IlcPD(const rclcpp::NodeOptions & options);
            ~IlcPD();

        protected:
            void controller_callback(const pr_msgs::msg::PRArrayH::ConstPtr& ref_msg,
                                     const pr_msgs::msg::PRArrayH::ConstPtr& pos_msg);

        private:

            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_ref;
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_pos;

            
            typedef message_filters::sync_policies::ApproximateTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH> SyncPolicy;
            /*
            typedef message_filters::sync_policies::ExactTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH> SyncPolicy;
            */
            typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
            std::shared_ptr<Synchronizer> sync_;


            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;

            std::string ref_path, load_path, save_path;

            Eigen::MatrixXd e_ant, u_ant, ref_matrix;
            
            Eigen::Vector4d pos, ref, e_init, e_k, u_k, e_ant_k, e_ant_k1, u_ant_k;
       
            int n_ref, iter;

            // Hay que meter a mano el numero de repeticiones que llevamos en esa trayectoria
            double traj_repetitions, alpha;

            std::vector<double> Kp, Kd;
            
    };

}

#endif // PR_ILC_PD_HPP_