// ROS includes
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <kr16_driver/kr16_nodeConfig.h>

// ROS message includes
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>




#include <kr16_node_common.cpp>


class kr16_node_ros
{
	public:
		ros::NodeHandle n_;
		
		dynamic_reconfigure::Server<kr16_driver::kr16_nodeConfig> server;
  		dynamic_reconfigure::Server<kr16_driver::kr16_nodeConfig>::CallbackType f;
		

		ros::Publisher joint_states_;
		ros::Publisher state_;
		

	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_follow_joint_trajectory_action_;
        
 
        kr16_node_data component_data_;
        kr16_node_config component_config_;
        kr16_node_impl component_implementation_;

        kr16_node_ros()
        :as_follow_joint_trajectory_action_(n_, "follow_joint_trajectory_action", boost::bind(&kr16_node_impl::callback_follow_joint_trajectory_action_, &component_implementation_, _1, &as_follow_joint_trajectory_action_), false)
        {
       	
  			f = boost::bind(&kr16_node_ros::configure_callback, this, _1, _2);
  			server.setCallback(f);
 			as_follow_joint_trajectory_action_.start();
        	
        
				joint_states_ = 	n_.advertise<sensor_msgs::JointState>("joint_states", 1);
				state_ = 	n_.advertise<control_msgs::JointTrajectoryControllerState>("state", 1);
  	

				n_.param("robot_ip_address", component_config_.robot_ip_address, (std::string)"192.1.10.20");
				n_.param("robot_description", component_config_.robot_description, (std::string)"/home/ros/");
				n_.param("robot_port", component_config_.robot_port, (int)49152);
            
        }
        
		
		void configure_callback(kr16_driver::kr16_nodeConfig &config, uint32_t level) 
		{
				component_config_.robot_ip_address = config.robot_ip_address;
				component_config_.robot_description = config.robot_description;
				component_config_.robot_port = config.robot_port;
		}

        void configure()
        {
			component_implementation_.configure(component_config_);
        }

        void update()
        {
            component_implementation_.update(component_data_, component_config_);
				joint_states_.publish(component_data_.out_joint_states);
				state_.publish(component_data_.out_state);
    
        }
 
};

int main(int argc, char** argv)
{

	ros::init(argc, argv, "kr16_node");

	kr16_node_ros node;
    node.configure();

	
 	ros::Rate loop_rate(10.0); // Hz

	while(node.n_.ok())
	{
        node.update();
		loop_rate.sleep();
		ros::spinOnce();
	}
	
    return 0;
}
