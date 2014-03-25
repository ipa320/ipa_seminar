// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <kr16_test_listener/kr16_test_clientConfig.h>

// ROS message includes
#include <control_msgs/JointTrajectoryControllerState.h>




#include <kr16_test_client_common.cpp>


class kr16_test_client_ros
{
	public:
		ros::NodeHandle n_;
		
		dynamic_reconfigure::Server<kr16_test_listener::kr16_test_clientConfig> server;
  		dynamic_reconfigure::Server<kr16_test_listener::kr16_test_clientConfig>::CallbackType f;
		

		

		ros::Subscriber state_;
        
 
        kr16_test_client_data component_data_;
        kr16_test_client_config component_config_;
        kr16_test_client_impl component_implementation_;

        kr16_test_client_ros()
        {
       	
  			f = boost::bind(&kr16_test_client_ros::configure_callback, this, _1, _2);
  			server.setCallback(f);
        	
        
					state_ =  n_.subscribe("state", 1, &kr16_test_client_ros::topicCallback_state, this);
  	

				n_.param("joint_position_1", component_config_.joint_position_1, (std::string)"-35 -98 108 -0.2 80 0.4 0");
				n_.param("joint_position_2", component_config_.joint_position_2, (std::string)"-25 -95 105 -0.5 80 10.5 0");
            
        }
        
        void topicCallback_state(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
		{
            component_data_.in_state = *msg;
            
        }
		
		void configure_callback(kr16_test_listener::kr16_test_clientConfig &config, uint32_t level) 
		{
				component_config_.joint_position_1 = config.joint_position_1;
				component_config_.joint_position_2 = config.joint_position_2;
		}

        void configure()
        {
			component_implementation_.configure(component_config_);
        }

        void update()
        {
            component_implementation_.update(component_data_, component_config_);
    
        }
 
};

int main(int argc, char** argv)
{

	ros::init(argc, argv, "kr16_test_client");

	kr16_test_client_ros node;
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
