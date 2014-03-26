#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
//driver
#include <KukaEthernetClient/KukaEthernetClient.h>
//actions
#include <ipa325_msgs/JobAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ipa325_msgs/LinMoveAction.h>
#include <ipa325_msgs/DigIOAction.h>
#include <ipa325_msgs/RobotMovementAction.h>
//publisher
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <boost/thread.hpp>


//**** TODO ****//
// Frame transformation via tf
//#include <tf/transform_listener.h>

#include <queue>


class EkiActionServer {
protected:

    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<ipa325_msgs::JobAction> job_as_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> traj_as_;
    actionlib::SimpleActionServer<ipa325_msgs::DigIOAction> digIo_as_;
    actionlib::SimpleActionServer<ipa325_msgs::LinMoveAction> pose_as_;
    actionlib::SimpleActionServer<ipa325_msgs::RobotMovementAction> robMov_as_;

    std::string action_name_;
    // create messages that are used to published feedback/result
    ipa325_msgs::JobFeedback jobfeedback_;
    ipa325_msgs::JobResult jobresult_;

    control_msgs::FollowJointTrajectoryFeedback trajfeedback_;
    control_msgs::FollowJointTrajectoryResult trajresult_;

    ipa325_msgs::LinMoveFeedback poseFeedback_;
    ipa325_msgs::LinMoveResult poseResult_;

    ipa325_msgs::DigIOFeedback ioFeedback_;
    ipa325_msgs::DigIOResult ioResult_;

    ipa325_msgs::RobotMovementFeedback robMovFeedback_;
    ipa325_msgs::RobotMovementResult robMovResult_;

    // Publisher
    ros::Publisher topicPub_JointState_;
    std::vector<std::string> joint_names_;

    //temporary tf publisher
    tf::TransformBroadcaster tfBroadcaster;
    KukaFrame robotPose;
    boost::thread* threadTfPublisher;

    //**** TODO ****//
    // Frame transformation
    //tf::TransformListener tf_listener_;

    // Kuka Communication specifics
    KukaEthernetClient kuka_client_;
    int received_msgID_;
    bool kukaCallbackReceived_;
    int pollingRate_;

public:

    EkiActionServer(std::string name) :
        job_as_(nh_, name+"_JobServer", boost::bind(&EkiActionServer::execute, this, _1), false),
        traj_as_(nh_, name+"_TrajServer", boost::bind(&EkiActionServer::executeTraj, this, _1), false),
        digIo_as_(nh_, name+"_DigIoServer", boost::bind(&EkiActionServer::executeIo, this, _1), false),
        pose_as_(nh_, name+"_PoseServer", boost::bind(&EkiActionServer::executePose, this, _1), false),
        robMov_as_(nh_, name+"_RobotMovementServer", boost::bind(&EkiActionServer::executeRobMov, this, _1), false),
        //**** TODO ****//
        //tf_listener_(ros::Duration(0.1)),
        action_name_(name),
        received_msgID_(-1),
        kukaCallbackReceived_(false)
    {
        //kuka_client_.Initialize("common/files/EKIServerFrame.xml", SocketAddress(config.KRC_ip_address, config.KRC_ip_port));

        std::string ip, xml_path;
        int port;
        bool autoConnect, dummyMode;

        if( !ros::param::get("~IP_address", ip) )
        {
            ROS_ERROR("Cannot find IP_address @ paramServer");
            nh_.shutdown();
        }

        if( !ros::param::get("~port", port) )
        {
            ROS_ERROR("Cannot find port @ paramServer");
            nh_.shutdown();
        }

        if( !ros::param::get("~krl_ethernet_xml", xml_path) )
        {
            ROS_ERROR("Cannot find krl_ethernet_xml @ paramServer");
            nh_.shutdown();
        }

        if( !ros::param::get("~dummyMode", dummyMode) ) dummyMode = false;
        if( !ros::param::get("~autoConnect", autoConnect) ) autoConnect = true;
        if( !ros::param::get("~pollingRate", pollingRate_) ) pollingRate_ = 100;

        // Get joint names
        XmlRpc::XmlRpcValue JointNamesXmlRpc;
        if (!ros::param::get("~joint_names", JointNamesXmlRpc))
        {
            ROS_ERROR("Parameter joint_names not set, shutting down node...");
            nh_.shutdown();
        }
        // Resize and assign of values to the joint_names_
        joint_names_.resize(JointNamesXmlRpc.size());
        for (int i = 0; i < JointNamesXmlRpc.size(); i++) {
            joint_names_[i] = (std::string) JointNamesXmlRpc[i];
        }

        topicPub_JointState_ = nh_.advertise<sensor_msgs::JointState> ("/joint_states", 1);

        /*********************************************/
        // for not really connecting to the robot!!!
        kuka_client_.setClientDummy(dummyMode);
        /*********************************************/

        ROS_INFO("Initializing KRC Ethernet Client");

        //if (!kuka_client_.Initialize("../common/files/EKI_Dummy.xml", SocketAddress("192.1.10.20", 49152), false))
	std::cout << xml_path;
        if (!kuka_client_.Initialize(xml_path, SocketAddress(ip, port), autoConnect))
        {
            int error = kuka_client_.getError();
            if ( error == 1)
            {
                ROS_ERROR("Connection to KUKA KRC failed");
            }
            else if (error == 2)
            {
                ROS_ERROR("Xml KRC definition file could not be parsed");
            }
            else if (error == 3)
            {
                ROS_ERROR("Connection failed AND Xml KRC definition file could not be parsed");
            }
        }
        else
        {
            ROS_INFO("Successfully connected to KUKA KRC");
        }


        boost::function<void (int)> fct( boost::bind( &EkiActionServer::kukaCallback, this, _1 ) );
        kuka_client_.setCallbackFcn(fct);

        job_as_.start();
        traj_as_.start();
        digIo_as_.start();
        pose_as_.start();
        robMov_as_.start();

        //start temporary tf publisher thread
        threadTfPublisher = new boost::thread(boost::bind(&EkiActionServer::workerTfPublisher, this));
    }

    ~EkiActionServer(void)
    {
    }

    void kukaCallback(int msgID)
    {
        ROS_INFO("KRC called back with MsgID %i done", msgID);

        received_msgID_ = msgID;

        kukaCallbackReceived_ = true;
        ROS_INFO("%s: Kuka Callback", action_name_.c_str());

        KukaAxis curAx = kuka_client_.getCurrentAxis();

        //Publishing jointstate
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.header.stamp = ros::Time::now();
        joint_state_msg.name = joint_names_;
        joint_state_msg.position = curAx.toStdVector();

        topicPub_JointState_.publish(joint_state_msg);

        robotPose = kuka_client_.getCurrentFrame();
    }

    //temporary tf publisher
    void workerTfPublisher() {
    	ROS_INFO("start tf publisher");
        ros::Rate rate(10.0);
        while (nh_.ok()) {
            tf::Transform transform;
            tf::Quaternion quaternion;
            quaternion.setEuler(robotPose.a[4] * 0.0174532925, robotPose.a[5] * 0.0174532925, robotPose.a[3] * 0.0174532925);
            transform.setOrigin(tf::Vector3(robotPose.a[0] / 1000, robotPose.a[1] / 1000, robotPose.a[2] / 1000));
            transform.setRotation(quaternion);

        	tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "tool"));

            rate.sleep();
        }
    }

    void execute(const ipa325_msgs::JobGoalConstPtr &goal)
    {
        // helper variables
        ros::Rate r(pollingRate_);
        kukaCallbackReceived_ = false;

        int msgId = 0;
        // publish info to the console for the user
        if (goal->msgId != 0)
        {
            msgId = goal->msgId;
            ROS_INFO("Received Job: %i, MsgId: %i", goal->jobId, msgId);
        }
        else
        {
            msgId = kuka_client_.getMessageID();
            ROS_WARN("Received Job: %i without valid MsgId (MsgId=%i) -> MsgId IS SET TO: %i", goal->jobId, goal->msgId, msgId);
        }

        // push_back the seeds for the job sequence
        jobfeedback_.stage=1;
        jobfeedback_.msgId = msgId;
        jobfeedback_.jobId = goal->jobId;

        // publish the feedback
        job_as_.publishFeedback(jobfeedback_);


        //        kuka_client_.addMessage(goal->msgId, goal->jobId, goal->param1, goal->param2, false);
        KukaAxis axis(goal->axis);
        KukaFrame frame(goal->pose);
        kuka_client_.addMessage(msgId, goal->jobId, goal->param1, goal->param2, axis, frame, goal->linVel, goal->axVel, false);

        jobresult_.jobId = goal->jobId;
        jobfeedback_.stage = 2;


        while (!kukaCallbackReceived_ && ros::ok() && !job_as_.isPreemptRequested())
        {
            // publish the feedback
            job_as_.publishFeedback(jobfeedback_);
            r.sleep();
        }

        if (job_as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            jobresult_.msgId = msgId;
            jobresult_.errorcode = 1;
            // set the action state to preempted
            job_as_.setPreempted(jobresult_);
        }
        else
        {
            jobresult_.msgId = received_msgID_; //to get from KRC Feedback
            jobresult_.errorcode = 0;

            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            job_as_.setSucceeded(jobresult_);
        }
    }

    void executeTraj(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
    {
        // helper variables
        ros::Rate r(pollingRate_);
        kukaCallbackReceived_ = false;

        std::queue<int> msgQueue;

        // publish the feedback
        traj_as_.publishFeedback(trajfeedback_); //??

        ROS_INFO("FollowjointTrajectorygoal received containing %i waypoints.", goal->trajectory.points.size()) ;

        for (u_int i = 0; i < goal->trajectory.points.size(); i++)
        {
            if (goal->trajectory.points[i].velocities.size() > 1)
            {
                ROS_WARN("Only first velocity is accepted and defines all axes!");
            }
            float overallVelocity = goal->trajectory.points[i].velocities[0];

            int current_msgId = kuka_client_.getMessageID();
            msgQueue.push(current_msgId);

            kuka_client_.movePTP(current_msgId,
                                 goal->trajectory.points[i].positions[0],
                                 goal->trajectory.points[i].positions[1],
                                 goal->trajectory.points[i].positions[2],
                                 goal->trajectory.points[i].positions[3],
                                 goal->trajectory.points[i].positions[4],
                                 goal->trajectory.points[i].positions[5],
                                 overallVelocity);
        }

        // waiti until execution of the action has finished
        while (!msgQueue.empty() && ros::ok() && !traj_as_.isPreemptRequested())
        {
            if(kukaCallbackReceived_)
            {
                kukaCallbackReceived_ = false;
                // work through msgId-Queue;
                if(msgQueue.front() != received_msgID_)
                {
                    ROS_WARN("QueueFront = %i; Recv = %i -> Indicating sync problem! Send Actions does not match responses by KRC", msgQueue.front(), received_msgID_);
                }
                msgQueue.pop();
            }

            // publish the feedback
            traj_as_.publishFeedback(trajfeedback_);
            r.sleep();
        }

        if (traj_as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            traj_as_.setPreempted(trajresult_);
        }
        else
        {
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            traj_as_.setSucceeded(trajresult_);
        }
    }

    void executeRobMov(const ipa325_msgs::RobotMovementGoalConstPtr &goal)
    {
        ROS_DEBUG_ONCE("Implementation of RobotMovementAction Server in EKI is still in alpha state!");

        // helper variables
        ros::Rate r(pollingRate_);
        kukaCallbackReceived_ = false;

        std::queue<int> msgQueue;

        // publish the feedback
        robMov_as_.publishFeedback(robMovFeedback_); //??

        ROS_INFO("RobotMovementActionServer received containing %i waypoints.", goal->points.size()) ;

        for (u_int i = 0; i < goal->points.size(); i++)
        {
            int current_msgId = kuka_client_.getMessageID();

            //**** TODO ****//
            // TODO Tranformation to Base Frame if needed
            if (goal->points[i].frame_id != "1")
            {
                ROS_ERROR("Frame Id differs from global ('1') in viaPoint[%i], THIS MUST BE TESTED!!!!!", i);
                //geometry_msgs::PoseStamped goal_pose, base_pose;
                //fromKukaToQuanterions(goal->points[i].pose, goal_pose);
                //try
                //{
                //    tf_listener_.transformPose("ipa325_0_joint", ros::Time::now(), goal_pose, goal->points[i].frame_id, base_pose);
                //}
                //catch(tf::TransformException& ex){
                //  ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
                //}
                ROS_ERROR("Transformation is ignored!");
            }

            KukaFrame viaPoint_base(goal->points[i].pose);

            if (goal->points[i].movement_type == 0) // PTP Movement to Frame in base coordinate system
            {
                kuka_client_.movePTP(current_msgId,
                                     viaPoint_base,
                                     goal->points[i].linVel);
            }
            else if (goal->points[i].movement_type == 1) // LIN Movement to Frame in base coordinate system
            {
                kuka_client_.moveLIN(current_msgId,
                                     viaPoint_base,
                                     goal->points[i].linVel);
            }
            else
            {
                ROS_ERROR("Unidentifiable movement_type found in goal.point[%i)", i);
                robMov_as_.setPreempted(robMovResult_);
                return;
            }

            msgQueue.push(current_msgId);
        }


        // waiti until execution of the action has finished
        while (!msgQueue.empty() && ros::ok() && !robMov_as_.isPreemptRequested())
        {
            if(kukaCallbackReceived_)
            {
                kukaCallbackReceived_ = false;
                // work through msgId-Queue;
                if(msgQueue.front() != received_msgID_)
                {
                    ROS_WARN("QueueFront = %i; Recv = %i -> Indicating sync problem! Send Actions does not match responses by KRC", msgQueue.front(), received_msgID_);
                }
                msgQueue.pop();
            }

            // publish the feedback
            robMov_as_.publishFeedback(robMovFeedback_);
            r.sleep();
        }

        if (robMov_as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            robMov_as_.setPreempted(robMovResult_);
        }
        else
        {
            ROS_INFO("%s: Succeeded", action_name_.c_str());
//            KukaFrame finalFrame = kuka_client_.getCurrentFrame();
//            robMovResult_.final_actual.pose[0] = finalFrame.a[0];
//            robMovResult_.final_actual.pose[1] = finalFrame.a[1];
//            robMovResult_.final_actual.pose[2] = finalFrame.a[2];
//            robMovResult_.final_actual.pose[3] = finalFrame.a[3];
//            robMovResult_.final_actual.pose[4] = finalFrame.a[4];
//            robMovResult_.final_actual.pose[5] = finalFrame.a[5];

            // set the action state to succeeded
            robMov_as_.setSucceeded(robMovResult_);
        }
    }

    void executePose(const ipa325_msgs::LinMoveGoalConstPtr &goal)
    {
        // helper variables
        ros::Rate r(pollingRate_);
        kukaCallbackReceived_ = false;

        if (goal->relativeMovement)
        {
            ROS_ERROR("Relative movement not implemented yet!");
            pose_as_.setPreempted(poseResult_);
        }

        // publish the feedback
        pose_as_.publishFeedback(poseFeedback_); //??

        ROS_INFO("LinMovegoal received containing goal (%f, %f, %f, %f, %f, %f).", goal->pose[0], goal->pose[1], goal->pose[2], goal->pose[3], goal->pose[4], goal->pose[5] ) ;


        int current_msgId = kuka_client_.getMessageID();

        kuka_client_.moveLIN(current_msgId, KukaFrame(goal->pose), goal->linVel);

        // waiti until execution of the action has finished
        while (!kukaCallbackReceived_ && ros::ok() && !pose_as_.isPreemptRequested())
        {
            // publish the feedback
            pose_as_.publishFeedback(poseFeedback_);
            r.sleep();
        }

        if (pose_as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            pose_as_.setPreempted(poseResult_);
        }
        else
        {
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            pose_as_.setSucceeded(poseResult_);
        }
    }

    void executeIo(const ipa325_msgs::DigIOGoalConstPtr &goal)
    {
        // helper variables
        ros::Rate r(pollingRate_);
        kukaCallbackReceived_ = false;

        // publish the feedback
        ioFeedback_.ioNr = goal->ioNr;
        ioFeedback_.stage = 1;
        digIo_as_.publishFeedback(ioFeedback_); //??

        ROS_INFO("DigIo goal received.") ;

        int current_msgId = goal->msgId;
        if (current_msgId <= 0)
        {
            current_msgId = kuka_client_.getMessageID();
            ROS_WARN("MessageID in goal was not valid and changed to %i", current_msgId);
        }

        kuka_client_.setIo(current_msgId, goal->ioNr, goal->newState);

        // May reflect return values by KRC?
        ioResult_.ioNr = goal->ioNr;
        ioResult_.state = goal->newState;

        // wait until execution of the action has finished
        while (!kukaCallbackReceived_ && ros::ok() && !digIo_as_.isPreemptRequested())
        {
            // publish the feedback
            digIo_as_.publishFeedback(ioFeedback_);
            r.sleep();
        }

        if (digIo_as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            digIo_as_.setPreempted(ioResult_);
        }
        else
        {
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            digIo_as_.setSucceeded(ioResult_);
        }
    }

    //**** TODO ****//
//    void fromKukaToQuanterions(const std::vector<float>& i_goal_pose, geometry_msgs::PoseStamped& o_goal_quant)
//    {
//        o_goal_quant.pose.position.x = i_goal_pose[0];
//        o_goal_quant.pose.position.y = i_goal_pose[1];
//        o_goal_quant.pose.position.z = i_goal_pose[2];
//        tf::Quaternion q = tf::createQuaternionFromRPY(
//                    (double)(i_goal_pose[3]),
//                    (double)(i_goal_pose[4]),
//                    (double)(i_goal_pose[5]));
//        o_goal_quant.pose.orientation.x = q.x();
//        o_goal_quant.pose.orientation.y = q.y();
//        o_goal_quant.pose.orientation.z = q.w();
//        o_goal_quant.pose.orientation.w = q.z();
//    }

//    void fromQuanterionsToKuka(const geometry_msgs::PoseStamped& i_goal_quant, std::vector<float>& o_goal_pose)
//    {
//        return;
//    }

    /*
    Quanterion to RPY:
    =======================================
    tf::Quaternion rotation;
    double yaw ,pitch ,roll;
    tf::Matrix3x3(rotation).getEulerYPR(yaw,pitch,roll);

    RPY to Quanterion:
    =======================================
    tf::Quaternion rotation;
    double roll ,pitch ,yaw;
    rotation.setRPY(roll, pitch, yaw);
     */
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "EkiDriver");

    EkiActionServer jobServer(ros::this_node::getName());
    ros::spin();

    return 0;
}
