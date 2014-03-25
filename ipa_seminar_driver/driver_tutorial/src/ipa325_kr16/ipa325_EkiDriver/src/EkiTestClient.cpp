
#include <ros/ros.h>
// general actionlib includes //
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// include of the action to be sent //
#include <ipa325_msgs/JobAction.h>
#include <ipa325_msgs/LinMoveAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ipa325_msgs/RobotMovementAction.h>

#define TEST_JOB_ACTION 0
#define TEST_LINMOV_ACTION 0
#define TEST_FOLLOWJOINT_ACTION 1
#define TEST_ROBMOV_ACTION 0


int main (int argc, char **argv)
{
    // *********************//
    // Initializing rosnode //
    // *********************//

    // announcing node to roscore
    ros::init(argc, argv, "test_action_client");

    // create the action client
    // true causes the client to spin its own thread

#if TEST_JOB_ACTION
    actionlib::SimpleActionClient<ipa325_msgs::JobAction> ac0("EkiDriver_JobServer", true);

    ROS_INFO("Waiting for action JobServer to start.");
    ac0.waitForServer(); //will wait for infinite time

    // *************************//
    // define content of action //
    // *************************//

    ipa325_msgs::JobGoal goal0;
    goal0.msgId = 1;
    goal0.jobId = 60;
    goal0.param1 = 1;
    goal0.param2 = 2;

    goal0.axis.clear();
    goal0.axis.push_back(7);
    goal0.axis.push_back(8);
    goal0.axis.push_back(9);
    goal0.axis.push_back(10);
    goal0.axis.push_back(11);
    goal0.axis.push_back(12);
    goal0.axVel = 25; //%

    // ********************************//
    // sending of goal to actionserver //
    // ********************************//

    // send the goal to the action server
    ac0.sendGoal(goal0);
    ROS_INFO("The JobGoal was sent");

    // ********************************//
    // waiting for reply by act-server //
    // ********************************//

    // wait for the action to return
    ROS_WARN("Waiting for jobAction to return");
    bool finished_before_timeout = ac0.waitForResult(ros::Duration(15.0));

    // handling possible timeout of actionserver reply
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac0.getState();
        ROS_WARN("Action finished: %s",state.toString().c_str());
    }
    else
    {
        ROS_WARN("Action did not finish before the time out.");
    }
#endif

#if TEST_LINMOV_ACTION
    actionlib::SimpleActionClient<ipa325_msgs::LinMoveAction> ac1("EkiDriver_PoseServer", true);

    ROS_INFO("Waiting for action PoseServer to start.");
    ac1.waitForServer(); //will wait for infinite time

    /// New Goal

    // declaring goal of the action
    ipa325_msgs::LinMoveGoal goal1;

    // Leave out if not wanted
    goal1.pose.clear();
    goal1.pose.push_back(521.01);
    goal1.pose.push_back(157);
    goal1.pose.push_back(636);
    goal1.pose.push_back(170);
    goal1.pose.push_back(1.5);
    goal1.pose.push_back(180);

    goal1.linVel = 0.5; //m/s

    // ********************************//
    // sending of goal to actionserver //
    // ********************************//

    // send the goal to the action server
    ac1.sendGoal(goal1);
    ROS_INFO("The LinMovegoal was sent");

    // ********************************//
    // waiting for reply by act-server //
    // ********************************//

    // wait for the action to return
    ROS_INFO("Waiting for action to return");
    finished_before_timeout = ac1.waitForResult(ros::Duration(60.0));

    // handling possible timeout of actionserver reply
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac1.getState();
        ROS_WARN("Action finished: %s",state.toString().c_str());
    }
    else
    {
        ROS_WARN("Action did not finish before the time out.");
    }
#endif

#if TEST_FOLLOWJOINT_ACTION

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac2("EkiDriver_TrajServer", true);

    ROS_INFO("Waiting for action TrajServer to start.");
    ac2.waitForServer(); //will wait for infinite time

    /// New Goal

    // declaring goal of the action
    control_msgs::FollowJointTrajectoryGoal goal2;
    // We will have two waypoints in this goal trajectory
    goal2.trajectory.points.resize(2);

    int ind = 0;
    goal2.trajectory.points[ind].positions.resize(7);
    goal2.trajectory.points[ind].positions[0] = -35;
    goal2.trajectory.points[ind].positions[1] = -98;
    goal2.trajectory.points[ind].positions[2] = 108;
    goal2.trajectory.points[ind].positions[3] = -0.2;
    goal2.trajectory.points[ind].positions[4] = 80;
    goal2.trajectory.points[ind].positions[5] = 0.4;
    goal2.trajectory.points[ind].positions[6] = 0.0;
    // Velocities
    goal2.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal2.trajectory.points[ind].velocities[j] = 10.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal2.trajectory.points[ind].time_from_start = ros::Duration(60.0);

    // Second trajectory point
    // Positions
    ind += 1;
    goal2.trajectory.points[ind].positions.resize(7);
    goal2.trajectory.points[ind].positions[0] = -25;
    goal2.trajectory.points[ind].positions[1] = -95;
    goal2.trajectory.points[ind].positions[2] = 105;
    goal2.trajectory.points[ind].positions[3] = -0.5;
    goal2.trajectory.points[ind].positions[4] = 80;
    goal2.trajectory.points[ind].positions[5] = 10.5;
    goal2.trajectory.points[ind].positions[6] = 0;
    // Velocities
    goal2.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal2.trajectory.points[ind].velocities[j] = 10.0;
    }
    // To be reached 2 seconds after starting along the trajectory
    goal2.trajectory.points[ind].time_from_start = ros::Duration(60.0);


    // ********************************//
    // sending of goal to actionserver //
    // ********************************//

    // send the goal to the action server
    ac2.sendGoal(goal2);
    ROS_INFO("The FollowJointgoal was sent");

    // ********************************//
    // waiting for reply by act-server //
    // ********************************//

    // wait for the action to return
    ROS_INFO("Waiting for action to return");
    bool finished_before_timeout = ac2.waitForResult(ros::Duration(60.0));

    // handling possible timeout of actionserver reply
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac2.getState();
        ROS_WARN("Action finished: %s",state.toString().c_str());
    }
    else
    {
        ROS_WARN("Action did not finish before the time out.");
    }

#endif

#if TEST_ROBMOV_ACTION

    actionlib::SimpleActionClient<ipa325_msgs::RobotMovementAction> ac3("EkiDriver_RobotMovementServer", true);

    ROS_INFO("Waiting for action RobMovServer to start.");
    ac3.waitForServer(); //will wait for infinite time

    /// New Goal

    // declaring goal of the action
    ipa325_msgs::RobotMovementGoal goal3;
    // We will have two waypoints in this goal trajectory
    goal3.points.resize(2);

    int ind2 = 0;
    goal3.points[ind2].pose.resize(6);
    goal3.points[ind2].pose[0] = -35;
    goal3.points[ind2].pose[1] = -98;
    goal3.points[ind2].pose[2] = 108;
    goal3.points[ind2].pose[3] = -0.2;
    goal3.points[ind2].pose[4] = 80;
    goal3.points[ind2].pose[5] = 0.4;

    //type
    goal3.points[ind2].movement_type = 0;

    //frame identifier
    goal3.points[ind2].frame_id = "1";

    // Velocity
    goal3.points[ind2].linVel = 0.5;

    // Acc
    goal3.points[ind2].linAcc = 0.2;


    // Second trajectory point
    // Positions
    ind2 += 1;
    goal3.points[ind2].pose.resize(6);
    goal3.points[ind2].pose[0] = -25;
    goal3.points[ind2].pose[1] = -38;
    goal3.points[ind2].pose[2] = 18;
    goal3.points[ind2].pose[3] = -0.6;
    goal3.points[ind2].pose[4] = 99;
    goal3.points[ind2].pose[5] = 0.1028;

    //type
    goal3.points[ind2].movement_type = 1;

    //frame identifier
    goal3.points[ind2].frame_id = "tool[0]";

    // Velocity
    goal3.points[ind2].linVel = 0.66;

    // Acc
    goal3.points[ind2].linAcc = 0.11;


    // ********************************//
    // sending of goal to actionserver //
    // ********************************//

    // send the goal to the action server
    ac3.sendGoal(goal3);
    ROS_INFO("The RobotMovementGoal was sent");

    // ********************************//
    // waiting for reply by act-server //
    // ********************************//

    // wait for the action to return
    ROS_INFO("Waiting for action to return");
    bool finished_before_timeout2 = ac3.waitForResult(ros::Duration(60.0));

    // handling possible timeout of actionserver reply
    if (finished_before_timeout2)
    {
        actionlib::SimpleClientGoalState state = ac3.getState();
        ROS_WARN("Action finished: %s",state.toString().c_str());
    }
    else
    {
        ROS_WARN("Action did not finish before the time out.");
    }

#endif
    //exit
    return 0;
}



