
#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>

#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

using namespace std;

void reportMarvin(Client& marvin_client) {
  if (marvin_client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
    ROS_INFO("marvin: Aborted");
  }
  else if (marvin_client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
    ROS_INFO("marvin: Preempted");
  }
  
  else if (marvin_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("marvin: Succeeded!");
  }
  else
     ROS_INFO("marvin: Terminated");
}

void reportRoberto(Client& roberto_client) {
  if (roberto_client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
    ROS_INFO("roberto: Aborted");
  }
  else if (roberto_client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
    ROS_INFO("roberto: Preempted");
  }
  
  else if (roberto_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("roberto: Succeeded!");
  }
  else
     ROS_INFO("roberto: Terminated");
}

void reportBender(Client& bender_client) {
  if (bender_client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
    ROS_INFO("bender: Aborted");
  }
  else if (bender_client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
    ROS_INFO("bender: Preempted");
  }
  
  else if (bender_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("bender: Succeeded!");
  }
  else
     ROS_INFO("bender: Terminated");
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "multi_robot_navigation");
  ros::NodeHandle n;
  
  ros::NodeHandle privateNode("~");
  string location1;
  string location2;
  string location3;
  privateNode.param<string>("location1",location1,"l3_414b");
  privateNode.param<string>("location2",location2,"l3_516");
  privateNode.param<string>("location3",location3,"l3_418");
  
  ROS_INFO_STREAM("marvin going to " << location1);
  ROS_INFO_STREAM("roberto going to " << location2);
  ROS_INFO_STREAM("bender going to" << location3);
  
  Client marvin_client("marvin/action_executor/execute_plan", true);
  marvin_client.waitForServer();
  
  bwi_kr_execution::ExecutePlanGoal goal1;
  
  bwi_kr_execution::AspRule rule1;
  bwi_kr_execution::AspFluent fluent1;
  fluent1.name = "not at";
  
  fluent1.variables.push_back(location1);
 
  rule1.body.push_back(fluent1);
  goal1.aspGoal.push_back(rule1);
  
  ros::Duration(0.5).sleep();
  ROS_INFO("sending marvin goal");
  ros::Time marvin_starting_time = ros::Time::now();
  marvin_client.sendGoal(goal1);

  Client roberto_client("roberto/action_executor/execute_plan", true);
  roberto_client.waitForServer();

  bwi_kr_execution::ExecutePlanGoal goal2;
  
  bwi_kr_execution::AspRule rule2;
  bwi_kr_execution::AspFluent fluent2;
  fluent2.name = "not at";
  
  fluent2.variables.push_back(location2);
 
  rule2.body.push_back(fluent2);
  goal2.aspGoal.push_back(rule2);

  ROS_INFO("sending roberto goal");
  ros::Time roberto_starting_time = ros::Time::now();
  roberto_client.sendGoal(goal2);

  Client bender_client("bender/action_executor/execute_plan", true);
  bender_client.waitForServer();

  bwi_kr_execution::ExecutePlanGoal goal3;
  
  bwi_kr_execution::AspRule rule3;
  bwi_kr_execution::AspFluent fluent3;
  fluent3.name = "not at";
  
  fluent3.variables.push_back(location3);
 
  rule3.body.push_back(fluent3);
  goal3.aspGoal.push_back(rule3);

  ROS_INFO("sending bender goal");
  ros::Time bender_starting_time = ros::Time::now();
  bender_client.sendGoal(goal3);
  
  ros::Rate wait_rate(10);
  bool marvin_done = false;
  bool roberto_done = false;
  bool bender_done = false;

  while(ros::ok() && (! (marvin_client.getState().isDone() && roberto_client.getState().isDone() && bender_client.getState().isDone()))) {
    wait_rate.sleep();
    if ((!marvin_done) && marvin_client.getState().isDone()) {
      reportMarvin(marvin_client);
      ROS_INFO_STREAM("marvin: total execution time " << (ros::Time::now()-marvin_starting_time).toSec());
      marvin_done = true;
    }
    if ((!roberto_done) && roberto_client.getState().isDone()) {
      reportRoberto(roberto_client);
      ROS_INFO_STREAM("roberto: total execution time " << (ros::Time::now()-roberto_starting_time).toSec());
      roberto_done = true;
    }
    if ((!bender_done) && bender_client.getState().isDone()) {
      reportBender(bender_client);
      ROS_INFO_STREAM("bender: total execution time " << (ros::Time::now()-bender_starting_time).toSec());
      bender_done = true;
    }
  }
  
  if (!marvin_client.getState().isDone()) {
    ROS_INFO("marvin: Canceling goal");
    marvin_client.cancelGoal();
    //and wait for canceling confirmation
    for(int i = 0; !marvin_client.getState().isDone() && i<50; ++i)
      wait_rate.sleep();
  }
  if (!marvin_done) {
    reportMarvin(marvin_client);
  }

  if (!roberto_client.getState().isDone()) {
    ROS_INFO("roberto: Canceling goal");
    roberto_client.cancelGoal();
    //and wait for canceling confirmation
    for(int i = 0; !roberto_client.getState().isDone() && i<50; ++i)
      wait_rate.sleep();
  }
  if (!roberto_done) {
    reportRoberto(roberto_client);
  }

  if (!bender_client.getState().isDone()) {
    ROS_INFO("bender: Canceling goal");
    bender_client.cancelGoal();
    //and wait for canceling confirmation
    for(int i = 0; !bender_client.getState().isDone() && i<50; ++i)
      wait_rate.sleep();
  }
  if (!bender_done) {
    reportRoberto(bender_client);
  }

  ros::ServiceClient set_model_client = n.serviceClient<gazebo_msgs::SetModelState>("gazebo/set_model_state");
  set_model_client.waitForExistence();

  geometry_msgs::Pose pose_1;
  pose_1.position.x = 13.35;
  pose_1.position.y = 8.15;
  pose_1.orientation = tf::createQuaternionMsgFromYaw(-1.512040504079174);
  geometry_msgs::Pose pose_2;
  pose_2.position.x = 24.400000000000006;
  pose_2.position.y = 8.15;
  pose_2.orientation = tf::createQuaternionMsgFromYaw(-1.512040504079174);
  geometry_msgs::Pose pose_3;
  pose_3.position.x = 6.500000000000002;
  pose_3.position.y = 13.55;
  pose_3.orientation = tf::createQuaternionMsgFromYaw(-1.5490606199531038);

  gazebo_msgs::SetModelState set_srv;
  set_srv.request.model_state.model_name = "marvin";
  set_srv.request.model_state.pose = pose_1;
  set_srv.request.model_state.twist = geometry_msgs::Twist();
  set_model_client.call(set_srv);
  set_srv.request.model_state.model_name = "roberto";
  set_srv.request.model_state.pose = pose_2;
  set_model_client.call(set_srv);
  set_srv.request.model_state.model_name = "bender";
  set_srv.request.model_state.pose = pose_3;
  set_model_client.call(set_srv);
  
  return 0;
}
