
#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

using namespace std;

int main(int argc, char**argv) {
  ros::init(argc, argv, "multi_robot_navigation");
  ros::NodeHandle n;
  
  ros::NodeHandle privateNode("~");
  string location1;
  string location2;
  privateNode.param<string>("location1",location1,"l3_414b");
  privateNode.param<string>("location2",location2,"l3_516");
  
  ROS_INFO_STREAM("marvin going to " << location1);
  ROS_INFO_STREAM("roberto going to " << location2);
  
  Client marvin_client("marvin/action_executor/execute_plan", true);
  marvin_client.waitForServer();
  
  bwi_kr_execution::ExecutePlanGoal goal1;
  
  bwi_kr_execution::AspRule rule1;
  bwi_kr_execution::AspFluent fluent1;
  fluent1.name = "not at";
  
  fluent1.variables.push_back(location1);
 
  rule1.body.push_back(fluent1);
  goal1.aspGoal.push_back(rule1);
  
  ROS_INFO("sending marvin goal");
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
  roberto_client.sendGoal(goal2);
  
  ros::Rate wait_rate(10);
  while(ros::ok() && (! (marvin_client.getState().isDone() && roberto_client.getState().isDone())))
    wait_rate.sleep();
  
  if (!marvin_client.getState().isDone()) {
    ROS_INFO("marvin: Canceling goal");
    marvin_client.cancelGoal();
    //and wait for canceling confirmation
    for(int i = 0; !marvin_client.getState().isDone() && i<50; ++i)
      wait_rate.sleep();
  }
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

  if (!roberto_client.getState().isDone()) {
    ROS_INFO("roberto: Canceling goal");
    roberto_client.cancelGoal();
    //and wait for canceling confirmation
    for(int i = 0; !roberto_client.getState().isDone() && i<50; ++i)
      wait_rate.sleep();
  }
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
  return 0;
}
