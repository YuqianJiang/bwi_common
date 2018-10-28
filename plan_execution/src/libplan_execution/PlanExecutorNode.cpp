#include "plan_execution/msgs_utils.h"

#include <actasp/action_utils.h>
#include <actasp/PlanExecutor.h>

#include "plan_execution/ExecutePlanAction.h"

#include <actionlib/server/simple_action_server.h>

#include <plan_execution/PlanExecutorNode.h>

#include <iostream>

using namespace std;
using namespace actasp;

namespace plan_exec {

const int MAX_N = 30;
const int PLANNER_TIMEOUT = 10; //seconds


PlanExecutorNode::PlanExecutorNode(actasp::PlanExecutor* executor) :
    executor(std::unique_ptr<actasp::PlanExecutor>(executor)),
    server({"~"}, "execute_plan", boost::bind(&PlanExecutorNode::executePlan, this, _1), false) {

  ros_observer = std::unique_ptr<RosActionServerInterfaceObserver>(new RosActionServerInterfaceObserver(server));

  server.start();

}

PlanExecutorNode::~PlanExecutorNode() = default;

void PlanExecutorNode::setRosObserver(RosActionServerInterfaceObserver* ros_observer) {
  this->ros_observer = std::unique_ptr<RosActionServerInterfaceObserver>(ros_observer);
}

void PlanExecutorNode::executePlan(const plan_execution::ExecutePlanGoalConstPtr &plan) {
  plan_execution::ExecutePlanResult result;
  vector<AspRule> goalRules;


  transform(plan->aspGoal.begin(), plan->aspGoal.end(), back_inserter(goalRules), TranslateRule());

  try {
    executor->setGoal(goalRules);
  } catch (std::logic_error &e) {
    server.setAborted(ros_observer->result);
    return;
  }

  ros::Rate loop(10);

  while (!executor->goalReached() && !executor->failed() && ros::ok() && server.isActive()) {
    if (!server.isPreemptRequested()) {
      executor->executeActionStep();
    } else {

      server.setPreempted(ros_observer->result);

      if (executor->goalReached())
        ROS_INFO("Preempted, but execution succeeded");
      else
        ROS_INFO("Preempted, execution aborted");

      if (server.isNewGoalAvailable()) {
        goalRules.clear();
        const plan_execution::ExecutePlanGoalConstPtr &newGoal = server.acceptNewGoal();
        transform(newGoal->aspGoal.begin(), newGoal->aspGoal.end(), back_inserter(goalRules), TranslateRule());

        try {
          executor->setGoal(goalRules);
        } catch (std::logic_error &e) {
          server.setAborted(ros_observer->result);
          return;
        }
      }
    }
    ros::spinOnce();
    loop.sleep();
  }


  if (executor->goalReached()) {
    ROS_INFO("Execution succeeded");
    if (server.isActive())
      server.setSucceeded(ros_observer->result);
  } else {
    ROS_INFO("Execution failed");
    if (server.isActive()) {
      server.setAborted(ros_observer->result);
    }
  }

}

}
