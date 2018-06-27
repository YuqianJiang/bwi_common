#include <bwi_msgs/QuestionDialog.h>
#include <plan_execution/ComputeAndExplainPlan.h>
#include <ros/ros.h>
#include <string>

using namespace std;

ros::ServiceClient explainClient;
ros::ServiceClient guiClient;

void displayMessage(string message, float timeout, bool confirm = false) {
  bwi_msgs::QuestionDialog question;

  if (!confirm) {
    question.request.type = question.request.DISPLAY;
    question.request.message = message;
    question.request.timeout = timeout;
  }
  else {
    vector<string> options;
    options.push_back("Got it");
    question.request.type = question.request.CHOICE_QUESTION;
    question.request.message = message;
    question.request.options = options;
    question.request.timeout = timeout;
  }

  if (!guiClient.call(question)) {
    ROS_ERROR("Failed to call service /question_dialog");
  }
}

void explainCookingPlan() {

  //construct ASP planning goal
  plan_execution::ComputeAndExplainPlan goal;

  plan_execution::AspRule rule;

  plan_execution::AspFluent fluent;
  fluent.name = "not true";
  rule.body.push_back(fluent);

  goal.request.goal.push_back(rule);

  if (explainClient.call(goal)) {
    ROS_INFO_STREAM(goal.response.plan_explanation);
    //displayMessage("I will " + goal.response.plan_explanation, 0.0, true);
  }
  else {
    ROS_ERROR("Failed to call service /compute_and_explain_plan");
    ros::shutdown();
  }

}

int main(int argc, char**argv) {
  ros::init(argc, argv, "food_assembly_experiment_node");
  ros::NodeHandle n;

  //guiClient = n.serviceClient<bwi_msgs::QuestionDialog>("/question_dialog");
  //guiClient.waitForExistence();
  explainClient = n.serviceClient<plan_execution::ComputeAndExplainPlan> ("compute_and_explain_plan");
  explainClient.waitForExistence();

  explainCookingPlan();

  return 0;
}