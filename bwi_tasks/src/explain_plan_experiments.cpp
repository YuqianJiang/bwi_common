#include <bwi_msgs/QuestionDialog.h>
#include <bwi_kr_execution/ExecutePlanAction.h>

#include <actionlib/client/simple_action_client.h>
#include <bwi_kr_execution/UpdateFluents.h>
#include <bwi_kr_execution/GetHriMessage.h>
#include <bwi_kr_execution/ComputeAndExplainPlan.h>
#include <ros/ros.h>
#include <string>

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

using namespace std;

ros::ServiceClient guiClient;
ros::ServiceClient updateClient;
ros::ServiceClient explainClient;
Client* client;

string plan_explanation;

class MessageServer {
public:

  MessageServer() : messages(), id_counter(0) {}

  static const bwi_kr_execution::HriMessage emptyMessage;

  string addMessage(string content, string from, string to) {
    bwi_kr_execution::HriMessage message;
    stringstream ss;
    ss << "m" << id_counter++;

    message.id = ss.str();
    message.content = content;
    message.from = from;
    message.to = to;
    messages.insert(make_pair<string,bwi_kr_execution::HriMessage> (message.id, message));
    return message.id;
  }

  void addMessage(bwi_kr_execution::HriMessage message) {
    messages.insert(make_pair<string,bwi_kr_execution::HriMessage> (message.id, message));
  }

  bool lookUpMessage(bwi_kr_execution::GetHriMessage::Request  &req,
                     bwi_kr_execution::GetHriMessage::Response &res) {
    if (messages.find(req.message_id) != messages.end()) {
      res.message = messages.find(req.message_id)->second;
    }
    else {
      res.message = MessageServer::emptyMessage;
    }

    return true;
  }

private:
  map<string, bwi_kr_execution::HriMessage> messages;
  int id_counter;

};

const bwi_kr_execution::HriMessage MessageServer::emptyMessage;
MessageServer messageServer;

string askTextQuestion(string message) {
  bwi_msgs::QuestionDialog question;
  question.request.type = question.request.TEXT_QUESTION;
  question.request.message = message;
  question.request.timeout = 0.0;

  if (guiClient.call(question)) {
    return question.response.text;
  }
  else {
    ROS_ERROR("Failed to call service /question_dialog");
    ros::shutdown();
  }

}

bool askYesNoQuestion(string message) {
  bwi_msgs::QuestionDialog question;
  question.request.type = question.request.CHOICE_QUESTION;
  question.request.message = message;
  question.request.options.push_back("Yes");
  question.request.options.push_back("No");
  question.request.timeout = 0.0;

  if (guiClient.call(question)) {
    if (question.response.index == 0) {
      return true;
    }
    else {
      return false;
    }
  }
  else {
    ROS_ERROR("Failed to call service /question_dialog");
    ros::shutdown();
  }
}

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

void planExplanationCb(const bwi_kr_execution::ExecutePlanFeedbackConstPtr& feedback) {
  //ROS_INFO_STREAM(feedback->plan_explanation);
  plan_explanation = feedback->plan_explanation;
  //displayMessage(feedback->plan_explanation, 0.0, true);
  //ros::Duration(5.0).sleep();
}

bool updateLookingFor(string target) {
  transform(target.begin(), target.end(), target.begin(), ::tolower);
  bwi_kr_execution::UpdateFluents uf;

  bwi_kr_execution::AspFluent lookingfor;
  lookingfor.name = "lookingfor";
  lookingfor.variables.push_back(target);
  uf.request.fluents.push_back(lookingfor);

  updateClient.call(uf);

  return uf.response.consistent;
}

bool updateLocations(string target, string locations) {
  transform(target.begin(), target.end(), target.begin(), ::tolower);
  bwi_kr_execution::UpdateFluents uf;

  size_t start = locations.find("l");
  size_t end = locations.find(";");
  if (start == string::npos) return false;

  while (end != string::npos) {
    bwi_kr_execution::AspFluent location;
    location.name = "possiblelocation";
    location.variables.push_back(target);
    location.variables.push_back(locations.substr(start, end-start));
    //ROS_INFO_STREAM(locations.substr(start, end-start));
    uf.request.fluents.push_back(location);

    start = locations.find("l", end);
    end = locations.find(";", start);
  }

  if (start != string::npos) {
    bwi_kr_execution::AspFluent location;
    location.name = "possiblelocation";
    location.variables.push_back(target);
    location.variables.push_back(locations.substr(start));
    //ROS_INFO_STREAM(locations.substr(start));
    uf.request.fluents.push_back(location);
  }

  updateClient.call(uf);

  return uf.response.consistent;
}

void explainDeliveryPlan(string target, string id) {

  //construct ASP planning goal
  bwi_kr_execution::ComputeAndExplainPlan goal;

  //find the person or conclude that the person cannot be found at the moment
  bwi_kr_execution::AspRule delivered_rule;

  bwi_kr_execution::AspFluent delivered;
  delivered.name = "not messagedelivered";
  delivered.variables.push_back(target);
  delivered.variables.push_back(id);
  delivered_rule.body.push_back(delivered);

  bwi_kr_execution::AspFluent not_found;
  not_found.name = "not -found";
  not_found.variables.push_back(target);
  delivered_rule.body.push_back(not_found);

  goal.request.goal.push_back(delivered_rule);

  //ROS_INFO("explaining delivery plan");

  if (explainClient.call(goal)) {
    //ROS_INFO_STREAM(goal.response.plan_explanation);
    displayMessage("I will " + goal.response.plan_explanation, 0.0, true);
  }
  else {
    ROS_ERROR("Failed to call service /compute_and_explain_plan");
    ros::shutdown();
  }

}

void sendCheckpointGoal(string location) {
  bwi_kr_execution::ExecutePlanGoal goal;
  
  bwi_kr_execution::AspRule rule;
  bwi_kr_execution::AspFluent fluent;
  fluent.name = "not beside";
  
  fluent.variables.push_back(location);
 
  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);
  
  //ROS_INFO("sending navigation goal");
  client->sendGoalAndWait(goal);
}

void sendFacingGoal(string door) {
  bwi_kr_execution::ExecutePlanGoal goal;
  
  bwi_kr_execution::AspRule rule;
  bwi_kr_execution::AspFluent fluent;
  fluent.name = "not facing";
  
  fluent.variables.push_back(door);
 
  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);
  
  //ROS_INFO("sending navigation goal");
  client->sendGoalAndWait(goal);
}

void sendAtGoal(string room) {
  bwi_kr_execution::ExecutePlanGoal goal;
  
  bwi_kr_execution::AspRule rule;
  bwi_kr_execution::AspFluent fluent;
  fluent.name = "not at";
  
  fluent.variables.push_back(room);
 
  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);
  
  //ROS_INFO("sending navigation goal");
  client->sendGoalAndWait(goal);
}

void sendDeliveryGoal(string target, string id) {
  //construct ASP planning goal
  bwi_kr_execution::ExecutePlanGoal goal;

  //find the person or conclude that the person cannot be found at the moment
  bwi_kr_execution::AspRule delivered_rule;

  bwi_kr_execution::AspFluent delivered;
  delivered.name = "not messagedelivered";
  delivered.variables.push_back(target);
  delivered.variables.push_back(id);
  delivered_rule.body.push_back(delivered);

  //bwi_kr_execution::AspFluent not_found;
  //not_found.name = "not -found";
  //not_found.variables.push_back(target);
  //delivered_rule.body.push_back(not_found);

  goal.aspGoal.push_back(delivered_rule);

  //ROS_INFO("sending goal");
  client->sendGoal(goal, Client::SimpleDoneCallback(), Client::SimpleActiveCallback(), &planExplanationCb);
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "hri_tasks_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("look_up_message", &MessageServer::lookUpMessage, &messageServer);

  client = new Client("action_executor/execute_plan", true);
  client->waitForServer();

  guiClient = n.serviceClient<bwi_msgs::QuestionDialog>("/question_dialog");
  guiClient.waitForExistence();
  updateClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ("update_fluents");
  updateClient.waitForExistence();
  explainClient = n.serviceClient<bwi_kr_execution::ComputeAndExplainPlan> ("compute_and_explain_plan");
  explainClient.waitForExistence();

  //go to checkpoint 1
  displayMessage("Follow me!", 0.0, false);
  sendCheckpointGoal("checkpoint_1");

  string content = "Hello!";
  string requester = "X";
  string target = "Y";
  string possiblelocation = "l3_414b";
  updateLocations(target, possiblelocation);
  updateLookingFor(target);
  string id = messageServer.addMessage(content, requester, target);

  //update message in domain knowledge
  transform(target.begin(), target.end(), target.begin(), ::tolower);
  bwi_kr_execution::UpdateFluents uf;
  bwi_kr_execution::AspFluent message;
  message.name = "message";
  message.variables.push_back(target);
  message.variables.push_back(id);
  uf.request.fluents.push_back(message);
  updateClient.call(uf);

  //explain
  explainDeliveryPlan(target, id);

  string goal_question = "Do you understand what my task is?";
  ROS_INFO_STREAM("1.1 understand task: " << askYesNoQuestion(goal_question));

  string guess_question = "Make a guess!";
  ROS_INFO_STREAM("1.2 guess task: " << askTextQuestion(guess_question));

  string confidence_question = "Do you believe my plan will work?";
  ROS_INFO_STREAM("1.3 can complete: " << askYesNoQuestion(confidence_question));

  //motion prediction 1
  bwi_msgs::QuestionDialog prediction_question_1;
  prediction_question_1.request.type = bwi_msgs::QuestionDialogRequest::CHOICE_QUESTION;
  string question = "Which of the following will I most likely do next? \n";
  question += "A. turn left to the instructional lab \n";
  question += "B. go straight \n";
  question += "C. turn right to take the elevator";
  prediction_question_1.request.message = question;

  prediction_question_1.request.options.push_back("A");
  prediction_question_1.request.options.push_back("B");
  prediction_question_1.request.options.push_back("C");
  prediction_question_1.request.timeout = 0.0;

  if (guiClient.call(prediction_question_1)) {
    ROS_INFO_STREAM("1.4 predict turn: " << prediction_question_1.response.index);
  }
  else {
    ROS_ERROR("Failed to call service /question_dialog");
    ros::shutdown();
  }

  //go to checkpoint 2
  displayMessage("Follow me!", 0.0, false);
  sendCheckpointGoal("checkpoint_2");
  //explainDeliveryPlan(target, id);

  //prediction of robot's request 1
  string door_question_1 = "Do I need the door on my right (the one connects to the 3.400 corridor) to be open?";
  ROS_INFO_STREAM("2. predict door: " << askYesNoQuestion(door_question_1));

  //go to checkpoint 3
  displayMessage("Follow me!", 0.0, false);
  sendCheckpointGoal("checkpoint_3");
  explainDeliveryPlan(target, id);

  ROS_INFO_STREAM("3.1 understand task: " << askYesNoQuestion(goal_question));
  ROS_INFO_STREAM("3.2 guess task: " << askTextQuestion(guess_question));
  ROS_INFO_STREAM("3.3 can complete: " << askYesNoQuestion(confidence_question));

  //prediction of motion
  bwi_msgs::QuestionDialog prediction_question_2;
  prediction_question_2.request.type = bwi_msgs::QuestionDialogRequest::CHOICE_QUESTION;
  question = "Which of the following will I most likely do next? \n";
  question += "A. turn right \n";
  question += "B. go straight \n";
  prediction_question_2.request.message = question;

  prediction_question_2.request.options.push_back("A");
  prediction_question_2.request.options.push_back("B");
  prediction_question_2.request.timeout = 0.0;

  if (guiClient.call(prediction_question_2)) {
    ROS_INFO_STREAM("3.4 predict turn: " << prediction_question_2.response.index);
  }
  else {
    ROS_ERROR("Failed to call service /question_dialog");
    ros::shutdown();
  }

  //go to checkpoint 4
  displayMessage("Follow me!", 0.0, false);
  sendCheckpointGoal("checkpoint_4");

  //prediction of robot's request 2
  string door_question_2 = "Do I need the door on my right (the one connects to robotics lab 3.414b) to be open?";
  ROS_INFO_STREAM("4.1 predict door: " << askYesNoQuestion(door_question_2));

  //turn around to the lab door
  sendFacingGoal("d3_414b2");

  //prediction of motion 3
  bwi_msgs::QuestionDialog prediction_question_3;
  prediction_question_3.request.type = bwi_msgs::QuestionDialogRequest::CHOICE_QUESTION;
  question = "Which of the following will I most likely do next? \n";
  question += "A. go through the door in front of me \n";
  question += "B. speak \n";
  prediction_question_3.request.message = question;

  prediction_question_3.request.options.push_back("A");
  prediction_question_3.request.options.push_back("B");
  prediction_question_3.request.timeout = 0.0;

  if (guiClient.call(prediction_question_3)) {
    ROS_INFO_STREAM("4.2 predict action: " << prediction_question_3.response.index);
  }
  else {
    ROS_ERROR("Failed to call service /question_dialog");
    ros::shutdown();
  }

  //go inside the lab
  displayMessage("Follow me!", 0.0, false);
  sendAtGoal("l3_414b");
  explainDeliveryPlan(target, id);

  //experimenter announces whether Y is here
  ROS_INFO_STREAM("5.1 understand task: " << askYesNoQuestion(goal_question));
  ROS_INFO_STREAM("5.2 guess task: " << askTextQuestion(guess_question));
  ROS_INFO_STREAM("5.3 can complete: " << askYesNoQuestion(confidence_question));

  sendDeliveryGoal(target, id);

  ros::Rate wait_rate(10);
  while(ros::ok() && !client->getState().isDone()) {
    ros::spinOnce();
    wait_rate.sleep();
  }
    
  if (!client->getState().isDone()) {
    //ROS_INFO("Canceling goal");
    client->cancelGoal();
    //and wait for canceling confirmation
    for(int i = 0; !client->getState().isDone() && i<50; ++i)
      wait_rate.sleep();
  }
  if (client->getState() == actionlib::SimpleClientGoalState::ABORTED) {
    displayMessage("Aborted", 5.0);
  }
  else if (client->getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
    displayMessage("Preempted", 5.0);
  }
  
  else if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    displayMessage("Succeeded!", 5.0);
  }
  else
    displayMessage("Terminated", 5.0);
    
  return 0;

}
