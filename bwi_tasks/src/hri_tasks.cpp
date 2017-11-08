#include <bwi_msgs/QuestionDialog.h>
#include <bwi_kr_execution/ExecutePlanAction.h>
#include <bwi_msgs/LogicalNavigationAction.h>

#include <actionlib/client/simple_action_client.h>
#include <bwi_msgs/LogicalNavigationAction.h>
#include <tf/transform_listener.h>
#include <bwi_kr_execution/UpdateFluents.h>
#include <bwi_kr_execution/CurrentStateQuery.h>
#include <bwi_kr_execution/GetHriMessage.h>
#include <bwi_msgs/UpdateObject.h>
#include <bwi_kr_execution/HriMessage.h>

#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <string>

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

using namespace std;

ros::ServiceClient guiClient;
ros::ServiceClient krClient;
ros::ServiceClient updateClient;
ros::ServiceClient lnClient;
bool isActive;
std::vector<string> doors;
Client* client;

struct IsFluentAt {
  bool operator()(const bwi_kr_execution::AspFluent& fluent) {
    return fluent.name == "at";
  }
};

struct IsFluentFound {
  IsFluentFound(string name) : name(name) {}
  bool operator()(const bwi_kr_execution::AspFluent& fluent) {
    return fluent.name == "found" && fluent.variables[0] == name;
  }
private:
  string name;
};


struct IsFluentBusy {
  IsFluentBusy(string name) : name(name) {}
  bool operator()(const bwi_kr_execution::AspFluent& fluent) {
    return fluent.name == "busy" && fluent.variables[0] == name;
  }
private:
  string name;
};

//class Message {
//public:
//  Message(int id, string content) : id(id), content(content) {}
//  int getId() {return id;}
//  string getContent() {return content;}
//
//private:
//  int id;
//  string content;
//
//};

class MessageServer {
public:

  MessageServer() : messages(), id_counter(0) {}

  static const bwi_kr_execution::HriMessage emptyMessage;

  string addMessage(string content, string from, string to) {
    bwi_kr_execution::HriMessage message;
    stringstream ss;
    ss << "message_" << id_counter++;

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

void goToDoor(string door) {  
  bwi_kr_execution::ExecutePlanGoal goal;

  bwi_kr_execution::AspRule rule;
  bwi_kr_execution::AspFluent fluent;
  fluent.name = "not facing";

  fluent.variables.push_back(door);

  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);

  ROS_INFO("sending goal");
  client->sendGoal(goal);
}

void goToRandomDoor() {
  ROS_INFO("going to random door");
  string door = doors[(rand() % doors.size())];
  goToDoor(door);
}

bwi_msgs::QuestionDialog askTextQuestion(string message, float timeout) {
  bwi_msgs::QuestionDialog question;
  question.request.type = question.request.TEXT_QUESTION;
  question.request.message = message;
  question.request.timeout = timeout;

  if (guiClient.call(question)) {
    if (question.response.index != bwi_msgs::QuestionDialogRequest::TEXT_RESPONSE) {
      ROS_INFO("No text response detected, going for a random walk");
      goToRandomDoor();
    }
  }
  else {
    ROS_ERROR("Failed to call service /question_dialog");
    ros::shutdown();
  }

  return question;
}

bwi_msgs::QuestionDialog askYesNoQuestion(string message, float timeout) {
  bwi_msgs::QuestionDialog question;
  question.request.type = question.request.CHOICE_QUESTION;
  question.request.message = message;
  question.request.options.push_back("Yes");
  question.request.options.push_back("No");
  question.request.timeout = timeout;

  if (guiClient.call(question)) {
    if (question.response.index < 0) {
      ROS_INFO("No text response detected, going for a random walk");
      goToRandomDoor();
    }
  }
  else {
    ROS_ERROR("Failed to call service /question_dialog");
    ros::shutdown();
  }

  return question;
}

void displayMessage(string message, float timeout) {
  bwi_msgs::QuestionDialog question;
  question.request.type = question.request.DISPLAY;
  question.request.message = message;
  question.request.timeout = timeout;

  if (!guiClient.call(question)) {
    ROS_ERROR("Failed to call service /question_dialog");
    ros::shutdown();
  }

}

bool checkFound(string person) {
  transform(person.begin(), person.end(), person.begin(), ::tolower);
  bwi_kr_execution::CurrentStateQuery csq;
  krClient.call(csq);

  vector<bwi_kr_execution::AspFluent>::const_iterator foundIt = 
                    find_if(csq.response.answer.fluents.begin(), 
                            csq.response.answer.fluents.end(), 
                            IsFluentFound(person));
                    
  if (foundIt == csq.response.answer.fluents.end()) {
    return false;
  }
  return true;
}

bool checkBusy(string person) {
  transform(person.begin(), person.end(), person.begin(), ::tolower);
  bwi_kr_execution::CurrentStateQuery csq;
  krClient.call(csq);

  vector<bwi_kr_execution::AspFluent>::const_iterator busyIt = 
                    find_if(csq.response.answer.fluents.begin(), 
                            csq.response.answer.fluents.end(), 
                            IsFluentBusy(person));
                    
  if (busyIt == csq.response.answer.fluents.end()) {
    return false;
  }
  return true;
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
    ROS_INFO_STREAM(locations.substr(start, end-start));
    uf.request.fluents.push_back(location);

    start = locations.find("l", end);
    end = locations.find(";", start);
  }

  if (start != string::npos) {
    bwi_kr_execution::AspFluent location;
    location.name = "possiblelocation";
    location.variables.push_back(target);
    location.variables.push_back(locations.substr(start));
    ROS_INFO_STREAM(locations.substr(start));
    uf.request.fluents.push_back(location);
  }

  updateClient.call(uf);

  return uf.response.consistent;
}

bool validateTarget(string target) {
  if (checkFound(target)) {      
    if (checkBusy(target)) {
      string question = target + " is busy last time I checked. \n";
      question += "Do you want me to check again?";

      bwi_msgs::QuestionDialog checkAgain = askYesNoQuestion(question, 10.0);
      if (checkAgain.response.index != 0) {
        return false;
      }
    }
    return true;
  }
  else {
    string question = "I do not know where " + target + " is. \n";
    question += "Do you want me to look for " + target + " ?";

    bwi_msgs::QuestionDialog lookFor = askYesNoQuestion(question, 10.0);

    if (lookFor.response.index == 0) {
      ROS_INFO_STREAM("I will look for " + target);

      if (updateLookingFor(target)) {
        return true;
      }
      else {
        // target is not modeled in the domain. ask for user help
        string question = "Can you suggest possible location(s) for " + target + "? \n";
        bwi_msgs::QuestionDialog canSuggest = askYesNoQuestion(question, 10.0);

        while (canSuggest.response.index == 0) {
          question = "syntax: l[floor]_[room number];\n";
          question += "e.g. l3_414a; l3_414b";

          bwi_msgs::QuestionDialog locations = askTextQuestion(question, 20.0);
          if (updateLocations(target, locations.response.text)) {
            return true;
          }

          question = "I do not recognize the location(s). Do you want to suggest other location(s) for " + target + "? \n";
          canSuggest = askYesNoQuestion(question, 10.0);
        }
        
        
      }
    }
    else if (lookFor.response.index == 1) {
      displayMessage("Okay. Have a nice day!", 10.0);
      return false;
    }
  }
}

bool updateRequesterInfo(string requester, string message_id) {
  transform(requester.begin(), requester.end(), requester.begin(), ::tolower);

  //get current pose as approach point for the requester
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try{
    listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1.0));
    while (! listener.canTransform("/map", "/base_link", ros::Time(0))) {
      ros::Duration(0.1).sleep();
    }
    listener.lookupTransform("/map", "/base_link",  
                             ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::shutdown();
  }
  geometry_msgs::Pose msg;
  tf::poseTFToMsg(transform, msg);

  //add object to map
  bwi_msgs::UpdateObject uo;
  uo.request.object_name = requester+"_marker";
  uo.request.pose = msg;
  lnClient.call(uo);

  //get current logical location
  bwi_kr_execution::CurrentStateQuery csq;
  krClient.call(csq);
  vector<bwi_kr_execution::AspFluent>::const_iterator atIt = 
                    find_if(csq.response.answer.fluents.begin(), csq.response.answer.fluents.end(), IsFluentAt());
                    
  if (atIt == csq.response.answer.fluents.end()) {
    ROS_ERROR("look_for_person_node: fluent \"at\" missing ");
    ros::shutdown();
  }
  string location = atIt->variables[0];

  bwi_kr_execution::UpdateFluents uf;

  bwi_kr_execution::AspFluent inroom;
  inroom.name = "inroom";
  inroom.variables.push_back(requester);
  inroom.variables.push_back(location);
  uf.request.fluents.push_back(inroom);

  bwi_kr_execution::AspFluent locationmarker;
  locationmarker.name = "locationmarker";
  locationmarker.variables.push_back(requester);
  locationmarker.variables.push_back(requester+"_marker");
  uf.request.fluents.push_back(locationmarker);

  updateClient.call(uf);

  return uf.response.consistent;
}

bwi_msgs::QuestionDialog getInitialPage() {

  bwi_msgs::QuestionDialog initial;
  initial.request.type = initial.request.CHOICE_QUESTION;
  initial.request.message = "Press a button to get started!";

  initial.request.options.push_back("visit a door");
  initial.request.options.push_back("ask a question");
  initial.request.options.push_back("deliver a message");

  initial.request.timeout = 30.0;

  return initial;

}

void sendDeliveryGoal(string target, string id) {
  transform(target.begin(), target.end(), target.begin(), ::tolower);
  bwi_kr_execution::UpdateFluents uf;

  bwi_kr_execution::AspFluent message;
  message.name = "message";
  message.variables.push_back(target);
  message.variables.push_back(id);
  uf.request.fluents.push_back(message);

  updateClient.call(uf);

  //construct ASP planning goal
  bwi_kr_execution::ExecutePlanGoal goal;

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

  goal.aspGoal.push_back(delivered_rule);

  ROS_INFO("sending goal");
  client->sendGoal(goal);
}


bool deliverMessageHandler() {
  string question = "Who are you sending a message to?";
  bwi_msgs::QuestionDialog getTarget = askTextQuestion(question, 20.0);

  if (getTarget.response.index == bwi_msgs::QuestionDialogRequest::TEXT_RESPONSE) {
    string target = getTarget.response.text;
    if (validateTarget(target)) {
      ROS_INFO("Target validated");

      question = "OK. What is your message?";
      bwi_msgs::QuestionDialog getMessage = askTextQuestion(question, 20.0);

      string question = "OK. What is your name?";
      bwi_msgs::QuestionDialog getUserName = askTextQuestion(question, 20.0);
      string requester = getUserName.response.text;

      string id = messageServer.addMessage(getMessage.response.text, requester, target);

      updateRequesterInfo(requester, id);
      sendDeliveryGoal(target, id);

      /*if (!checkFound(target)) {
        displayMessage("OK. I will look for " + target + " in ___ and deliver your message", 5.0);
      }
      else {
        displayMessage("OK. I will go to ___ and deliver your message", 5.0);
      }*/
      

    }
  }

}

int main(int argc, char**argv) {
  ros::init(argc, argv, "hri_tasks_node");
  ros::NodeHandle n;

  doors.push_back("d3_414b1");
  doors.push_back("d3_414b2");
  doors.push_back("d3_414a1");
  doors.push_back("d3_414a2");

  ros::ServiceServer service = n.advertiseService("look_up_message", &MessageServer::lookUpMessage, &messageServer);

  client = new Client("action_executor/execute_plan", true);
  client->waitForServer();

  actionlib::SimpleActionClient<bwi_msgs::LogicalNavigationAction> navClient("execute_logical_goal", true);;
  navClient.waitForServer();

  guiClient = n.serviceClient<bwi_msgs::QuestionDialog>("/question_dialog");
  guiClient.waitForExistence();
  krClient = n.serviceClient<bwi_kr_execution::CurrentStateQuery> ("current_state_query");
  krClient.waitForExistence();
  updateClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ("update_fluents");
  updateClient.waitForExistence();
  lnClient = n.serviceClient<bwi_msgs::UpdateObject> ("update_object");
  lnClient.waitForExistence();

  ros::Duration rate(0.5);

  while (ros::ok()) {

    if (isActive && client->getState().isDone()) {
      isActive = false;
    }

    bwi_msgs::QuestionDialog initial = getInitialPage();

    if (!isActive) {

      if (guiClient.call(initial)) {

        if (initial.response.index >= 0) {
          client->cancelAllGoals();
          navClient.cancelAllGoals();
          isActive = true;

          if (initial.response.index == 0) {
            goToRandomDoor();
          }
          else if (initial.response.index == 1){
          }
          else if (initial.response.index == 2) {
            deliverMessageHandler();
          }
        }
        else {
          if (client->getState().isDone()) {
            ROS_INFO("No response detected, going for a random walk");
            isActive = false;
            goToRandomDoor();
          }
        }

      }
      else {
        ROS_ERROR("Failed to call service /question_dialog");
        ros::shutdown();
      }

    }

    if (client->getState() == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_INFO("Aborted");
    } else if (client->getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
      ROS_INFO("Preempted");
    }
    else if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Succeeded!");
    }

    rate.sleep();
    ros::spinOnce();

  }

}