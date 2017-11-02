#include "DeliverMessage.h"

#include "ActionFactory.h"

#include "CallGUI.h"

#include "bwi_kr_execution/AspFluent.h"

#include <bwi_kr_execution/UpdateFluents.h>
#include <bwi_kr_execution/CurrentStateQuery.h>

#include <ros/ros.h>
#include <sound_play/sound_play.h>

#include <string>
#include <iostream>

using namespace std;

namespace bwi_krexec {

DeliverMessage::DeliverMessage() : 
            person(),
            message_id(),
            done(false){
            }

ros::Publisher DeliverMessage::message_pub;
bool DeliverMessage::pub_set(false);

struct IsFluentAt {
  
  bool operator()(const bwi_kr_execution::AspFluent& fluent) {
    return fluent.name == "at";
  }
  
};

void DeliverMessage::run() {

  ros::NodeHandle n;
  if (!pub_set) { 
    message_pub = n.advertise<sound_play::SoundRequest>("robotsound", 1000);
    pub_set = true;
  }

  ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ( "update_fluents" );
  krClient.waitForExistence();

//  if (message_pub.getNumSubscribers() == 0) return; //if the subscriber is not connected, sleep

  //speak
  sound_play::SoundRequest sound_req;
  sound_req.sound = sound_play::SoundRequest::SAY;
  sound_req.command = sound_play::SoundRequest::PLAY_ONCE;
  std::stringstream ss;
  
  ss << "I have a message for you. Do you want to read it now?\n";
  sound_req.arg = ss.str();

  message_pub.publish(sound_req);

  vector<string> options;
  options.push_back("Yes");
  options.push_back("No");

  CallGUI ask("ask", CallGUI::CHOICE_QUESTION, ss.str(), 20.0, options);
  ask.run();

  bwi_kr_execution::UpdateFluents uf;

  bwi_kr_execution::AspFluent delivered;
  delivered.name = ask.getResponseIndex() == 0 ? "messagedelivered" : "-messagedelivered";
  delivered.variables.push_back(person);
  delivered.variables.push_back(message_id);
  uf.request.fluents.push_back(delivered);

  if (ask.getResponseIndex() == 0) {
    string s = "Hi\n";
    CallGUI message("message", CallGUI::DISPLAY, s, 5.0);
    message.run();
    ros::Duration(5.0).sleep();
    bwi_kr_execution::AspFluent not_message;
    not_message.name = "-message";
    not_message.variables.push_back(person);
    not_message.variables.push_back(message_id);
    uf.request.fluents.push_back(not_message);
  }
  else if (ask.getResponseIndex() == 1) {
    string s = "OK. You can ask me again later.\n";
    CallGUI message("message", CallGUI::DISPLAY, s, 5.0);
    message.run();
  }
  else {
    ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::CurrentStateQuery> ("current_state_query");
    krClient.waitForExistence();
    bwi_kr_execution::CurrentStateQuery csq;
    krClient.call(csq);
    vector<bwi_kr_execution::AspFluent>::const_iterator atIt = 
                      find_if(csq.response.answer.fluents.begin(), csq.response.answer.fluents.end(), IsFluentAt());
                      
    if(atIt == csq.response.answer.fluents.end()) {
      ROS_ERROR("DeliverMessage: fluent \"at\" missing ");
    }
    else {
      bwi_kr_execution::AspFluent not_inroom;
      not_inroom.name = "-inroom";
      not_inroom.variables.push_back(person);
      not_inroom.variables.push_back(atIt->variables[0]);
      uf.request.fluents.push_back(not_inroom);
    }
  }

  krClient.call(uf);

  CallGUI clear("clear", CallGUI::DISPLAY,  "");
  clear.run();

  done = true;
}

actasp::Action* DeliverMessage::cloneAndInit(const actasp::AspFluent& fluent) const {
  DeliverMessage *newAction = new DeliverMessage();
  newAction->person = fluent.getParameters().at(0);
  newAction->message_id = fluent.getParameters().at(1);

  return newAction;
}

std::vector<std::string> DeliverMessage::getParameters() const {
  vector<string> param;
  param.push_back(person);
  param.push_back(message_id);
  return param;
}


ActionFactory DeliverMessageFactory(new DeliverMessage());
  
}