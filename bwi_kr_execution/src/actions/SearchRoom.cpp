#include "SearchRoom.h"

#include "ActionFactory.h"

#include "CallGUI.h"

#include "bwi_kr_execution/AspFluent.h"

#include "bwi_kr_execution/CurrentStateQuery.h"
#include <bwi_kr_execution/UpdateFluents.h>

#include <ros/ros.h>
#include <sound_play/sound_play.h>

#include <string>
#include <iostream>

using namespace std;

namespace bwi_krexec {

SearchRoom::SearchRoom() : 
            person(),
            room(),
            done(false),
            failed(false),
            waiting_speach(false),
            wait_over(false){
            }

ros::Publisher SearchRoom::ask_pub;
bool SearchRoom::pub_set(false);
  
void SearchRoom::run() {

  person[0] = toupper(person[0]);

  ros::NodeHandle n;
  if (!pub_set) { 
    ask_pub = n.advertise<sound_play::SoundRequest>("robotsound", 1000);
    pub_set = true;
  }

  //current state query
  ros::ServiceClient currentClient = n.serviceClient<bwi_kr_execution::CurrentStateQuery> ( "current_state_query" );
  currentClient.waitForExistence();

  bwi_kr_execution::AspFluent atFluent;
  atFluent.name = "at";
  atFluent.timeStep = 0;
  atFluent.variables.push_back(room);
  
  bwi_kr_execution::AspRule rule;
  rule.head.push_back(atFluent);
  
  bwi_kr_execution::CurrentStateQuery csq;
  csq.request.query.push_back(rule);
  
  currentClient.call(csq);
  
  bool at = csq.response.answer.satisfied;

  if (at) {

    if (!wait_over && !waiting_speach && ask_pub.getNumSubscribers() == 0 ) {
      waiting_speach = true;
      starting_wating = ros::Time::now();
      return;
    }
    else if (!wait_over&& waiting_speach && 
      (ask_pub.getNumSubscribers() != 0 || (ros::Time::now() - starting_wating).toSec() > 1.)) {
      waiting_speach = false;
      wait_over = true;
    }


    //speak
    sound_play::SoundRequest sound_req;
    sound_req.sound = sound_play::SoundRequest::SAY;
    sound_req.command = sound_play::SoundRequest::PLAY_ONCE;
    std::stringstream ss;
    ss << "Is " << person << " in the room?";
    sound_req.arg = ss.str();
    
    ask_pub.publish(sound_req);
  }

  vector<string> options;
  options.push_back("yes");
  options.push_back("no");

  CallGUI searchRoom("searchRoom", CallGUI::CHOICE_QUESTION,  "Is " + person + " in the room " + room + "?", 60.0, options);
  searchRoom.run();

  int response = searchRoom.getResponseIndex();

  if (response >= 0) {
    sound_play::SoundRequest sound_req;
    sound_req.sound = sound_play::SoundRequest::SAY;
    sound_req.command = sound_play::SoundRequest::PLAY_ONCE;
    if (response > 0) {  
      sound_req.arg = "Thank you !";
      CallGUI thank("thank", CallGUI::DISPLAY,  "Thank you!");
      thank.run();
    }
    else {
      std::stringstream ss;
      ss << "Hi " << person << "!";
      sound_req.arg = ss.str();
      CallGUI hi("hi", CallGUI::DISPLAY,  ss.str());
      hi.run();
    }
    ask_pub.publish(sound_req);
  }else
    failed = true;

  ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ( "update_fluents" );
  krClient.waitForExistence();

  bwi_kr_execution::UpdateFluents uf;
  bwi_kr_execution::AspFluent fluent;
  fluent.timeStep = 0;
  person[0] = tolower(person[0]);
  fluent.variables.push_back(person);
  fluent.variables.push_back(room);

  fluent.name = ((response == 0) ? "inroom" : "-inroom");

  uf.request.fluents.push_back(fluent);
  krClient.call(uf);

  done = true;

}  
  
actasp::Action* SearchRoom::cloneAndInit(const actasp::AspFluent& fluent) const {
  SearchRoom *newAction = new SearchRoom();
  newAction->person = fluent.getParameters().at(0);
  newAction->room = fluent.getParameters().at(1);
  
  return newAction;
}

std::vector<std::string> SearchRoom::getParameters() const {
  vector<string> param;
  param.push_back(person);
  param.push_back(room);
  return param;
}


ActionFactory SearchRoomFactory(new SearchRoom());
  
}
