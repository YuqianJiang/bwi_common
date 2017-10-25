#include "Knock.h"

#include "ActionFactory.h"

#include "CallGUI.h"

#include "bwi_kr_execution/AspFluent.h"

#include <bwi_kr_execution/UpdateFluents.h>

#include <ros/ros.h>
#include <sound_play/sound_play.h>

#include <string>
#include <iostream>

using namespace std;

namespace bwi_krexec {

Knock::Knock() : 
            door(),
            done(false){
            }

ros::Publisher Knock::knock_pub;
bool Knock::pub_set(false);

void Knock::run() {

  ros::NodeHandle n;
  if (!pub_set) { 
    knock_pub = n.advertise<sound_play::SoundRequest>("robotsound", 1000);
    pub_set = true;
  }

  ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ( "update_fluents" );
  krClient.waitForExistence();

//  if (knock_pub.getNumSubscribers() == 0) return; //if the subscriber is not connected, sleep

  //speak
  sound_play::SoundRequest sound_req;
  sound_req.sound = sound_play::SoundRequest::SAY;
  sound_req.command = sound_play::SoundRequest::PLAY_ONCE;
  std::stringstream ss;
  
  ss << "Can I come in?\n";
  sound_req.arg = ss.str();

  knock_pub.publish(sound_req);

  vector<string> options;
  options.push_back("Yes");
  options.push_back("No");

  CallGUI knock("knock", CallGUI::CHOICE_QUESTION, ss.str(), 20.0, options);
  knock.run();

  bwi_kr_execution::UpdateFluents uf;
  bwi_kr_execution::AspFluent fluent;

  fluent.variables.push_back(door);

  fluent.name = knock.getResponseIndex() == 0 ? "accessgranted" : "-accessgranted";
  uf.request.fluents.push_back(fluent);
  krClient.call(uf);

  CallGUI clear("clear", CallGUI::DISPLAY,  "");
  clear.run();

  done = true;
}

actasp::Action* Knock::cloneAndInit(const actasp::AspFluent& fluent) const {
  Knock *newAction = new Knock();
  newAction->door = fluent.getParameters().at(0);
  
  return newAction;
}

std::vector<std::string> Knock::getParameters() const {
  vector<string> param;
  param.push_back(door);
  return param;
}


ActionFactory KnockFactory(new Knock());
  
}