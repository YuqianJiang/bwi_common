#include "OpenDoor.h"
#include "CallGUI.h"
#include "LogicalNavigation.h"
#include "actasp/AspFluent.h"

#include "plan_execution/CurrentStateQuery.h"
#include "plan_execution/AspRule.h"
#include "plan_execution/AspFluent.h"
#include "SenseLocation.h"

#include <ros/ros.h>

using namespace std;
using namespace ros;

namespace bwi_krexec {

OpenDoor::OpenDoor(const int door_id, knowledge_rep::LongTermMemoryConduit &ltmc) :
            door_id(door_id), 
            door_entity(door_id, ltmc), 
            door_name(), 
            done(false), 
            failed(false), 
            requestSent(false),
            startTime(), 
            ltmc(ltmc) {}

bool OpenDoor::checkDoorOpen() {
  bwi_msgs::CheckBool open_srv;
  doorStateClient.call(open_srv);

  if (open_srv.response.value) {
    ROS_INFO_STREAM("Door " << door_name << " is open");
    door_entity.add_attribute("is_open", true);

    return true;
  }

  return false;
}
  
void OpenDoor::run() {
  NodeHandle n;

  if (!requestSent) {

    if (!door_entity.is_valid()) {
      failed = true;
      return;
    }
    auto attrs = door_entity.get_attributes("name");
    
    if (attrs.size() != 1) {
      failed = true;
      return;
    }

    door_name = attrs.at(0).get_string_value();

    CallGUI askToOpen("askToOpen", CallGUI::DISPLAY,  "Can you open door " + door_name + ", please?");
    askToOpen.run();

    requestSent = true;
    startTime = ros::Time::now();

    doorStateClient = n.serviceClient<bwi_msgs::CheckBool>("/sense_door_state");
  }

  done = checkDoorOpen();
  
  if(done) {
    CallGUI thank("thank", CallGUI::DISPLAY,  "Thanks!");
    thank.run();
  }

  if(!done && (ros::Time::now() - startTime) > ros::Duration(120.0)) {
    failed = true;
    done = true;
  }

}  
  
actasp::Action* OpenDoor::cloneAndInit(const actasp::AspFluent& fluent) const {
  return nullptr;
}

std::vector<std::string> OpenDoor::getParameters() const {
  return {to_string(door_id)};
}

}
