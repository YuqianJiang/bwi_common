#include "WaitForOpen.h"

#include "bwi_kr_execution/CurrentStateQuery.h"
#include "bwi_msgs/DoorHandlerInterface.h"

#include "ActionFactory.h"
#include "LogicalNavigation.h"

#include "actasp/AspFluent.h"

#include <ros/ros.h>

using namespace std;
using namespace ros;

namespace bwi_krexec {


WaitForOpen::WaitForOpen() : door(), done(false) {}

void WaitForOpen::run() {
  NodeHandle n;

  vector<string> params;
  params.push_back(door);
  LogicalNavigation senseDoor("sensedoor",params);

  senseDoor.run();

  ros::ServiceClient currentClient = n.serviceClient<bwi_kr_execution::CurrentStateQuery> ("current_state_query");
  bwi_kr_execution::AspFluent openFluent;
  openFluent.name = "open";
  openFluent.timeStep = 0;
  openFluent.variables.push_back(door);

  bwi_kr_execution::AspRule rule;
  rule.head.push_back(openFluent);

  bwi_kr_execution::CurrentStateQuery csq;
  csq.request.query.push_back(rule);

  currentClient.call(csq);

  done = csq.response.answer.satisfied;

}


actasp::Action* WaitForOpen::cloneAndInit(const actasp::AspFluent& fluent) const {
  WaitForOpen *newAction = new WaitForOpen();
  newAction->door = fluent.getParameters().at(0);

  return newAction;
}

std::vector<std::string> WaitForOpen::getParameters() const {
  vector<string> param;
  param.push_back(door);
  return param;
}

bwi_krexec::ActionFactory waitForOpen(new WaitForOpen(), true);

}
