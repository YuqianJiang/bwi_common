#include "WaitForOpen.h"

#include "bwi_kr_execution/CurrentStateQuery.h"
#include "bwi_msgs/DoorHandlerInterface.h"

#include "ActionFactory.h"
#include "LogicalNavigation.h"

#include "actasp/AspFluent.h"


using namespace std;
using namespace ros;

namespace bwi_krexec {


WaitForOpen::WaitForOpen() : door(), open(false), done(false), failed(false) {}

ros::Subscriber WaitForOpen::open_listener;
bool WaitForOpen::subscriber_set(false);

void WaitForOpen::openCallback(const std_msgs::String::ConstPtr& msg) {
  if (msg->data == door) {
    open = true;
  }
}

void WaitForOpen::run() {
  NodeHandle n;

  if (!open) {
    if (!subscriber_set) {
      open_listener = n.subscribe("open_door", 1, &WaitForOpen::openCallback, this);
    }
    return;
  }

  vector<string> params;
  params.push_back(door);
  LogicalNavigation approach("approach",params);

  approach.run();

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

  failed = csq.response.answer.satisfied;
  done = true;

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

bwi_krexec::ActionFactory waitForOpen(new WaitForOpen());

}
