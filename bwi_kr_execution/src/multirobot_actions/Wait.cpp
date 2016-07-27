#include "Wait.h"

#include "ActionFactory.h"

#include "actasp/AspFluent.h"

#include <ros/ros.h>

using namespace std;
using namespace ros;

namespace bwi_krexec {


Wait::Wait() : time(), done(false) {}

void Wait::run() {

  ros::Duration(time*9).sleep();

  done = true;

}


actasp::Action* Wait::cloneAndInit(const actasp::AspFluent& fluent) const {
  Wait *newAction = new Wait();
  newAction->time = atoi(fluent.getParameters().at(0).c_str());

  return newAction;
}

std::vector<std::string> Wait::getParameters() const {
  vector<string> param;
  stringstream ss;
  ss << time;
  param.push_back(ss.str());
  return param;
}

bwi_krexec::ActionFactory wait(new Wait(), true);

}
