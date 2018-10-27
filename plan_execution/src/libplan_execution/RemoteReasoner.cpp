#include "plan_execution/RemoteReasoner.h"

#include "plan_execution/CurrentStateQuery.h"
#include "plan_execution/UpdateFluents.h"
#include "plan_execution/ComputePlan.h"
#include "plan_execution/ComputeAllPlans.h"
#include "plan_execution/IsPlanValid.h"

#include "plan_execution/msgs_utils.h"

#include "actasp/Action.h"

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <algorithm>
#include <iterator>

using namespace std;
using namespace ros;

namespace plan_exec {

RemoteReasoner::RemoteReasoner(actasp::FilteringQueryGenerator *actualReasoner,unsigned int max_n,const actasp::ActionSet& allActions) :
  local(actualReasoner,max_n,allActions) {}

actasp::AnswerSet RemoteReasoner::currentStateQuery(const std::vector<actasp::AspRule>& query) const noexcept {
  return local.currentStateQuery(query);
}

actasp::ActionSet RemoteReasoner::availableActions() const noexcept {
  return local.availableActions();
}

std::list< std::list<actasp::AspAtom> > RemoteReasoner::query(const std::string &queryString, unsigned int timeStep) const noexcept {
  return local.query(queryString,timeStep);
}

bool RemoteReasoner::updateFluents(const std::vector<actasp::AspFluent> &observations) noexcept {
  NodeHandle n;
  ros::ServiceClient updateClient = n.serviceClient<plan_execution::UpdateFluents> ("update_fluents");
  updateClient.waitForExistence();

  plan_execution::UpdateFluents uf;

  transform(observations.begin(),observations.end(),back_inserter(uf.request.fluents),TranslateFluent());

  updateClient.call(uf);

  return uf.response.consistent;
}

actasp::AnswerSet RemoteReasoner::computePlan(const std::vector<actasp::AspRule>& goal) const throw (std::logic_error) {

  return local.computePlan(goal);

}


bool RemoteReasoner::isPlanValid(const actasp::AnswerSet& plan, const std::vector<actasp::AspRule>& goal)  const noexcept {
  return local.isPlanValid(plan,goal);

}

std::vector< actasp::AnswerSet > RemoteReasoner::computeAllPlans(
  const std::vector<actasp::AspRule>& goal,
  double suboptimality) const throw (std::logic_error) {

  return local.computeAllPlans(goal,suboptimality);


}

actasp::AnswerSet RemoteReasoner::computeOptimalPlan(const std::vector<actasp::AspRule>& goal,
                                                    double suboptimality, bool minimum, bool filterActions) const throw (std::logic_error) {
    return local.computeOptimalPlan(goal,suboptimality, minimum, filterActions);
  }

actasp::GraphPolicy* RemoteReasoner::computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error) {
 return local.computePolicy(goal,suboptimality);
}
  
  
actasp::AnswerSet RemoteReasoner::filterState(const std::vector<actasp::AnswerSet>& plans, const std::vector<actasp::AspRule>& goals) {
  return local.filterState(plans,goals);
}



void RemoteReasoner::resetCurrentState() noexcept {
  NodeHandle n;
  ros::ServiceClient resetClient = n.serviceClient<std_srvs::Empty> ("reset_state");
  resetClient.waitForExistence();

  std_srvs::Empty empty;
  resetClient.call(empty);
}


}
