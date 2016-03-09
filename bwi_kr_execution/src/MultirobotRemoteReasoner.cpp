#include "MultirobotRemoteReasoner.h"

#include "bwi_kr_execution/MultirobotComputePlan.h"
#include "bwi_kr_execution/MultirobotUpdateFluents.h"

#include "msgs_utils.h"
#include <std_srvs/Empty.h>

#include <algorithm>
#include <iterator>

#include <ros/ros.h>

using namespace std;

namespace bwi_krexec {

MultirobotRemoteReasoner::MultirobotRemoteReasoner(string name,actasp::FilteringQueryGenerator *actualReasoner,unsigned int max_n,const actasp::ActionSet& allActions) :
  name(name), local(actualReasoner,max_n,allActions) {}

actasp::AnswerSet MultirobotRemoteReasoner::currentStateQuery(const std::vector<actasp::AspRule>& query) const throw() {
  return local.currentStateQuery(query);
}

actasp::ActionSet MultirobotRemoteReasoner::availableActions() const throw() {
  return local.availableActions();
}

bool MultirobotRemoteReasoner::isPlanValid(const actasp::AnswerSet& plan, const std::vector<actasp::AspRule>& goal)  const throw() {
  return local.isPlanValid(plan,goal);
}

std::list< std::list<actasp::AspAtom> > MultirobotRemoteReasoner::query(const std::string &queryString, unsigned int timeStep) const throw() {
  return local.query(queryString,timeStep);
}

std::vector< actasp::AnswerSet > MultirobotRemoteReasoner::computeAllPlans(const std::vector<actasp::AspRule>& goal,double suboptimality) const throw (std::logic_error) {
  return local.computeAllPlans(goal,suboptimality);
}

actasp::PartialPolicy* MultirobotRemoteReasoner::computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error) {
 return local.computePolicy(goal,suboptimality);
}

bool MultirobotRemoteReasoner::updateFluents(const std::vector<actasp::AspFluent> &observations) throw() {
  //
  ros::NodeHandle n;
  ros::ServiceClient updateClient = n.serviceClient<bwi_kr_execution::MultirobotUpdateFluents>("multirobot_update_fluents");
  updateClient.waitForExistence();

  bwi_kr_execution::MultirobotUpdateFluents uf;
  uf.request.name = this->name;
  transform(observations.begin(),observations.end(),back_inserter(uf.request.fluents),TranslateFluent());

  updateClient.call(uf);

  set<actasp::AspFluent> state;
  transform(uf.response.state.begin(),uf.response.state.end(),inserter(state,state.begin()),TranslateFluent());
  local.setCurrentState(state);

  return true;
}

actasp::AnswerSet MultirobotRemoteReasoner::computePlan(const std::vector<actasp::AspRule>& goal) const throw (std::logic_error) {
	//
  ros::NodeHandle n;
  ros::ServiceClient planClient = n.serviceClient<bwi_kr_execution::MultirobotComputePlan>("multirobot_compute_plan");
  planClient.waitForExistence();

  bwi_kr_execution::MultirobotComputePlan cp;
  cp.request.name = this->name;
  transform(goal.begin(),goal.end(),back_inserter(cp.request.goal),TranslateRule());
  
  planClient.call(cp);

  TranslateAnswerSet translate;
  return translate(cp.response.plan);
}

void MultirobotRemoteReasoner::resetCurrentState() throw() {
  ros::NodeHandle n;
  ros::ServiceClient resetClient = n.serviceClient<std_srvs::Empty> ("reset_state");
  resetClient.waitForExistence();

  std_srvs::Empty empty;
  resetClient.call(empty);
}

}