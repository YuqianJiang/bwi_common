

#include <actasp/executors/MultirobotActionExecutor.h>

#include <actasp/AspKR.h>
#include <actasp/AnswerSet.h>
#include <actasp/Planner.h>
#include <actasp/Action.h>
#include <actasp/action_utils.h>
#include <actasp/ExecutionObserver.h>
#include <actasp/PlanningObserver.h>
#include <actasp/execution_observer_utils.h>

#include <list>
#include <algorithm>
#include <iterator>
#include <ros/time.h>
#include <ros/console.h>

using namespace std;

namespace actasp {

MultirobotActionExecutor::MultirobotActionExecutor(actasp::AspKR* reasoner,
    actasp::Planner *planner,
    const std::map<std::string, Action * > &actionMap
                                                  ) throw (std::invalid_argument) :
  goalRules(),
  isGoalSet(false),
  isGoalReached(true),
  hasFailed(false),
  actionMap(),
  plan(),
  actionCounter(0),
  newAction(true),
  kr(reasoner),
  planner(planner),
  executionObservers(),
  lastPlanTime(){
  if (reasoner == NULL)
    throw invalid_argument("MultirobotActionExecutor: reasoner is NULL");

  if (planner == NULL)
    throw invalid_argument("MultirobotActionExecutor: planner is NULL");

  transform(actionMap.begin(),actionMap.end(),inserter(this->actionMap,this->actionMap.begin()),ActionMapDeepCopy());
}

MultirobotActionExecutor::~MultirobotActionExecutor() {
  for_each(actionMap.begin(),actionMap.end(),ActionMapDelete());
}

struct NotifyNewPlan {
  
  NotifyNewPlan(const AnswerSet& plan) : plan(plan) {}
  
  void operator()(PlanningObserver* observer) {
    observer->planChanged(plan);
  }
  
  AnswerSet plan;
  
};

void MultirobotActionExecutor::computePlan() {
  isGoalReached = kr->currentStateQuery(goalRules).isSatisfied();

  if (!isGoalReached) {
    plan = planner->computePlan(goalRules).instantiateActions(actionMap);
    actionCounter = 0;
  }

  hasFailed = plan.empty();
  
  if(!hasFailed)
    for_each(planningObservers.begin(),planningObservers.end(),NotifyNewPlan(planToAnswerSet(plan)));

  lastPlanTime = ros::Time::now().toSec();
  
}

void MultirobotActionExecutor::setGoal(const std::vector<actasp::AspRule>& goalRules) throw() {
  this->goalRules = goalRules;
  this->isGoalSet = true;

  computePlan();
}

void MultirobotActionExecutor::acceptNewPlan(const double time, const actasp::AnswerSet newPlan) {
  if (time > lastPlanTime) {
    for_each(plan.begin(),plan.end(),ActionDeleter());
    plan.clear();
    plan = newPlan.instantiateActions(actionMap);
  
    isGoalReached = kr->currentStateQuery(goalRules).isSatisfied();
    if(plan.empty() && (!isGoalReached)) {
      hasFailed = true;
    }
    else {
      for_each(planningObservers.begin(),planningObservers.end(),NotifyNewPlan(planToAnswerSet(plan)));
      lastPlanTime = ros::Time::now().toSec();
      actionCounter = 0;
    }
    lastPlanTime = ros::Time::now().toSec();
  }
}


void MultirobotActionExecutor::executeActionStep() {

  if (isGoalReached || hasFailed)
    return;


  Action *current = plan.front();

  if(newAction) {
      for_each(executionObservers.begin(),executionObservers.end(),NotifyActionStart(current->toFluent(actionCounter)));
      newAction = false;
  }

  current->run();

  if (current->hasFinished()) {
    //destroy the action and pop a new one
    
    for_each(executionObservers.begin(),executionObservers.end(),NotifyActionTermination(current->toFluent(actionCounter++)));
    
    delete current;
    plan.pop_front();
    
    newAction = true;

    //if (plan.empty() || !kr->isPlanValid(planToAnswerSet(plan),goalRules)) {
    if (plan.empty()) {  
      //if not valid, replan
      for_each(plan.begin(),plan.end(),ActionDeleter());
      plan.clear();

      computePlan();
    }

  }
  
  
  
}

void MultirobotActionExecutor::addExecutionObserver(ExecutionObserver *observer) throw() {
  executionObservers.push_back(observer);
}

void MultirobotActionExecutor::removeExecutionObserver(ExecutionObserver *observer) throw() {
  executionObservers.remove(observer);
}

void MultirobotActionExecutor::addPlanningObserver(PlanningObserver *observer) throw() {
  planningObservers.push_back(observer);
}

void MultirobotActionExecutor::removePlanningObserver(PlanningObserver *observer) throw() {
  planningObservers.remove(observer);
}

}
