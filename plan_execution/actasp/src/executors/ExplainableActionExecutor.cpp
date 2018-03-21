#include <actasp/executors/ExplainableActionExecutor.h>

#include <actasp/AspKR.h>
#include <actasp/AnswerSet.h>
#include <actasp/Planner.h>
#include <actasp/Action.h>
#include <actasp/action_utils.h>
#include <actasp/ExecutionObserver.h>
#include <actasp/PlanningObserver.h>
#include <actasp/execution_observer_utils.h>

using namespace std;

namespace actasp {

ExplainableActionExecutor::ExplainableActionExecutor(actasp::AspKR* reasoner,
    actasp::Planner *planner,
    const std::map<std::string, Action * > &actionMap
                                                  ) throw (std::invalid_argument) :
  ReplanningActionExecutor(reasoner, planner, actionMap) {}

struct NotifyNewPlan {

  NotifyNewPlan(const AnswerSet& plan) : plan(plan) {}

  void operator()(PlanningObserver* observer) {
    observer->planChanged(plan);
  }

  AnswerSet plan;

};

void ExplainableActionExecutor::computePlan() {

  isGoalReached = kr->currentStateQuery(goalRules).isSatisfied();

  AnswerSet answer;
  if (!isGoalReached) {
    answer = planner->computePlan(goalRules);
    plan = answer.instantiateActions(actionMap);
    actionCounter = 0;
  }

  hasFailed = plan.empty();

  if(!hasFailed)
    for_each(planningObservers.begin(),planningObservers.end(),NotifyNewPlan(answer));

}

}