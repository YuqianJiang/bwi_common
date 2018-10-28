#include <actasp/executors/PetlonPlanExecutor.h>

#include <actasp/AspKR.h>
#include <actasp/ExecutionObserver.h>

#include <iostream>
#include <actasp/action_utils.h>
#include <actasp/execution_observer_utils.h>

using namespace std;

namespace actasp {

PetlonPlanExecutor::PetlonPlanExecutor(AspKR &reasoner,
                                       MultiPlanner &planner,
                                       const std::map<std::string, ActionFactory> &actionMap,
                                       const std::map<std::string, CostFactory> &evaluableActionMap,
                                       ResourceManager &resourceManager
) noexcept(false) :
    ReplanningPlanExecutor(reasoner, planner, actionMap, resourceManager),
    optimal_planner_(planner),
    evaluableActionMap_(evaluableActionMap),
    evaluated_pairs_() {

}

PetlonPlanExecutor::~PetlonPlanExecutor() = default;

void PetlonPlanExecutor::computePlan() {
  isGoalReached = kr.currentStateQuery(goalRules).isSatisfied();

  if (isGoalReached) return;

  bool evaluation = true;

  while (evaluation) {
    AnswerSet answer = optimal_planner_.computeOptimalPlan(goalRules, 1, true, false);
    //AnswerSet answer = optimal_planner_.computePlan(goalRules);
    plan = answer.instantiateActions(actionMap, resourceManager);

    hasFailed = plan.empty();

    if (hasFailed) return;

    for_each(planningObservers.begin(), planningObservers.end(), NotifyNewPlan(answer));

    //check if new evaluation was needed
    evaluation = false;

    auto actIt = plan.begin();
    
    for (int i = 0; i < answer.maxTimeStep(); ++i, ++actIt) {

      if (evaluableActionMap_.find(actIt->get()->getName()) == evaluableActionMap_.end()) {
        continue;
      }

      string state = stateToString(removeActions(answer.getFluentsAtTime(i), actionMapToSet(actionMap)));
      string action = actIt->get()->toASP();

      string state_action = state + "," + action;

      cout << state_action << endl;

      if (evaluated_pairs_.find(state_action) == evaluated_pairs_.end()) {
        evaluation = true;
        evaluated_pairs_.insert(state_action);
      }
    }

  }

  actionCounter = 0;
  

}

}
