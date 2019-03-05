#include <actasp/executors/IterativePlanningPlanExecutor.h>

#include <actasp/AspKR.h>
#include <actasp/ExecutionObserver.h>

#include <iostream>
#include <actasp/action_utils.h>
#include <actasp/execution_observer_utils.h>
#include <actasp/AspTask.h>

using namespace std;

namespace actasp {

IterativePlanningPlanExecutor::IterativePlanningPlanExecutor(AspKR &reasoner,
                                       MultiPlanner &planner,
                                       const std::map<std::string, ActionFactory> &actionMap,
                                       const std::set<std::string> &evaluableActionSet,
                                       const std::set<std::string> &stateFluentSet,
                                       ResourceManager &resourceManager,
                                       TaskPlanTracker &taskPlanTracker
) noexcept(false) :
    ReplanningPlanExecutor(reasoner, planner, actionMap, resourceManager),
    planner(planner),
    evaluableActionSet(evaluableActionSet),
    stateFluentSet(stateFluentSet),
    tracker(taskPlanTracker) {

}

IterativePlanningPlanExecutor::~IterativePlanningPlanExecutor() = default;

void IterativePlanningPlanExecutor::computePlan() {
  isGoalReached = kr.currentStateQuery(goalRules).isSatisfied();

  if (isGoalReached) return;

  while (true) {
    AnswerSet answer = planner.computePlan(goalRules);

    if (!answer.isSatisfied()) {
      //std::cout << "!!!!!!No better plan!!!!!" << std::endl;
      if (!tracker.getCurrentPlan().isSatisfied()) {
        hasFailed = true;
        //std::cout << "No valid plan!!!!!!" << std::endl;
      }
      else { 
        plan = tracker.getCurrentPlan().instantiateActions(actionMap, resourceManager);
        std::cout << "Using last plan!!!!!!" << std::endl;
      }
      break;
    }

    for_each(planningObservers.begin(), planningObservers.end(), NotifyNewPlan(answer));

    tracker.setPlan(answer);

    hasFailed = false;

  }

  actionCounter = 0;
  newAction = true;
  
}

void IterativePlanningPlanExecutor::setGoal(const std::vector<actasp::AspRule> &goalRules) noexcept {

  this->goalRules = goalRules;

  for_each(executionObservers.begin(), executionObservers.end(), NotifyGoalChanged(goalRules));

  //AnswerSet currentState = kr.currentStateQuery({});
  
  //tracker.currentTask = AspTask(currentState.getFluents(), goalRules);
  //cout << "The task is " << tracker.currentTask.name << endl;

  computePlan();

  failureCount = 0;
}

}
