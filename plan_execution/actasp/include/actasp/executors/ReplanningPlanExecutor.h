#pragma once

#include <actasp/PlanExecutor.h>
#include <actasp/PlanningObserver.h>
#include <actasp/AnswerSet.h>

#include <stdexcept>
#include <list>
#include <map>
#include <actasp/Action.h>

namespace actasp {

class AspKR;

class Planner;

class Action;

class PlanningObserver;

struct NotifyNewPlan {

  explicit NotifyNewPlan(AnswerSet plan) : plan(std::move(plan)) {}

  void operator()(PlanningObserver &observer) {
    observer.planChanged(plan);
  }

  AnswerSet plan;

};

class ReplanningPlanExecutor : public PlanExecutor {

public:

  ReplanningPlanExecutor(AspKR &reasoner,
                         Planner &planner,
                         const std::map<std::string, ActionFactory> &actionMap,
                         actasp::ResourceManager &resourceManager
  ) noexcept(false);

  using PlanExecutor::setGoal;

  virtual void setGoal(const std::vector<actasp::AspRule> &goalRules) noexcept;

  bool goalReached() const noexcept {
    return isGoalReached;
  }

  bool failed() const noexcept {
    return hasFailed;
  }

  void executeActionStep() override;

  void addExecutionObserver(ExecutionObserver &observer) noexcept override;

  void removeExecutionObserver(ExecutionObserver &observer) noexcept override;

  void addPlanningObserver(PlanningObserver &observer) noexcept;

  void removePlanningObserver(PlanningObserver &observer) noexcept;


  ~ReplanningPlanExecutor();


protected:
  std::vector<actasp::AspRule> goalRules;
  bool isGoalReached;
  bool hasFailed;
  std::map<std::string, ActionFactory> actionMap;

  std::list<std::unique_ptr<Action>> plan;
  unsigned int actionCounter;
  bool newAction;
  unsigned int failureCount;

  AspKR &kr;
  Planner &planner;
  ResourceManager &resourceManager;

  std::list<std::reference_wrapper<ExecutionObserver>> executionObservers;
  std::list<std::reference_wrapper<PlanningObserver>> planningObservers;

private:
  virtual void computePlan();


};


}

