#pragma once

#include <actasp/executors/ReplanningPlanExecutor.h>
#include <actasp/MultiPlanner.h>
#include <actasp/AspTask.h>

namespace actasp {

struct TaskPlanTracker {

  TaskPlanTracker() :
    taskPlanMap(),
    currentTask({}, {}) {
  }

  AnswerSet& getCurrentPlan() {
    return taskPlanMap[currentTask];
  }

  void setPlan(AnswerSet &answer) {
    taskPlanMap[currentTask] = answer;
  }

  std::map<AspTask, AnswerSet> taskPlanMap;
  AspTask currentTask;
};

class IterativePlanningPlanExecutor : public ReplanningPlanExecutor {

public:

  IterativePlanningPlanExecutor(AspKR &reasoner,
                     MultiPlanner &planner,
                     const std::map<std::string, ActionFactory> &actionMap,
                     const std::set<std::string> &evaluableActionSet,
                     const std::set<std::string> &stateFluentSet,
                     actasp::ResourceManager &resourceManager,
                     TaskPlanTracker &taskPlanTracker
  ) noexcept(false);

  void setGoal(const std::vector<actasp::AspRule> &goalRules) noexcept;

  ~IterativePlanningPlanExecutor();


private:
  virtual void computePlan();

  MultiPlanner &planner;
  std::set<std::string> evaluableActionSet;
  std::set<std::string> stateFluentSet;
  TaskPlanTracker &tracker;

};


}

