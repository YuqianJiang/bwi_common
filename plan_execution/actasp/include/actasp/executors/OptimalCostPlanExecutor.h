#pragma once

#include <actasp/executors/ReplanningPlanExecutor.h>
#include <actasp/MultiPlanner.h>

namespace actasp {

class OptimalCostPlanExecutor : public ReplanningPlanExecutor {

public:

  OptimalCostPlanExecutor(AspKR &reasoner,
                     MultiPlanner &planner,
                     const std::map<std::string, ActionFactory> &actionMap,
                     const std::set<std::string> &evaluableActionSet,
                     const std::set<std::string> &stateFluentSet,
                     actasp::ResourceManager &resourceManager,
                     bool evaluate_actions = true
  ) noexcept(false);

  void setGoal(const std::vector<actasp::AspRule> &goalRules) noexcept;

  ~OptimalCostPlanExecutor();


private:
  virtual void computePlan();

  MultiPlanner &optimal_planner_;
  std::set<std::string> evaluated_pairs_;
  std::set<std::string> evaluableActionSet;
  std::set<std::string> stateFluentSet;
  bool evaluate_actions_;


};


}

