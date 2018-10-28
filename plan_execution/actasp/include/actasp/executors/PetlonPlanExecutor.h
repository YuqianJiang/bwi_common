#pragma once

#include <actasp/executors/ReplanningPlanExecutor.h>
#include <actasp/MultiPlanner.h>

namespace actasp {

class PetlonPlanExecutor : public ReplanningPlanExecutor {

public:

  PetlonPlanExecutor(AspKR &reasoner,
                     MultiPlanner &planner,
                     const std::map<std::string, ActionFactory> &actionMap,
                     const std::map<std::string, CostFactory> &evaluableActionMap,
                     actasp::ResourceManager &resourceManager
  ) noexcept(false);


  ~PetlonPlanExecutor();


private:
  virtual void computePlan();

  MultiPlanner &optimal_planner_;
  std::set<std::string> evaluated_pairs_;
  std::map<std::string, CostFactory> evaluableActionMap_;


};


}

