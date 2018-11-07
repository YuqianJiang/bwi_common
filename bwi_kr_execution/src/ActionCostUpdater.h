#pragma once

#include <actasp/AnswerSet.h>
#include <actasp/ResourceManager.h>
#include <actasp/action_utils.h>
#include <actasp/state_utils.h>
//#include "actions/action_registry.h"
#include "actions/ActionCostEstimator.h"

#include <ros/package.h>

#include <pybind11/embed.h> // everything needed for embedding
#include <pybind11/stl.h> // ...plus this

namespace py = pybind11;

namespace bwi_krexec {

#pragma GCC visibility push(hidden)

struct ActionCostUpdater : public actasp::PlanningObserver {

  explicit ActionCostUpdater(const std::map<std::string, actasp::ActionFactory> &actionMap, 
                            const std::set<std::string> &evaluableActionSet, 
                            const std::set<std::string> &stateFluentSet,
                            actasp::ResourceManager &resourceManager):
    actionMap(actionMap),
    evaluableActionSet(evaluableActionSet),
    stateFluentSet(stateFluentSet),
    resourceManager(resourceManager),
    guard() {
      std::string path = ros::package::getPath("bwi_kr_execution") + "/src/bwi_kr_execution";

      py::module sys = py::module::import("sys");
      sys.attr("path").attr("append")(path);

      py::object learner_class = py::module::import("cost_learner").attr("CostLearner");
      learner = learner_class();
    }

  void planChanged(const actasp::AnswerSet &newPlan) noexcept override {
      ActionCostEstimator estimator(resourceManager);

      for (int i = 0; i < newPlan.maxTimeStep(); ++i) {
        auto actions = extractActions(newPlan.getFluentsAtTime(i+1), actionMapToSet(actionMap));

        // not likely to happen
        if (actions.size() == 0) {
          ROS_INFO_STREAM("Cannot find action at time step " << i);
          continue;
        }

        if (evaluableActionSet.find(actions.begin()->getName()) != evaluableActionSet.end()) {

          float cost = estimator.getActionCost(*actions.begin());

          auto fluents = filterFluents(newPlan.getFluentsAtTime(i), stateFluentSet);
          std::vector<std::string> state;
          transform(fluents.begin(), fluents.end(), std::back_inserter(state), 
                    [](const actasp::AspFluent& fluent){return fluent.toStringNoTimeStep();});

          std::vector<std::string> state_next = {};
          std::string action = actions.begin()->toStringNoTimeStep();

          learner.attr("learn")(state, state_next, action, cost);
        }
        /*else {
          ROS_INFO_STREAM(actions.begin()->getName() << " not evaluable");
        }*/
      }

      learner.attr("table_to_asp")("cost_table");
  }

private:
  std::map<std::string, actasp::ActionFactory> actionMap;
  std::set<std::string> evaluableActionSet;
  std::set<std::string> stateFluentSet;
  actasp::ResourceManager &resourceManager;

  std::map<std::string, float> cost_map_;
  py::object learner;
  py::scoped_interpreter guard;

};
#pragma GCC visibility pop

}