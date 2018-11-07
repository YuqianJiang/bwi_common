#pragma once

#include <actasp/AnswerSet.h>
#include <actasp/ResourceManager.h>
#include <actasp/action_utils.h>
#include <actasp/state_utils.h>
#include <actasp/AspTask.h>
#include <actasp/executors/PeorlPlanExecutor.h>
#include "actions/ActionCostEstimator.h"
#include <knowledge_representation/Instance.h>

#include <ros/package.h>

#include <pybind11/embed.h> // everything needed for embedding
#include <pybind11/stl.h> // ...plus this

namespace py = pybind11;

namespace bwi_krexec {

#pragma GCC visibility push(hidden)

struct PeorlCostLearner : public actasp::ExecutionObserver, public actasp::PlanningObserver {

  explicit PeorlCostLearner(const std::map<std::string, actasp::ActionFactory> &actionMap, 
                            const std::set<std::string> &evaluableActionSet, 
                            const std::set<std::string> &stateFluentSet,
                            actasp::ResourceManager &resourceManager,
                            actasp::TaskPlanTracker &taskPlanTracker):
    actionMap(actionMap),
    evaluableActionSet(evaluableActionSet),
    stateFluentSet(stateFluentSet),
    resourceManager(resourceManager),
    tracker(taskPlanTracker),
    ltmc(dynamic_cast<BwiResourceManager&>(resourceManager).ltmc),
    guard(),
    learners_map(),
    goal() {
      std::string path = ros::package::getPath("bwi_kr_execution") + "/src/bwi_kr_execution";

      py::module sys = py::module::import("sys");
      sys.attr("path").attr("append")(path);

      learner_class = py::module::import("cost_learner").attr("CostLearner");
      //learner = learner_class();
    }

  void planChanged(const actasp::AnswerSet &newPlan) noexcept override {
      ActionCostEstimator estimator(resourceManager);
      py::object& learner = learners_map[goal];
      std::vector<std::pair<std::vector<std::string>, std::string>> path;

      for (int i = 0; i < newPlan.maxTimeStep(); ++i) {
        auto actions = extractActions(newPlan.getFluentsAtTime(i+1), actionMapToSet(actionMap));

        // not likely to happen
        if (actions.size() == 0) {
          ROS_INFO_STREAM("Cannot find action at time step " << i);
          continue;
        }

        std::vector<std::string> state;
        getStateAtTime(newPlan, i, state);

        std::vector<std::string> state_next;
        getStateAtTime(newPlan, i+1, state_next);

        std::string action = actions.begin()->toStringNoTimeStep();

        path.push_back({state, action});

        if (evaluableActionSet.find(actions.begin()->getName()) != evaluableActionSet.end()) {

          float cost = estimator.getActionCost(*actions.begin());

          learner.attr("learn")(state, state_next, action, cost);
        }
        /*else {
          ROS_INFO_STREAM(actions.begin()->getName() << " not evaluable");
        }*/
      }

      learner.attr("table_to_asp")("ro_table");

      //constrain
      learner.attr("constrain_plan_quality")(path);
  }

  void actionStarted(const actasp::AspFluent &action) noexcept override {
      learners_map[goal].attr("clear_constraint")();
    }

  void actionTerminated(const actasp::AspFluent &action, bool succeeded) noexcept override {
      //ROS_INFO_STREAM("Terminating execution: " << action.toString() << " Success:" << succeeded);
  }

  void planTerminated(const PlanStatus status, const actasp::AspFluent &final_action, const actasp::AnswerSet &plan_remainder) noexcept override {

  }

  void goalChanged(const std::vector<actasp::AspRule>& newGoalRules) noexcept override {
    tracker.currentTask = actasp::AspTask(getStateFluents(), newGoalRules);
    std::cout << "The task is " << tracker.currentTask.name << std::endl;

    goal = tracker.currentTask.getGoalString();
    if (learners_map.find(goal) == learners_map.end()) {
      learners_map[goal] = learner_class();
    }


    std::vector<std::pair<std::vector<std::string>, std::string>> path;
    auto& plan = tracker.getCurrentPlan();

    if (!plan.isSatisfied()) {
      learners_map[goal].attr("clear_constraint")();
      return;
    }

    for (int i = 0; i < plan.maxTimeStep(); ++i) {
        auto actions = extractActions(plan.getFluentsAtTime(i+1), actionMapToSet(actionMap));

        // not likely to happen
        if (actions.size() == 0) {
          ROS_INFO_STREAM("Cannot find action at time step " << i);
          continue;
        }

        std::vector<std::string> state;
        getStateAtTime(plan, i, state);

        std::string action = actions.begin()->toStringNoTimeStep();

        path.push_back({state, action});
    }

    learners_map[goal].attr("constrain_plan_quality")(path);
  }

  void policyChanged(actasp::PartialPolicy *policy) noexcept override {}

  void getStateAtTime(const actasp::AnswerSet &plan, int time, std::vector<std::string> &state) {
    auto fluents = filterFluents(plan.getFluentsAtTime(time), stateFluentSet);
    transform(fluents.begin(), fluents.end(), std::back_inserter(state), 
                    [](const actasp::AspFluent& fluent){return fluent.toStringNoTimeStep();});
  }

  std::vector<actasp::AspFluent> getStateFluents() {
    std::vector<actasp::AspFluent> fluents;

    knowledge_rep::Instance self = ltmc.get().get_robot();

    for (const auto& fluent_name : stateFluentSet) {
      std::vector<knowledge_rep::EntityAttribute> attributes = self.get_attributes(fluent_name);

      for (const auto& attribute : attributes) {
        std::vector<std::string> params = {std::to_string(attribute.entity_id), std::to_string(attribute.get_int_value())};
        actasp::AspFluent fluent(attribute.attribute_name, params, 0);
        fluents.push_back(fluent);
      }
    }

    return fluents;

  }

private:
  std::map<std::string, actasp::ActionFactory> actionMap;
  std::set<std::string> evaluableActionSet;
  std::set<std::string> stateFluentSet;
  actasp::ResourceManager &resourceManager;
  actasp::TaskPlanTracker &tracker;
  std::reference_wrapper<knowledge_rep::LongTermMemoryConduit> ltmc;

  std::map<std::string, float> cost_map_;
  py::scoped_interpreter guard;
  py::object learner_class;

  std::map<std::string, py::object> learners_map;
  std::string goal;

};
#pragma GCC visibility pop

}