#pragma once

#include <actasp/AnswerSet.h>
#include <actasp/ResourceManager.h>
#include <actasp/action_utils.h>
#include <actasp/state_utils.h>
#include "actions/ActionCostEstimator.h"
#include <knowledge_representation/Instance.h>

#include <fstream>

#include <ros/package.h>

#include <chrono>

#include <pybind11/embed.h> // everything needed for embedding
#include <pybind11/stl.h> // ...plus this

namespace py = pybind11;
using namespace std::chrono;

namespace bwi_krexec {

#pragma GCC visibility push(hidden)

struct ActionCostUpdater : public actasp::ExecutionObserver, public actasp::PlanningObserver {

  explicit ActionCostUpdater(const std::map<std::string, actasp::ActionFactory> &actionMap, 
                            const std::set<std::string> &evaluableActionSet, 
                            const std::set<std::string> &stateFluentSet,
                            actasp::ResourceManager &resourceManager,
                            bool use_motion_cost = true):
    actionMap(actionMap),
    evaluableActionSet(evaluableActionSet),
    stateFluentSet(stateFluentSet),
    resourceManager(resourceManager),
    use_motion_cost(use_motion_cost),
    ltmc(dynamic_cast<BwiResourceManager&>(resourceManager).ltmc),
    guard(),
    action_start(),
    planning_start(),
    execution_start(),
    planning_finished(false),
    actionLog(),
    fs() {
      std::string path = ros::package::getPath("bwi_kr_execution") + "/src/bwi_kr_execution";

      py::module sys = py::module::import("sys");
      sys.attr("path").attr("append")(path);

      py::object learner_class = py::module::import("cost_learner").attr("CostLearner");
      learner = learner_class();

      auto t = std::time(nullptr);
      auto tm = *std::localtime(&t);
      std::stringstream stampstream;
      stampstream << std::put_time(&tm, "%d-%m-%Y_%H-%M-%S");
      auto results_path = "/tmp/results/results_petlon_" + stampstream.str() + ".csv";

      fs.open(results_path, std::ios::out);
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

        if ((use_motion_cost) && 
           (evaluableActionSet.find(actions.begin()->getName()) != evaluableActionSet.end())) {

          float cost = estimator.getActionCost(*actions.begin());

          auto fluents = filterFluents(newPlan.getFluentsAtTime(i), stateFluentSet);
          std::vector<std::string> state;
          transform(fluents.begin(), fluents.end(), std::back_inserter(state), 
                    [](const actasp::AspFluent& fluent){return fluent.toStringNoTimeStep();});

          std::vector<std::string> state_next = {};
          std::string action = actions.begin()->toStringNoTimeStep();

          learner.attr("learn")(state, state_next, action, -cost);
        }
        /*else {
          ROS_INFO_STREAM(actions.begin()->getName() << " not evaluable");
        }*/
      }

      learner.attr("table_to_asp")("cost_table");
  }

  void actionStarted(const actasp::AspFluent &action) noexcept override {
    if (! planning_finished) {
      execution_start = ros::Time::now();
      duration<double> time_span = duration_cast<duration<double>>(steady_clock::now() - planning_start);
      fs << (time_span.count()) << ",";
      fs.flush();
      ROS_INFO_STREAM("Planning time was: " << time_span.count());
      planning_finished = true;
    }

    actionLog << "\"" << action.toStringNoTimeStep() << "\"" << ",";

    action_start = ros::Time::now();
  }

  void actionTerminated(const actasp::AspFluent &action, bool succeeded) noexcept override {
    ros::Time action_end = ros::Time::now();

    actionLog << (action_end - action_start).toSec() << ",";
  }

  void planTerminated(const PlanStatus status, const actasp::AspFluent &final_action, const actasp::AnswerSet &plan_remainder) noexcept override {
    ROS_INFO("Plan terminated");

    ros::Time execution_end = ros::Time::now();

    fs << actasp::ExecutionObserver::planStatusToString(status) << ",";
    fs << (execution_end - execution_start).toSec() << ",";
    fs << actionLog.str() << "\n";
    fs.flush();

    actionLog.str("");
  }

  void goalChanged(const std::vector<actasp::AspRule>& newGoalRules) noexcept override {
    auto task = actasp::AspTask(getStateFluents(), newGoalRules);
    std::cout << "The task is " << task.name << std::endl;
    
    fs << "\"" << task.name << "\"" << ",";
    planning_finished = false;
    planning_start = steady_clock::now();
  }

  void policyChanged(actasp::PartialPolicy *policy) noexcept override {}

  void getStateAtTime(const actasp::AnswerSet &plan, int t, std::vector<std::string> &state) {
    auto fluents = filterFluents(plan.getFluentsAtTime(t), stateFluentSet);
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
  bool use_motion_cost;
  std::reference_wrapper<knowledge_rep::LongTermMemoryConduit> ltmc;

  std::map<std::string, float> cost_map_;
  py::object learner;
  py::scoped_interpreter guard;

  ros::Time action_start;
  steady_clock::time_point planning_start;
  ros::Time execution_start;
  bool planning_finished;

  std::stringstream actionLog;
  std::fstream fs;


};
#pragma GCC visibility pop

}