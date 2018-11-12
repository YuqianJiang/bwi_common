
#include "plan_execution/msgs_utils.h"
#include "plan_execution/RemoteReasoner.h"

#include "actasp/action_utils.h"
#include "actasp/executors/ReplanningPlanExecutor.h"
#include "actasp/executors/PetlonPlanExecutor.h"
#include "actasp/executors/PeorlPlanExecutor.h"
#include "actasp/ExecutionObserver.h"
#include "actasp/PlanningObserver.h"
#include "actasp/AnswerSet.h"
#include <actasp/reasoners/Clingo.h>

#include "plan_execution/ExecutePlanAction.h"
#include <plan_execution/PlanExecutorNode.h>
#include <plan_execution/observers.h>

#include "observers.h"
#include "utils.h"
#include "BwiResourceManager.h"
#include "ActionCostUpdater.h"
#include "PeorlCostLearner.h"

#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <boost/filesystem.hpp>

#include <string>

#include <iostream>
#include <fstream>
#include "actions/action_registry.h"

using namespace plan_exec;
using namespace bwi_krexec;
using namespace std;
using namespace actasp;

const static std::string memory_log_path = string("/tmp/bwi_action_execution_logs/");
const int MAX_N = 30;
const int PLANNER_TIMEOUT = 5; //seconds

std::string working_memory_path;
void updateFacts() {

  string as_string = memoryConduitToAsp();
  std::ofstream working_memory(working_memory_path.c_str());
  working_memory << as_string;
  working_memory.close();

  {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    stringstream stampstream;
    stampstream << std::put_time(&tm, "%d-%m-%Y_%H-%M-%S");
    string log_name = stampstream.str() + ".asp";

    ofstream log((memory_log_path + log_name).c_str());
    log << as_string;
    log.close();
  }
}


int main(int argc, char**argv) {
  ros::init(argc, argv, "action_executor");
  ros::NodeHandle n;

//   if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
//     ros::console::notifyLoggerLevelsChanged();
//   }
  
  ros::NodeHandle privateNode("~");
  string domainDirectory;
  
  n.param<std::string>("bwi_kr_execution/domain_directory", domainDirectory, ros::package::getPath("bwi_kr_execution")+"/domain/");
  
  if(domainDirectory.at(domainDirectory.size()-1) != '/')
    domainDirectory += '/';


  bool simulating;
  bool use_learning;
  bool use_motion_cost;
  privateNode.param<bool>("simulation",simulating,false);
  privateNode.param<bool>("use_learning", use_learning, true);
  privateNode.param<bool>("use_motion_cost", use_motion_cost, true);

  if (use_learning) {
    ROS_INFO("!!!!!!!!using learning!!!!!!!");
  }
  if (use_motion_cost) {
    ROS_INFO("!!!!!!!!using motion cost!!!!!!!");
  }
  

  working_memory_path = "/tmp/current.asp";
  string cost_memory_path = "/tmp/costs.asp";
  string constraint_path = "/tmp/constraint.asp";
  string distance_path = ros::package::getPath("utexas_gdc") + "/maps/simulation/multimap/3ne/distances.asp";

  // Touch the memory files so the reasoner can verify that it exists
  fstream fs;
  fs.open(working_memory_path, ios::out);
  fs.close();

  fs.open(cost_memory_path, ios::out);
  fs.close();

  fs.open(constraint_path, ios::out);
  if (!use_learning) {
    fs << "#program step(n).\n:~ cost(X,Y). [X@1,Y]\n";
    fs.flush();
  }
  fs.close();

  unique_ptr<BwiResourceManager> resourceManager = unique_ptr<BwiResourceManager>(new BwiResourceManager());

  map<string, actasp::ActionFactory> &actions = bwi_krexec::real_actions;
  if (simulating) {
    actions = bwi_krexec::simulated_actions;
  }

  set<string> evaluable_actions = {"navigate_to", "go_through"};

  set<string> state_fluents = {"is_in", "is_near"};

  FilteringQueryGenerator *generator;
  if (use_motion_cost) {
    generator = Clingo::getQueryGenerator("n", domainDirectory, {distance_path},
                                                                {working_memory_path, cost_memory_path, constraint_path},
                                                                actionMapToSet(actions),
                                                                PLANNER_TIMEOUT);
  }
  else {
    generator = Clingo::getQueryGenerator("n", domainDirectory, {working_memory_path, cost_memory_path, constraint_path},
                                                                actionMapToSet(actions),
                                                                PLANNER_TIMEOUT);
  }
  unique_ptr<actasp::AspKR> planningReasoner = unique_ptr<actasp::AspKR>(new RemoteReasoner(generator, MAX_N, actionMapToSet(actions)));

  ReplanningPlanExecutor* replanner;
  unique_ptr<TaskPlanTracker> tracker = unique_ptr<TaskPlanTracker>(new TaskPlanTracker());

  if (!use_learning) {
    replanner = new PetlonPlanExecutor(*planningReasoner, *planningReasoner, actions, evaluable_actions, state_fluents, *resourceManager, use_motion_cost);
  }
  else {
    replanner = new PeorlPlanExecutor(*planningReasoner, *planningReasoner, actions, evaluable_actions, state_fluents, *resourceManager, *tracker);
  }

  PlanExecutor* executor = replanner;

  ConsoleObserver observer;
  std::function<void()> function = std::function<void()>(updateFacts);
  KnowledgeUpdater updating_observer(function, *resourceManager);
  ActionCostUpdater* action_cost_updater;
  PeorlCostLearner* cost_learner;

  replanner->addPlanningObserver(observer);
  executor->addExecutionObserver(observer);

  replanner->addPlanningObserver(updating_observer);
  executor->addExecutionObserver(updating_observer);

  if (!use_learning) {    
    action_cost_updater = new ActionCostUpdater(actions, evaluable_actions, state_fluents, *resourceManager, use_motion_cost);
    replanner->addPlanningObserver(*action_cost_updater);
    executor->addExecutionObserver(*action_cost_updater);
  }
  else {
    cost_learner = new PeorlCostLearner(actions, evaluable_actions, state_fluents, *resourceManager, *tracker, use_motion_cost);
    replanner->addPlanningObserver(*cost_learner);
    executor->addExecutionObserver(*cost_learner);
  }

  PlanExecutorNode node(executor);

  auto diagnosticsPath = boost::filesystem::path(domainDirectory) / "diagnostics";
  RosActionServerInterfaceObserver* ros_observer;

  if (boost::filesystem::is_directory(diagnosticsPath)) {
    auto diagnosticReasoner = std::unique_ptr<actasp::QueryGenerator>(actasp::Clingo::getQueryGenerator("n", diagnosticsPath.string(), {working_memory_path}, {}, PLANNER_TIMEOUT));
    ros_observer = new ExplainingRosActionServerInterfaceObserver(node.getActionServer(), std::move(diagnosticReasoner));
  } else {
    ros_observer = new RosActionServerInterfaceObserver(node.getActionServer());
  }

  node.setRosObserver(ros_observer);

  ros::spin();

  
  return 0;
}
