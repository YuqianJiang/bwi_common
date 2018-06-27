
#include "plan_execution/msgs_utils.h"
#include "plan_execution/RemoteReasoner.h"
#include "PlanExplainer.h"
#include "plan_execution/ComputeAndExplainPlan.h"
#include <actasp/reasoners/Clingo4_5.h>

#include <ros/ros.h>
#include <boost/filesystem.hpp>

#include <string>
#include <iterator>

using namespace std;
using namespace actasp;
using namespace bwi_krexec;
using namespace plan_exec;

const int MAX_N = 30;
const int PLANNER_TIMEOUT = 40; //seconds
const std::string queryDirectory("~/explan/query");
const bool baseline = false;

AspKR *reasoner;
PlanExplainer *explainer;
ActionSet actionSet;

bool computeAndExplainPlan(plan_execution::ComputeAndExplainPlan::Request  &req,
                           plan_execution::ComputeAndExplainPlan::Response &res) {
  vector<AspRule> goal;
  transform(req.goal.begin(),req.goal.end(),back_inserter(goal),TranslateRule());

  //TODO catch exception
  AnswerSet answer = reasoner->computePlan(goal);

  stringstream planStream;

  ROS_INFO_STREAM("plan size: " << answer.maxTimeStep());

  remove_copy_if(answer.getFluents().begin(),
                    answer.getFluents().end(), 
                    ostream_iterator<string>(planStream," "), 
                    not1(IsAnAction(actionSet)));

  ROS_INFO_STREAM(planStream.str());

  /*explainer->setPlan(answer);
  //ROS_INFO_STREAM(explainer->getAllPairs());

  if (baseline) {
    res.plan_explanation = explainer->getRandomExplanation();
  }
  else {
    res.plan_explanation = explainer->getLearnedExplanation();
  }*/

  return true;
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "explain_plan_server");
  ros::NodeHandle n;

//   if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
//     ros::console::notifyLoggerLevelsChanged();
//   }
  
  ros::NodeHandle privateNode("~");
  string domainDirectory;
  n.param<std::string>("domain_directory", domainDirectory, "~/explan/domain/");
  
  if(domainDirectory.at(domainDirectory.size()-1) != '/')
    domainDirectory += '/';
  
  boost::filesystem::create_directories(queryDirectory);

  actionSet.insert(AspFluent("pick_up_from_surface", vector<string>(3), 0));
  actionSet.insert(AspFluent("pick_up_from_container", vector<string>(3), 0));
  actionSet.insert(AspFluent("put_on_surface", vector<string>(3), 0));
  actionSet.insert(AspFluent("put_in_container", vector<string>(3), 0));
  actionSet.insert(AspFluent("stack", vector<string>(3), 0));
  actionSet.insert(AspFluent("go_to", vector<string>(3), 0));
  actionSet.insert(AspFluent("open_door", vector<string>(2), 0));
  actionSet.insert(AspFluent("squeeze_container_above", vector<string>(3), 0));

  FilteringQueryGenerator *generator = new Clingo4_5("n",queryDirectory,domainDirectory,actionSet,PLANNER_TIMEOUT);

  reasoner = new RemoteReasoner(generator, MAX_N,actionSet);

  explainer = new PlanExplainer(actionSet);
  ros::ServiceServer compute_and_explain = n.advertiseService("compute_and_explain_plan", computeAndExplainPlan);
  

  ros::spin();

  delete generator;
  delete reasoner;
  delete explainer;

  
  return 0;
}