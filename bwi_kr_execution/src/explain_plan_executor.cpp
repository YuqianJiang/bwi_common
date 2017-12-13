
#include "msgs_utils.h"
#include "RemoteReasoner.h"
#include "StaticFacts.h"
#include "PlanExplainer.h"

#include "actasp/action_utils.h"
#include "actasp/executors/ExplainableActionExecutor.h"
#include "actasp/ExecutionObserver.h"
#include "actasp/PlanningObserver.h"
#include "actasp/AnswerSet.h"
#include "actasp/action_utils.h"
#include <actasp/reasoners/Clingo4_2.h>

#include "bwi_kr_execution/ExecutePlanAction.h"
#include "bwi_kr_execution/ComputeAndExplainPlan.h"

#include "actions/ActionFactory.h"
#include "actions/LogicalNavigation.h"

#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <boost/filesystem.hpp>

#include <string>
#include <iterator>

const int MAX_N = 30;
const int PLANNER_TIMEOUT = 40; //seconds
const std::string queryDirectory("/tmp/bwi_action_execution/");
const bool baseline = false;


using namespace std;
using namespace bwi_krexec;
using namespace actasp;

typedef actionlib::SimpleActionServer<bwi_kr_execution::ExecutePlanAction> Server;


ActionExecutor *executor;
AspKR *reasoner;
PlanExplainer *explainer;
IsAnAction isAnAction(actionMapToSet(ActionFactory::actions()));


class ExplainPlanObserver : public ExecutionObserver, public PlanningObserver {

public:
  ExplainPlanObserver(Server* server, PlanExplainer *explainer) : 
    server(server), explainer(explainer) {}

  void actionStarted(const AspFluent& action) throw() {
    ROS_INFO_STREAM("Starting execution: " << action.toString());
  }
  
  void actionTerminated(const AspFluent& action) throw() {
    ROS_INFO_STREAM("Terminating execution: " << action.toString());
  }
  
  
  void planChanged(const AnswerSet& newFluents) throw() {
    stringstream planStream;

    ROS_INFO_STREAM("plan size: " << newFluents.maxTimeStep());

    remove_copy_if(newFluents.getFluents().begin(),
                    newFluents.getFluents().end(), 
                    ostream_iterator<string>(planStream," "), 
                    not1(isAnAction));

    ROS_INFO_STREAM(planStream.str());

    explainer->setPlan(newFluents);

    bwi_kr_execution::ExecutePlanFeedback planFeedback;
    planFeedback.plan_explanation = explainer->getRandomExplanation();
    server->publishFeedback(planFeedback);

  }
  
  void goalChanged(std::vector<actasp::AspRule> newGoalRules) throw() {}
  
  void policyChanged(PartialPolicy* policy) throw() {}

private:
  Server* server;
  PlanExplainer* explainer;
};

void executePlan(const bwi_kr_execution::ExecutePlanGoalConstPtr& plan, Server* as) {

  vector<AspRule> goalRules;

  transform(plan->aspGoal.begin(),plan->aspGoal.end(),back_inserter(goalRules),TranslateRule());

  //Update fluents before sending new goals
  LogicalNavigation senseState("senseState");
  while (!senseState.hasFinished()) {
    senseState.run();
  }

  executor->setGoal(goalRules);

  ros::Rate loop(10);

  while (!executor->goalReached() && !executor->failed() && ros::ok() && as->isActive()) {

    if (!as->isPreemptRequested()) {
      executor->executeActionStep();
    }
    else {
      
      as->setPreempted();
      
      if (executor->goalReached()) 
        ROS_INFO("Preempted, but execution succeded");
      else 
        ROS_INFO("Preempted, execution aborted");
      
      if(as->isNewGoalAvailable()) {
        goalRules.clear();
        const bwi_kr_execution::ExecutePlanGoalConstPtr& newGoal = as->acceptNewGoal();
        transform(newGoal->aspGoal.begin(),newGoal->aspGoal.end(),back_inserter(goalRules),TranslateRule());

        //Update fluents before resending goal
        LogicalNavigation senseState("senseState");
        while (!senseState.hasFinished()) {
          senseState.run();
        }
        executor->setGoal(goalRules);
      }
    }
         loop.sleep();
  }


  if (executor->goalReached()) {
    ROS_INFO("Execution succeded");
    if(as->isActive())
      as->setSucceeded();
  } else {
    ROS_INFO("Execution failed");
   if(as->isActive())
    as->setAborted();
  }
}

bool computeAndExplainPlan(bwi_kr_execution::ComputeAndExplainPlan::Request  &req,
                           bwi_kr_execution::ComputeAndExplainPlan::Response &res) {
  vector<AspRule> goal;
  transform(req.goal.begin(),req.goal.end(),back_inserter(goal),TranslateRule());

  //TODO catch exception
  AnswerSet answer = reasoner->computePlan(goal);

  stringstream planStream;

  ROS_INFO_STREAM("plan size: " << answer.maxTimeStep());

  remove_copy_if(answer.getFluents().begin(),
                    answer.getFluents().end(), 
                    ostream_iterator<string>(planStream," "), 
                    not1(isAnAction));

  ROS_INFO_STREAM(planStream.str());

  explainer->setPlan(answer);
  //ROS_INFO_STREAM(explainer->getAllPairs());

  if (baseline) {
    res.plan_explanation = explainer->getRandomExplanation();
  }
  else {
    res.plan_explanation = explainer->getLearnedExplanation();
  }

  return true;
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

//  create initial state
  /*LogicalNavigation setInitialState("senseState");
  while (ros::ok() && (!setInitialState.hasFinished())) {
    setInitialState.run();
  }*/

  bool simulating;
  privateNode.param<bool>("simulation",simulating,false);
  ActionFactory::setSimulation(simulating); 
  
  boost::filesystem::create_directories(queryDirectory);

  FilteringQueryGenerator *generator = new Clingo4_2("n",queryDirectory,domainDirectory,actionMapToSet(ActionFactory::actions()),PLANNER_TIMEOUT);

  reasoner = new RemoteReasoner(generator, MAX_N,actionMapToSet(ActionFactory::actions()));
  StaticFacts::retrieveStaticFacts(reasoner, domainDirectory);

  explainer = new PlanExplainer(actionMapToSet(ActionFactory::actions()));
  ros::ServiceServer compute_and_explain = n.advertiseService("compute_and_explain_plan", computeAndExplainPlan);
  
  //need a pointer to the specific type for the observer
  ExplainableActionExecutor *replanner = new ExplainableActionExecutor(reasoner,reasoner,ActionFactory::actions());
  executor = replanner;
  
  Server server(privateNode, "execute_plan", boost::bind(&executePlan, _1, &server), false);

  ExplainPlanObserver observer(&server, explainer);
  executor->addExecutionObserver(&observer);
  replanner->addPlanningObserver(&observer);


  server.start();

  ros::spin();

  delete generator;
  delete reasoner;
  delete explainer;
  delete replanner;

  
  return 0;
}