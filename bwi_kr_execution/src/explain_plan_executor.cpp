
#include "msgs_utils.h"
#include "RemoteReasoner.h"
#include "StaticFacts.h"

#include "actasp/action_utils.h"
#include "actasp/executors/ExplainableActionExecutor.h"
#include "actasp/ExecutionObserver.h"
#include "actasp/PlanningObserver.h"
#include "actasp/AnswerSet.h"
#include <actasp/reasoners/Clingo4_2.h>

#include "bwi_kr_execution/ExecutePlanAction.h"

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


using namespace std;
using namespace bwi_krexec;
using namespace actasp;

typedef actionlib::SimpleActionServer<bwi_kr_execution::ExecutePlanAction> Server;


ActionExecutor *executor;

string fluentToText(const AspFluent& fluent, bool isTrue) {
  if (fluent.getName() == "at") 
    if (isTrue) {return "go to " + fluent.getParameters()[0];}
    else {return "leave " + fluent.getParameters()[0];}

  if (fluent.getName() == "facing") 
    if (isTrue) {return "turn to " + fluent.getParameters()[0];}
    else {return "turn away from " + fluent.getParameters()[0];}

  if (fluent.getName() == "beside") 
    if (isTrue) {return "approach " + fluent.getParameters()[0];}
    else {return "get away from " + fluent.getParameters()[0];}

  if (fluent.getName() == "open") 
    if (isTrue) {return "open " + fluent.getParameters()[0];}
    else {return "check that " + fluent.getParameters()[0] + " is closed";}

  if (fluent.getName() == "accessgranted") 
    if (isTrue) {return "check that I'm allowed to go through " + fluent.getParameters()[0];}
    else {return "know that I'm not allowed to go through " + fluent.getParameters()[0];}

  if (fluent.getName() == "found") 
    if (isTrue) {return "find " + fluent.getParameters()[0];}
    else {return "not know the location of " + fluent.getParameters()[0];}

  if (fluent.getName() == "inroom") 
    if (isTrue) {return "check that " + fluent.getParameters()[0] + " is in room " + fluent.getParameters()[1];}
    else {return "check that " + fluent.getParameters()[0] + " is not in room " + fluent.getParameters()[1];}

  if (fluent.getName() == "messagedelivered") 
    if (isTrue) {return "deliver a message to " + fluent.getParameters()[0];}
    else {return "not deliver a message to " + fluent.getParameters()[0];}

  return "undefined predicate " + fluent.getName();
}


class ExplainPlanObserver : public ExecutionObserver, public PlanningObserver {

public:
  ExplainPlanObserver(Server* server, const ActionSet& actions) : 
    server(server), isAnAction(actions) {}

  void actionStarted(const AspFluent& action) throw() {
    ROS_INFO_STREAM("Starting execution: " << action.toString());
  }
  
  void actionTerminated(const AspFluent& action) throw() {
    ROS_INFO_STREAM("Terminating execution: " << action.toString());
  }
  
  
  void planChanged(const AnswerSet& newFluents) throw() {
    stringstream planStream;

    ROS_INFO_STREAM("plan size: " << newFluents.maxTimeStep());

    copy(newFluents.getFluents().begin(),newFluents.getFluents().end(),ostream_iterator<string>(planStream," "));

    ROS_INFO_STREAM(planStream.str());

    changes.clear();


    for (int t = 1; t <= newFluents.maxTimeStep(); ++t) {
      stringstream fluentsStream;
      
      vector<pair<AspFluent, bool> > changesAtTime = getFluentDifference(newFluents.getFluentsAtTime(t-1), newFluents.getFluentsAtTime(t));
      changes.push_back(changesAtTime);

      ROS_INFO_STREAM("changes at " << t);
      transform(changesAtTime.begin(),changesAtTime.end(),ostream_iterator<string>(fluentsStream," "), boost::bind(&pair<AspFluent, bool>::first, _1));
      ROS_INFO_STREAM(fluentsStream.str());
    }

    stringstream explanations;
    stringstream feedback;
    bwi_kr_execution::ExecutePlanFeedback planFeedback;

    int count = 0;

    for (int i = 0; i < changes.size(); ++i) {
      for (vector<pair<AspFluent, bool> >::iterator it1 = changes[i].begin(); it1 != changes[i].end(); ++it1) {

        for (int j = i; j < changes.size(); ++j) {
          for (vector<pair<AspFluent, bool> >::iterator it2 = changes[j].begin(); it2 != changes[j].end(); ++it2) {
            if (it1 != it2) {
              explanations << it1->first.toString() << "=" << (it1->second ? "true" : "false") << " ";
              explanations << it2->first.toString() << "=" << (it2->second ? "true" : "false") << "\n";
              feedback << fluentToText(it1->first, it1->second) << ", ";
              feedback << fluentToText(it2->first, it2->second) << "\n";
              count++;
            }
            
          }
        }

      }
    }

    explanations << "Number of possible explanations: " << count;
    ROS_INFO_STREAM(explanations.str());

    feedback << "Number of possible explanations: " << count;
    planFeedback.plan_explanation = feedback.str();
    server->publishFeedback(planFeedback);

  }
  
  void goalChanged(std::vector<actasp::AspRule> newGoalRules) throw() {}
  
  void policyChanged(PartialPolicy* policy) throw() {}

private:
  Server* server;
  IsAnAction isAnAction;
  vector<vector<pair<AspFluent, bool> > > changes;

  vector<pair<AspFluent, bool> > getFluentDifference(const set<AspFluent>& set1, 
                                                      const set<AspFluent>& set2) {

    vector<pair<AspFluent, bool> > result;

    set<AspFluent>::const_iterator it1 = set1.begin();
    set<AspFluent>::const_iterator it2 = set2.begin();
    while (true) {
      if (it1 == set1.end()) {
        //transform(it2, set2.end(), back_inserter(result), boost::bind(make_pair<AspFluent, bool>, _1, true));
        for (; it2 != set2.end(); ++it2) {
          if (! isAnAction(*it2)) {
            result.push_back(make_pair<AspFluent, bool>(*it2, true)); 
          }
        }
        return result;
      }
      if (it2==set2.end()) {
        //transform(it1, set1.end(), back_inserter(result), boost::bind(make_pair<AspFluent, bool>, _1, true));
        for (; it1 != set1.end(); ++it1) {
          if (! isAnAction(*it1)) {
            result.push_back(make_pair<AspFluent, bool>(*it1, true)); 
          }
        }
        return result;
      }
      if (it1->toString(0) < it2->toString(0)) {
        if (! isAnAction(*it1)) {
          result.push_back(make_pair<AspFluent, bool>(*it1, false));
        }
        ++it1; 
      }
      else if (it2->toString(0) < it1->toString(0)) { 
        if (! isAnAction(*it2)) {
          result.push_back(make_pair<AspFluent, bool>(*it2, true)); 
        }
        ++it2; 
      }
      else {
        ++it1; 
        ++it2; 
      }
    }

    return result;
  }
  
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
  LogicalNavigation setInitialState("senseState");
  while (!setInitialState.hasFinished()) {
    setInitialState.run();
  }


  bool simulating;
  privateNode.param<bool>("simulation",simulating,false);
  ActionFactory::setSimulation(simulating); 
  
  boost::filesystem::create_directories(queryDirectory);

  FilteringQueryGenerator *generator = new Clingo4_2("n",queryDirectory,domainDirectory,actionMapToSet(ActionFactory::actions()),PLANNER_TIMEOUT);
  AspKR *reasoner = new RemoteReasoner(generator, MAX_N,actionMapToSet(ActionFactory::actions()));
  StaticFacts::retrieveStaticFacts(reasoner, domainDirectory);
  
  //need a pointer to the specific type for the observer
  ExplainableActionExecutor *replanner = new ExplainableActionExecutor(reasoner,reasoner,ActionFactory::actions());
  executor = replanner;
  
  Server server(privateNode, "execute_plan", boost::bind(&executePlan, _1, &server), false);

  ExplainPlanObserver observer(&server, actionMapToSet(ActionFactory::actions()));
  executor->addExecutionObserver(&observer);
  replanner->addPlanningObserver(&observer);

  server.start();

  ros::spin();

  
  return 0;
}