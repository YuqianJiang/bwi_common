
#include "msgs_utils.h"
#include "MultirobotRemoteReasoner.h"
#include "StaticFacts.h"

#include "actasp/action_utils.h"
#include "actasp/executors/MultirobotActionExecutor.h"
#include "actasp/ExecutionObserver.h"
#include "actasp/PlanningObserver.h"
#include "actasp/AnswerSet.h"
#include <actasp/reasoners/Clingo4_2.h>

#include "bwi_kr_execution/ExecutePlanAction.h"
#include "bwi_kr_execution/NewPlan.h"
#include "std_msgs/String.h"

#include "multirobot_actions/ActionFactory.h"
#include "multirobot_actions/LogicalNavigation.h"

#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <boost/filesystem.hpp>

#include <string>

const int MAX_N = 20;
const int PLANNER_TIMEOUT = 20; //seconds


using namespace std;
using namespace bwi_krexec;
using namespace actasp;

typedef actionlib::SimpleActionServer<bwi_kr_execution::ExecutePlanAction> Server;


MultirobotActionExecutor *executor;
string name;

struct PrintFluent {
  
  PrintFluent(ostream& stream) : stream(stream) {}
  
  string operator()(const AspFluent& fluent) {
    stream << fluent.toString() << " ";
  }
  
  ostream &stream;
  
};

struct Observer : public ExecutionObserver, public PlanningObserver {
  
  void actionStarted(const AspFluent& action) throw() {
    ROS_INFO_STREAM(name << ": Starting execution: " << action.toString());
  }
  
  void actionTerminated(const AspFluent& action) throw() {
    ROS_INFO_STREAM(name << ": Terminating execution: " << action.toString());
  }
  
  
  void planChanged(const AnswerSet& newPlan) throw() {
   stringstream planStream;
   
   ROS_INFO_STREAM(name << ": plan size: " << newPlan.getFluents().size());
   
   copy(newPlan.getFluents().begin(),newPlan.getFluents().end(),ostream_iterator<string>(planStream," "));
   
   ROS_INFO_STREAM(planStream.str());
  }
  
    void goalChanged(std::vector<actasp::AspRule> newGoalRules) throw() {}
  
  void policyChanged(PartialPolicy* policy) throw() {}
  
  
};

void acceptNewPlan(const bwi_kr_execution::NewPlan::ConstPtr& np_msg) {
  if (executor->goalSet()) {
    ROS_INFO_STREAM(name << ": Accepting new plan!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    double time = np_msg->time.toSec();
    actasp::AnswerSet plan = TranslateAnswerSet()(np_msg->plan);
    executor->acceptNewPlan(time, plan);
  }
}

void getNewGoal(Server *as) {
  ROS_INFO("accepting new goal");
  //  create initial state
  LogicalNavigation setInitialState("noop");
  setInitialState.run();
  
  const bwi_kr_execution::ExecutePlanGoalConstPtr& goal = as->acceptNewGoal();
  vector<AspRule> goalRules;
  transform(goal->aspGoal.begin(),goal->aspGoal.end(),back_inserter(goalRules),TranslateRule());
  executor->setGoal(goalRules);

  if (as->isPreemptRequested()) {
    as->setPreempted();
    if (executor->goalReached()) 
      ROS_INFO("Preempted, but execution succeded");
    else 
      ROS_INFO("Preempted, execution aborted");
  }
}

void getPreemptRequest(Server *as) {
  as->setPreempted();
  if (executor->goalReached()) 
    ROS_INFO("Preempted, but execution succeded");
  else 
    ROS_INFO("Preempted, execution aborted");
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

  domainDirectory = ros::package::getPath("bwi_kr_execution")+"/domain_new/";

  name = n.getNamespace().substr(2);
  string queryDir = "/tmp/bwi_action_execution/"+name+"/";
  string currentFilePath = "/tmp/bwi_action_execution/"+name+"/current.asp";
  boost::filesystem::create_directories(queryDir);

  bool simulating;
  privateNode.param<bool>("simulation",simulating,false);
  ActionFactory::setSimulation(simulating); 

  FilteringQueryGenerator *generator = new Clingo4_2("n",queryDir,domainDirectory,actionMapToSet(ActionFactory::actions()),currentFilePath,PLANNER_TIMEOUT);
  AspKR *reasoner = new MultirobotRemoteReasoner(name, generator, MAX_N,actionMapToSet(ActionFactory::actions()));
  StaticFacts::retrieveStaticFacts(reasoner, domainDirectory);

  //ros::ServiceServer update_client = n.advertiseService("update_fluents",&MultirobotRemoteReasoner::updateFluents,static_cast<MultirobotRemoteReasoner*>(reasoner));
  ROS_INFO_STREAM(name << ": sensing initial state");

//  create initial state
  LogicalNavigation setInitialState("noop");
  setInitialState.run();
  
  //need a pointer to the specific type for the observer
  MultirobotActionExecutor *replanner = new MultirobotActionExecutor(reasoner,reasoner,ActionFactory::actions());
  executor = replanner;
  
  Observer observer;
  executor->addExecutionObserver(&observer);
  replanner->addPlanningObserver(&observer);

  ros::Subscriber new_plan_sub = n.subscribe("notify_new_plan", 10, acceptNewPlan);
  ros::Publisher stop_pub = n.advertise<std_msgs::String>("stop_robot", 1000);

  Server server(privateNode, "execute_plan", false);
  server.registerGoalCallback(boost::bind(&getNewGoal,&server));
  server.registerPreemptCallback(boost::bind(&getPreemptRequest,&server));
  server.start();

  ros::Rate loop(10);

  while (ros::ok()) {
    if (server.isActive()) {
      if (!executor->goalReached() && !executor->failed()) {
        executor->executeActionStep();
      }
      else if (executor->goalReached()) {
        ROS_INFO_STREAM(name << ": Execution succeded");
        if(server.isActive())
          server.setSucceeded();
      } 
      else {
        ROS_INFO_STREAM(name << ": Execution failed");
        if(server.isActive())
          server.setAborted();
      }
    }

    ros::spinOnce();
  }

  ROS_INFO_STREAM(name << ": stopped");
  std_msgs::String stop_msg;
  stop_msg.data = name;
  stop_pub.publish(stop_msg);
  
  return 0;
}
