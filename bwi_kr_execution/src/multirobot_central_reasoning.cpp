
#include "actasp/AnswerSet.h"
#include "actasp/AspRule.h"
#include "actasp/action_utils.h"
#include "actasp/reasoners/Reasoner.h"
#include "actasp/reasoners/Clingo4_2.h"
#include "ActionWithTime.h"
#include "msgs_utils.h"
#include "bwi_kr_execution/MultirobotComputePlan.h"
#include "bwi_kr_execution/MultirobotUpdateFluents.h"
#include "bwi_kr_execution/NewPlan.h"
#include "bwi_msgs/AvailableRobotArray.h"
#include "std_msgs/String.h"

#include <ros/ros.h>
#include <iostream>
#include <fstream> 
#include <vector>
#include <sstream>
#include <algorithm>
#include <iterator>

#include <boost/filesystem.hpp>
#include <ros/package.h>
#include <unistd.h>

using namespace std;
using namespace actasp;
using namespace bwi_krexec;

string common_directory = ros::package::getPath("bwi_kr_execution")+"/domain_new/";

struct Robot {
  Robot(const string s, const ros::ServiceServer plan_server, const ros::ServiceServer state_server, const ros::Subscriber stop_subscriber,
          const ros::Publisher plan_publisher, const string externalFilePath, Reasoner* const reasoner) : 
                                name(s), plan_server(plan_server), state_server(state_server), 
                                stop_subscriber(stop_subscriber), plan_publisher(plan_publisher),
                                externalFilePath(externalFilePath), externalFluents(), reasoner(reasoner) {}
  string name;
  ros::ServiceServer plan_server;
  ros::ServiceServer state_server;
  ros::Subscriber stop_subscriber;
  ros::Publisher plan_publisher;
  AnswerSet plan;
  string externalFilePath;
  vector<AspRule> goal;
  stringstream externalFluents;
  actasp::Reasoner *reasoner;

  ~Robot() {
    delete reasoner;
  }
};


map<string, Robot*> robots;
std::map<std::string, actasp::Action *> actionMap;
const int MAX_N = 15;
const double SUBOPTIMALITY = 1.1;
const double alpha = 1;
const double rho = 10;
const double collision_penalty = 100;
bool printFluent = false;
bool printPlans = true;
bool considerAll = true;
bool coordination;

vector<ActionWithTime> readBestPlan(const AnswerSet &set);
void startRobot(const string name);

AnswerSet planHelper(const map<string,Robot*>::iterator it) {
  AnswerSet answer = it->second->reasoner->computeOptimalPlan(it->second->goal, false, SUBOPTIMALITY, true);
  
  if (!answer.isSatisfied()) {
    ROS_INFO("Plan query is not satisfiable");
    return answer;
  }

  vector<ActionWithTime> plan = readBestPlan(answer);

  it->second->externalFluents.str("");

  for(int t = 1; t <= answer.maxTimeStep(); ++t) {

    set<AspFluent> fluents = answer.getFluentsAtTime(t);

    if (plan[t-1].getName() == "opendoor") {
      it->second->externalFluents << plan[t-1].toExternalAction(it->first,t) << "." << endl;
      std::set<actasp::AspFluent>::const_iterator flu = fluents.begin();
      for(; flu != fluents.end(); ++flu)
      {
        if ((flu->getName() == "cumucost") || (flu->getName() == "cumuparam")) {
          stringstream externalFluent;
          externalFluent << "ex" << flu->getName() << "(" << it->first << "," << flu->getParameters()[0] << "," << flu->getTimeStep() << ")";
          it->second->externalFluents << externalFluent.str() << "." << endl;
        }
      }
    }

    if ((t < answer.maxTimeStep()) && (plan[t].getName() == "goto")) {
      it->second->externalFluents << plan[t].toExternalAction(it->first,t+1) << "." << endl;
      std::set<actasp::AspFluent>::const_iterator flu = fluents.begin();
      for(; flu != fluents.end(); ++flu)
      {
        if ((flu->getName() == "cumucost") || (flu->getName() == "cumuparam")) {
          stringstream externalFluent;
          externalFluent << "ex" << flu->getName() << "(" << it->first << "," << flu->getParameters()[0] << "," << flu->getTimeStep() << ")";
          it->second->externalFluents << externalFluent.str() << "." << endl;
        }
      }
    }
  }

  if (plan[0].getName() == "goto") {
    it->second->externalFluents << plan[0].toExternalAction(it->first,1) << "." << endl;
    it->second->externalFluents << "excumucost(" << it->first << ",0,0)." << endl << "excumuparam(" << it->first << ",0,0)." << endl;
  }

  return answer;
}

void writeExternalFluents(const map<string,Robot*>::iterator current, const map<string,Robot*>::iterator previous, const double care)
{
  ofstream externalFile(current->second->externalFilePath.c_str());
  externalFile << "#program base." << endl;
  externalFile << "#const mu = " << care*collision_penalty << "." << endl;
  externalFile << "#const rho = " << (1-care)*rho << "." << endl;
  
  if (considerAll) {
    map<string,Robot*>::iterator it = robots.begin();
    for (; it != robots.end(); ++it) {
      if (it != current) {
        externalFile << it->second->externalFluents.str();
      }
    }
  }
  else {
    ROS_INFO_STREAM("considering " << previous->first);
    externalFile << previous->second->externalFluents.str();
  }

  externalFile << "#program cumulative(n)." << endl;
  externalFile << ":~ cost(X,Y). [X@1,Y]" << endl << ":~ collisioncost(X,Y). [X@1,Y]" << endl << ":~ param(X,Y). [X/2@1,Y]" << endl;

  //put progress here
  /*
  for (vector<string>::iterator it = progress.variables.begin(); it != progress.variables.end();) {
    if (it == progress.variables.begin())
      externalFile << "(";
    externalFile << (*it);
    ++it;
    if (it != progress.variables.end())
      externalFile << "," ;
    else
      externalFile << ")." << endl;
  }

  if (progress.name == "nowgotocost") {
    externalFile << "nowgotocost(";
    externalFile << progress.variables[0] << "," << progress.variables[0] << ",";
    externalFile << "X-" << progress.variables[2] << ") :- ";
    externalFile << "dist(" << progress.variables[0] << "," << progress.variables[1] << ",X).";
  }
  */
  externalFile.close();
}

void copy_domain(const string domainDirectory)
{
  symlink((common_directory+"all_actions.asp").c_str(), (domainDirectory+"all_actions.asp").c_str());
  symlink((common_directory+"navigation.asp").c_str(), (domainDirectory+"navigation.asp").c_str());
  symlink((common_directory+"navigation_facts.asp").c_str(), (domainDirectory+"navigation_facts.asp").c_str());
  symlink((common_directory+"costs.asp").c_str(), (domainDirectory+"costs.asp").c_str());
  symlink((common_directory+"lua.asp").c_str(), (domainDirectory+"lua.asp").c_str());
}

bool plan(bwi_kr_execution::MultirobotComputePlan::Request &req,
          bwi_kr_execution::MultirobotComputePlan::Response &res)
{
  ROS_INFO("Plan query received");
  string name = req.name;
  map<string,Robot*>::iterator it = robots.find(name);
  if (it == robots.end()) {
    ROS_INFO_STREAM("Failed to find robot " << name << " in the planning queue");
    startRobot(name);
  }
  it->second->goal.erase(it->second->goal.begin(), it->second->goal.end());
  transform(req.goal.begin(),req.goal.end(),back_inserter(it->second->goal),TranslateRule());
  if (it->second->goal.size() == 0) {
    res.plan = bwi_kr_execution::AnswerSet();
    return true;
  }

  ofstream externalFile(it->second->externalFilePath.c_str());
  externalFile << "#program cumulative(n)." << endl;
  externalFile << ":~ cost(X,Y). [X@1,Y]" << endl << ":~ collisioncost(X,Y). [X@1,Y]" << endl << ":~ param(X,Y). [X/2@1,Y]" << endl;
  externalFile.close();
  it->second->plan = planHelper(it);

  if (!it->second->plan.isSatisfied()) {
    TranslateAnswerSet translator;
    res.plan = translator(it->second->plan);
    return true;
  }

  if (coordination && (robots.size() > 1)) {
    double care = 0.0;
    while (care < 1) {
      care += alpha;
      map<string,Robot*>::iterator rIt = robots.begin();
      map<string,Robot*>::iterator pIt = robots.end();
      pIt--;

      for (; rIt != robots.end(); ++rIt) {
        if ((!rIt->second->goal.empty()) && (rIt != pIt)) {
          writeExternalFluents(rIt,pIt,care);
          if (printPlans) ROS_INFO_STREAM("planning " << rIt->first);
          rIt->second->plan = planHelper(rIt);
          ofstream externalFile(rIt->second->externalFilePath.c_str());
          externalFile.close();
        }
        pIt = rIt;
      }
    }

    TranslateAnswerSet translator;
    res.plan = translator(it->second->plan);

    //somehow notify other robots of new plan
    for (map<string,Robot*>::iterator rIt = robots.begin(); rIt != robots.end(); ++rIt) {
      if ((rIt != it) && (!rIt->second->goal.empty())) {
        bwi_kr_execution::NewPlan np_msg;
        np_msg.time = ros::Time::now();
        np_msg.plan = translator(rIt->second->plan);
        rIt->second->plan_publisher.publish(np_msg);
      }
    }
  }
  else {
    ofstream externalFile(it->second->externalFilePath.c_str());
    externalFile.close();
    TranslateAnswerSet translator;
    res.plan = translator(it->second->plan);
  }
  

  return true;
}

bool update(bwi_kr_execution::MultirobotUpdateFluents::Request &req,
            bwi_kr_execution::MultirobotUpdateFluents::Response &res)
{
  string name = req.name;
  map<string,Robot*>::iterator it = robots.find(name);
  if (it == robots.end()) {
    ROS_INFO_STREAM("Failed to find robot " << name << " in the planning queue");
    startRobot(name);
  }

  ROS_INFO("update state request received");
  vector<AspFluent> new_fluents;
  transform(req.fluents.begin(),req.fluents.end(),back_inserter(new_fluents),TranslateFluent());
  res.consistent = it->second->reasoner->updateFluents(new_fluents);

  vector<AspRule> query;
  AnswerSet current = it->second->reasoner->currentStateQuery(query);
  transform(current.getFluents().begin(),current.getFluents().end(),back_inserter(res.state),TranslateFluent());

  if ((!it->second->goal.empty()) && (it->second->reasoner->currentStateQuery(it->second->goal).isSatisfied())) {
    it->second->goal.erase(it->second->goal.begin(), it->second->goal.end());
    ROS_INFO_STREAM(it->first << ": reached goal");
  } 
  return true;
}

void stopRobot(const std_msgs::String::ConstPtr& msg)
{
  ros::NodeHandle n;
  string name = msg->data;
  map<string,Robot*>::iterator it = robots.find(name);
  if (it != robots.end()) {
    string tmpDirectory = "/tmp/bwi_multirobot_planning/" + name +"/";
    boost::filesystem::remove_all(tmpDirectory);
    robots[name]->plan_server.shutdown();
    robots[name]->state_server.shutdown();
    delete robots[name]; 
    robots.erase(it);
  }
}

void startRobot(const string name)
{
  map<string,Robot*>::iterator it = robots.find(name);
  if (it == robots.end()) {  
    ros::NodeHandle n;
    string plan_srv_name = "/" + name + "/multirobot_compute_plan";
    string state_srv_name = "/" + name + "/multirobot_update_fluents";
    string stop_msg_name = "/" + name + "/stop_robot";
    string notify_msg_name = "/" + name + "/notify_new_plan";
    string queryDirectory = "/tmp/bwi_multirobot_planning/" + name + "/query/";
    string domainDirectory = "/tmp/bwi_multirobot_planning/" + name + "/domain/";
    boost::filesystem::create_directories(queryDirectory);
    boost::filesystem::create_directories(domainDirectory);
    copy_domain(domainDirectory);
    //remove previous state
    string currentFile = "/tmp/bwi_multirobot_planning/" + name + "/current.asp";
    remove(currentFile.c_str());
    string externalFilePath = domainDirectory + "external.asp";
    ofstream externalFile(externalFilePath.c_str());
    externalFile.close();
    FilteringQueryGenerator *generator = new Clingo4_2("n",queryDirectory,domainDirectory,actionMapToSet(actionMap),currentFile);
    Reasoner* reasoner = new Reasoner(generator,MAX_N,actionMapToSet(actionMap));
    ROS_INFO_STREAM("starting robot " + name);
    Robot* r =  new Robot(name, n.advertiseService(plan_srv_name, plan), n.advertiseService(state_srv_name, update), 
                                n.subscribe(stop_msg_name, 10, stopRobot), n.advertise<bwi_kr_execution::NewPlan>(notify_msg_name, 10), externalFilePath, reasoner);
    robots[name] = r;
  }
}

void startAllRobots(const bwi_msgs::AvailableRobotArray::ConstPtr& allRobots)
{
  for (vector<bwi_msgs::AvailableRobot>::const_iterator it = allRobots->robots.begin(); it != allRobots->robots.end(); ++it) {
    if (it->type == 1) {
      startRobot(it->name);
    }
  }
}

struct DeleteAction {
  
  void operator()(Action *act) {
    delete act;
  }
};

vector<ActionWithTime> readBestPlan(const AnswerSet &set) {
  stringstream answers;

  answers <<  "------------------------------";

  list<Action*> plan = set.instantiateActions(actionMap);
  list<Action*>::iterator pIt = plan.begin();
  vector<ActionWithTime> planWithTime;

  for(int t=1; pIt != plan.end(); ++pIt, ++t) {
      ActionWithTime action((*pIt)->toFluent(t).getName(), (*pIt)->toFluent(t).getParameters(), t, 0);
      planWithTime.push_back(action);
  }

  if (printFluent) answers << endl << endl << "Fluents" << endl;
  
  //fluents in an answer set are ordered by time step.

  unsigned int lastTimeStep = set.maxTimeStep();

  for(int t = 0; t <= lastTimeStep; ++t) {
    
    std::set<actasp::AspFluent> fluentsAtTimeT = set.getFluentsAtTime(t);
    
    std::set<actasp::AspFluent>::const_iterator flu = fluentsAtTimeT.begin();
    for(; flu != fluentsAtTimeT.end(); ++flu)
    {
      if ((flu->getName() == "cost") && (flu->getTimeStep() > 0)) {
        planWithTime[flu->getTimeStep()-1].setTimeParameter(atoi((flu->getParameters()[0]).c_str()));
      }
      if (printFluent) answers << flu->toString() << " ";
    }  
    if (printFluent) answers << endl;
  }

  answers << endl << "Plan" << endl;

  vector<ActionWithTime>::iterator aIt = planWithTime.begin();
  for (int t=1; aIt != planWithTime.end(); ++aIt, ++t)
    answers << "action: " << aIt->toASP(t) << " time: " << aIt->getTimeParameter() << endl;

  for_each(plan.begin(),plan.end(),DeleteAction());
  
  answers << endl;
  if (printPlans) ROS_INFO_STREAM(answers.str());

  return planWithTime;

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "multirobot_knowledge_reasoning");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  nh.param<bool>("coordination",coordination,true);

  ros::Subscriber start_listener = n.subscribe("available_robots",1,startAllRobots);

  actionMap.insert(std::make_pair(std::string("approach"), new ActionWithTime("approach",1)));
  actionMap.insert(std::make_pair(std::string("gothrough"), new ActionWithTime("gothrough",1)));
  actionMap.insert(std::make_pair(std::string("opendoor"), new ActionWithTime("opendoor",1)));
  actionMap.insert(std::make_pair(std::string("goto"), new ActionWithTime("goto",2)));
  actionMap.insert(std::make_pair(std::string("waitforopen"), new ActionWithTime("waitforopen",1)));
  actionMap.insert(std::make_pair(std::string("wait"), new ActionWithTime("wait",1)));
  ros::spin();
  return 0;
}
