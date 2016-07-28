#include "GoTo.h"

#include "ActionFactory.h"

#include "actasp/AnswerSet.h"

#include "bwi_kr_execution/CurrentStateQuery.h"

#include <ros/ros.h>

#include <algorithm>




using namespace std;
using namespace actasp;

namespace bwi_krexec {
  
  static vector<string> createVector(const std::string& to) {
    vector<string> paramVector(1);
    if (to.length()>0) 
      paramVector[0] = "o"+to.substr(1);
    else
      paramVector[0] = to;
    
    return paramVector;
  }
    
  
GoTo::GoTo(const std::string& from, const std::string& to): 
              LogicalNavigation("goto",createVector(to)),
              failed(false),
              from(from),
              to(to){}

vector<string> GoTo::getParameters() const {
  vector<string> paramVector;
  paramVector.push_back(from);
  paramVector.push_back(to);
  return paramVector;
}
 
 
 struct IsFluentAt {
   
   bool operator()(const bwi_kr_execution::AspFluent& fluent) {
     return fluent.name == "at";
   }
   
 };
 
void GoTo::run()  {
  
  ros::NodeHandle n;
  ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::CurrentStateQuery> ( "current_state_query" );
  krClient.waitForExistence();
  
  bwi_kr_execution::CurrentStateQuery csq;
  
  krClient.call(csq);
  
  vector<bwi_kr_execution::AspFluent>::const_iterator atIt = 
                    find_if(csq.response.answer.fluents.begin(), csq.response.answer.fluents.end(), IsFluentAt());
  
   bool error = false;
                    
  if(atIt == csq.response.answer.fluents.end()) {
    ROS_ERROR("Goto: fluent \"at\" missing ");
    error = true;
  }
  
  bwi_krexec::LogicalNavigation::run();
  
   krClient.call(csq);
  
  atIt = find_if(csq.response.answer.fluents.begin(), csq.response.answer.fluents.end(), IsFluentAt());
                    
  if(!error && atIt != csq.response.answer.fluents.end()) {
    string goalPosition = "l" + (this->getParameters()[0]).substr(1);
    failed = goalPosition != atIt->variables[0];
  }
  else {
    failed = true;
  }
  
  
}

static ActionFactory gotoFactory(new GoTo("",""));
  
  
}
