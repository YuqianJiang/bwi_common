
#ifndef bwi_krexec_WaitForOpen_h__guard
#define bwi_krexec_WaitForOpen_h__guard

#include "actasp/Action.h"

#include <string>

#include "std_msgs/String.h"
#include <ros/ros.h>

namespace bwi_krexec {

class WaitForOpen : public actasp::Action{
public:
  WaitForOpen();

  int paramNumber() const {return 1;}
  
  std::string getName() const {return "waitforopen";}
  
  void openCallback(const std_msgs::String::ConstPtr& msg);

  void run();
  
  bool hasFinished() const {return done;}

  bool hasFailed() const {return failed;}
  
  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  virtual actasp::Action *clone() const {return new WaitForOpen(*this);}
  
private:
  
 std::vector<std::string> getParameters() const;
 
 std::string door;
 bool open;
 bool done;
 bool failed;
 static ros::Subscriber open_listener;
 static bool subscriber_set;
 
};

}
 
#endif
 