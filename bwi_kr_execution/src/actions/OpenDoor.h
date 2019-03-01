
#ifndef bwi_krexec_OpenDoor_h__guard
#define bwi_krexec_OpenDoor_h__guard

#include "actasp/Action.h"
#include <bwi_msgs/CheckBool.h>
#include "../BwiResourceManager.h"
#include <knowledge_representation/LTMCEntity.h>

#include <string>
#include <ros/ros.h>

namespace bwi_krexec {

class OpenDoor : public actasp::Action{
public:
  OpenDoor(const int door_id, knowledge_rep::LongTermMemoryConduit &ltmc);

  int paramNumber() const override {return 1;}

  std::string getName() const override {return "open_door";}
  
  void run();
  
  bool hasFinished() const {return done;}
  
  virtual bool hasFailed() const {return failed;}
  
  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  virtual actasp::Action *clone() const {return nullptr;}

  static std::unique_ptr<actasp::Action> create(const actasp::AspFluent & fluent, actasp::ResourceManager &resource_manager) {
    auto door_id = std::atoi(fluent.getParameters().at(0).c_str());
    auto& resource_manager_cast = dynamic_cast<BwiResourceManager&>(resource_manager);
    return std::unique_ptr<actasp::Action>(new bwi_krexec::OpenDoor(door_id, resource_manager_cast.ltmc));
  }


private:
  
 std::vector<std::string> getParameters() const;
 bool checkDoorOpen();
 
 int door_id;
 knowledge_rep::Entity door_entity;
 std::string door_name;
 bool done;
 bool failed;
 bool requestSent;
 ros::Time startTime;

 ros::ServiceClient doorStateClient;

 knowledge_rep::LongTermMemoryConduit &ltmc;
 
};

}
 
#endif
