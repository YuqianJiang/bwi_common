#ifndef actasp_ActionWithTime_h__guard
#define actasp_ActionWithTime_h__guard

#include <actasp/Action.h>
#include <sstream>

class ActionWithTime : public actasp::Action { 

public:
  ActionWithTime() : done(false) {}
  ActionWithTime(std::string name, int param_number) : 
    name(name), param_number(param_number), done(false) {}
  ActionWithTime(std::string name, std::vector<std::string> params, int timeStep, int time) :
    name(name), params(params), param_number(params.size()), timeStep(timeStep), time(time), done(false) {}
  ActionWithTime(actasp::AspFluent fluent) :
    name(fluent.getName()), params(fluent.getParameters()), param_number(fluent.getParameters().size()), done(false) {}
  int paramNumber() const {return param_number;}
  std::vector<std::string> getParameters() const {return params;}
  std::string getName() const {return name;}
  void run() {
    done = true;
  }
  bool hasFinished() const {return done;}

  bool operator!=(const ActionWithTime& action) {
    if (this->name != action.name) return true;
    if (this->param_number != action.paramNumber()) return true;
    for(int i = 0; i < this->param_number; ++i) {
      if (this->params[i] != action.getParameters()[i]) return true;
    }
    return false;
  }

  std::string toExternalAction(const std::string robot, const unsigned int timeStep) const {
    std::stringstream externalAction;
    externalAction << "ex";
    externalAction << this->name << "(" << robot << ",";

    for(int i=0, size=this->getParameters().size(); i<size ; ++i)
      externalAction << this->getParameters()[i] << ",";
    externalAction << timeStep <<")";
    
    return externalAction.str();
  }

  virtual Action *cloneAndInit(const actasp::AspFluent & fluent) const {
    return this->cloneAndInit(fluent,0);
  }

  virtual actasp::Action *cloneAndInit(const actasp::AspFluent &f, const int time = 0) const {
    ActionWithTime *newAction = new ActionWithTime(*this);
    newAction->name = f.getName();
    newAction->params = f.getParameters();
    newAction->time = time;
    return newAction;
  }
  
  virtual actasp::Action *clone() const {
    return new ActionWithTime(*this);
  }

  int getTimeStep() const {return timeStep;}

  int getTimeParameter() const {return time;}
  void setTimeParameter(const int t) {time = t;}
  
private:
  std::string name;
  std::vector<std::string> params;
  int param_number;
  int timeStep;
  int time;
  bool done;
};

#endif
