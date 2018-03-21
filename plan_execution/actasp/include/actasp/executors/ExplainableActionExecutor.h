#ifndef actasp_ExplainableActionExecutor_h__guard
#define actasp_ExplainableActionExecutor_h__guard


#include <actasp/executors/ReplanningActionExecutor.h>

#include <stdexcept>
#include <list>
#include <map>

namespace actasp {

class AnswerSet;

class ExplainableActionExecutor : public ReplanningActionExecutor {

public:
  ExplainableActionExecutor(actasp::AspKR* reasoner, 
               actasp::Planner *planner,
               const std::map<std::string, Action * > &actionMap
              ) throw (std::invalid_argument);
  

private:
  void computePlan();

};


}
#endif
