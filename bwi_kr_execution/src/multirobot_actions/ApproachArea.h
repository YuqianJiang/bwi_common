#ifndef bwi_actexec_ApproachArea_h__guard
#define bwi_actexec_ApproachArea_h__guard

#include "LogicalNavigation.h"

//#include <string>

namespace bwi_krexec {

  
class ApproachArea : public LogicalNavigation {
public:  
  explicit ApproachArea(const std::string& from, const std::string& to);
  
  void run();
  
  bool hasFailed() const {return failed;}
  
  Action *cloneAndInit(const actasp::AspFluent & fluent) const {
    return new ApproachArea(fluent.getParameters().at(0), fluent.getParameters().at(1));
  }
  
  virtual Action *clone() const {return new ApproachArea(*this);}
    
private:
  bool failed;
};  
}

#endif
