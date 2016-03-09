#ifndef bwi_actexec_GoTo_h__guard
#define bwi_actexec_GoTo_h__guard

#include "LogicalNavigation.h"

//#include <string>

namespace bwi_krexec {

  
class GoTo : public LogicalNavigation {
public:  
  explicit GoTo(const std::string& from, const std::string& to);
  
  void run();
  
  bool hasFailed() const {return failed;}
  
  Action *cloneAndInit(const actasp::AspFluent & fluent) const {
    return new GoTo(fluent.getParameters().at(0), fluent.getParameters().at(1));
  }
  
  virtual Action *clone() const {return new GoTo(*this);}
    
private:
  bool failed;
};  
}

#endif
