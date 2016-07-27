
#ifndef bwi_krexec_Wait_h__guard
#define bwi_krexec_Wait_h__guard

#include "actasp/Action.h"

#include <string>

namespace bwi_krexec {

class Wait : public actasp::Action{
public:
  Wait();

  int paramNumber() const {return 1;}
  
  std::string getName() const {return "wait";}
  
  void run();
  
  bool hasFinished() const {return done;}
  
  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  virtual actasp::Action *clone() const {return new Wait(*this);}
  
private:
  
 std::vector<std::string> getParameters() const;
 
 int time;
 bool done;
 
};

}
 
#endif
 