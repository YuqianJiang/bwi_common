
#ifndef bwi_krexec_WaitForOpen_h__guard
#define bwi_krexec_WaitForOpen_h__guard

#include "actasp/Action.h"

#include <string>

namespace bwi_krexec {

class WaitForOpen : public actasp::Action{
public:
  WaitForOpen();

  int paramNumber() const {return 1;}
  
  std::string getName() const {return "waitforopen";}
  
  void run();
  
  bool hasFinished() const {return done;}
  
  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  virtual actasp::Action *clone() const {return new WaitForOpen(*this);}
  
private:
  
 std::vector<std::string> getParameters() const;
 
 std::string door;
 bool done;
 
};

}
 
#endif
 