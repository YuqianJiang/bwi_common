

#ifndef actasp_state_util_h__guard
#define actasp_state_util_h__guard

#include <actasp/AspFluent.h>

#include <functional>
#include <set>
#include <string>

namespace actasp {

template<typename FluentClass> //typically AspFluent, possibly AspFluentRef
struct StateComparator : public std::binary_function< std::set<FluentClass>, std::set<FluentClass>, bool> {
    
  bool operator()(const std::set<FluentClass> &first,const std::set<FluentClass> &second) const {


    if (first.size() != second.size())
      return first.size() < second.size();

    //they have the same number of fluents
    typename std::set<FluentClass>::const_iterator thisV = first.begin();
    typename std::set<FluentClass>::const_iterator otherV = second.begin();

    ActionComparator lessThen; //ignores the time steps!

    for (; thisV != first.end(); ++thisV, ++otherV) {
      //this comparison is costly, so I'm using this unelegant expression to minimize the calls to it.
      if (lessThen(*thisV , *otherV))
        return true;
      if (lessThen(*otherV , *thisV))
        return false;
    }

    //they are the same
    return false;

  }
    
};





//assumes fluents are in the same order. 
//equals if everything else than timestep is equal.
struct stateEquals {

  stateEquals(std::set<AspFluent> state) : state(state) {}

  bool operator()(const std::set<AspFluent> otherstate) const {

    if (state.size() != otherstate.size()) {
      return false;
    }
    std::set<AspFluent>::const_iterator thisIt = state.begin();
    std::set<AspFluent>::const_iterator otherIt = otherstate.begin();
    for(; thisIt!=state.end(); ++thisIt) {
      std::string thisstring = thisIt->toString(0);
      std::string otherstring = otherIt->toString(0);
      if (thisstring.compare(otherstring)!=0) { //different
        return false;
      }
      ++otherIt;
    }
    return true; //at this point..
  }

  std::set<AspFluent> state;
};  


}

#endif
