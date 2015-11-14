#ifndef actasp_GraphPolicy_h__guard
#define actasp_GraphPolicy_h__guard

#include <actasp/PartialPolicy.h>

#include <actasp/AspFluent.h>
#include <actasp/state_utils.h>

#include <map>
#include <set>

namespace actasp {
  
struct GraphPolicy : public PartialPolicy {
public:
  
  GraphPolicy(const ActionSet& actions);
  
  ActionSet actions(const std::set<AspFluent>& state) const throw();
  
  void merge(const AnswerSet& plan) throw(std::logic_error);
  
  void merge(const PartialPolicy* otherPolicy);
  
  void merge(const GraphPolicy* otherPolicy);
  
  bool empty() const throw();
  
  std::vector<actasp::AnswerSet> plansFrom(const std::set<AspFluent>& state) throw();
  
  typedef std::map<AspFluent, std::set<AspFluent>, ActionComparator > ActionStateMap;
  typedef std::map<std::set<AspFluent>, ActionStateMap, StateComparator<AspFluent> > GraphMap;
  
private:
  //map( initial state , map (action = aspfluent, final state) )
  //if the final state is the goal, the final state set is empty.
  GraphMap policyWithFinalState;
   unsigned int maxPlanLength;//used to avoid longer plans from policy 
  ActionSet allActions;
  
  void plansFromRec(const std::set<AspFluent>& state, std::vector<std::vector<AspFluent> >& result, std::vector<std::set<AspFluent> > visited) const throw();
  
};
  
}

#endif
