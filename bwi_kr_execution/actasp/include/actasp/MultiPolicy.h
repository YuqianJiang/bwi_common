#ifndef actasp_MultiPolicy_h__guard
#define actasp_MultiPolicy_h__guard

#include <actasp/AnswerSet.h>
#include <actasp/AspFluent.h>
#include <actasp/state_utils.h>

#include <set>
#include <map>
#include <stdexcept>

namespace actasp {
  
class MultiPolicy {
public:
  
  MultiPolicy(const ActionSet& actions);
  MultiPolicy();
  
	ActionSet actions(const std::set<AspFluent>& state) const throw();
	
	void merge(const AnswerSet& plan, bool finalState = false) throw(std::logic_error);
  void merge(const MultiPolicy& otherPolicy, bool finalState = false);
	
	bool empty()const throw();
	bool finalStateEmpty() const throw();

	std::vector<actasp::AnswerSet> plansFrom(std::set<AspFluent>& state) throw();
  std::set<AspFluent> nextExpected(std::set<AspFluent> initial, AspFluent action); 

private:
  std::map<std::set<AspFluent>, ActionSet, StateComparator<AspFluent> > policy;
  ActionSet allActions;

  //map( initial state , map (action = aspfluent, final state) )
  //if the final state is the goal, the final state set is empty.
  std::map<std::set<AspFluent>, std::map<AspFluent, std::set<AspFluent> >, StateComparator<AspFluent> > policyWithFinalState;
  unsigned int maxPlanLength; //used to avoid longer plans from policy 
  void plansFromRec(std::set<AspFluent>& state, std::vector<std::vector<AspFluent> >& result, std::vector<std::set<AspFluent> > visited) const throw();
};
	
}

#endif
