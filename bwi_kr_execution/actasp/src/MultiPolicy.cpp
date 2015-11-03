#include <actasp/MultiPolicy.h>

#include <actasp/Action.h>
#include <actasp/action_utils.h>

#include <algorithm>

#include <functional> // ?

//#include <time.h>


using namespace std;

namespace actasp {
  
MultiPolicy::MultiPolicy(const ActionSet& actions) : policy(), policyWithFinalState(), allActions(actions), maxPlanLength(0) {}

MultiPolicy::MultiPolicy() {}
	
ActionSet MultiPolicy::actions(const std::set<AspFluent>& state) const throw() {
  
	std::map<set<AspFluent>, ActionSet >::const_iterator acts = policy.find(state);

	if(acts != policy.end()) {
		return acts->second;
	}
	
	return ActionSet();
}

void MultiPolicy::merge(const AnswerSet& plan, bool finalState) throw(logic_error) {

  //in clingo 4, actions are from timestep 1 to last time step
  unsigned int planLength = plan.maxTimeStep();

  if (planLength > maxPlanLength) {
    maxPlanLength = planLength; //update longest
  }

  set<AspFluent> fluents = plan.getFluentsAtTime(0); //action for state 0 is at timestep 1

	for (int timeStep = 1; timeStep <= planLength; ++timeStep) {
		
		set<AspFluent> fluentsWithAction = plan.getFluentsAtTime(timeStep);
    
    //find the action
    set<AspFluent>::iterator actionIt = find_if(fluentsWithAction.begin(),fluentsWithAction.end(),IsAnAction(allActions));
    
    if(actionIt == fluentsWithAction.end())
      throw logic_error("MultiPolicy: no action for some state");
    
    AspFluent action = *actionIt;
		
		//remove the action from there
		fluentsWithAction.erase(actionIt);
		
		ActionSet &stateActions = policy[fluents]; //creates an empty vector if not present

		stateActions.insert(action);


    if (finalState) {

      set<AspFluent> final;
      if (timeStep==planLength) { //goal state
        final = set<AspFluent>();
      }
      else { //not goal state
        final = fluentsWithAction; //currently last state
      }
      action.setTimeStep(0);
      std::map<AspFluent, std::set<AspFluent> > &actstate = policyWithFinalState[fluents];
      actstate.insert(pair<AspFluent, std::set<AspFluent> >(action, final));
    }

    fluents = fluentsWithAction;

	}
}

struct MergeActions {
  MergeActions( std::map<std::set<AspFluent>, ActionSet, StateComparator<AspFluent> > &policy) : policy(policy) {}
 
 void operator()(const std::pair<set<AspFluent>, ActionSet >& stateActions) {
  
   map<set<AspFluent>, ActionSet >::iterator found = policy.find(stateActions.first);
   if(found == policy.end())
     policy.insert(stateActions);
   
   else {
     found->second.insert(stateActions.second.begin(),stateActions.second.end());
   }
      
  }
  std::map<std::set<AspFluent>, ActionSet, StateComparator<AspFluent> > &policy;
};

struct MergeActionsFinal { //merges policy with final states
  MergeActionsFinal( std::map<std::set<AspFluent>, std::map<AspFluent, std::set<AspFluent> >, StateComparator<AspFluent> > &policyWithFinalState) : policyWithFinalState(policyWithFinalState) {}
 
 void operator()(const std::pair<std::set<AspFluent>, std::map<AspFluent, std::set<AspFluent> > >& stateActionsState) {
  
   map<set<AspFluent>, map<AspFluent, set<AspFluent> > >::iterator found = policyWithFinalState.find(stateActionsState.first);

   if(found == policyWithFinalState.end())
     policyWithFinalState.insert(stateActionsState);
   
   else {
    //check if action is already there. if it is, update final state, if it isnt add it. 
     found->second.insert(stateActionsState.second.begin(),stateActionsState.second.end());
   }
      
 }
  std::map<std::set<AspFluent>, std::map<AspFluent, std::set<AspFluent> >, StateComparator<AspFluent> > &policyWithFinalState;
};


void MultiPolicy::merge(const MultiPolicy& otherPolicy, bool finalState) {
  
  set_union(otherPolicy.allActions.begin(),otherPolicy.allActions.end(),
                 allActions.begin(),allActions.end(),
                 inserter(allActions,allActions.begin()));
  
  for_each(otherPolicy.policy.begin(),otherPolicy.policy.end(),MergeActions(policy));

  if (finalState) {
    for_each(otherPolicy.policyWithFinalState.begin(),otherPolicy.policyWithFinalState.end(),MergeActionsFinal(policyWithFinalState));
  }

  if (otherPolicy.maxPlanLength > maxPlanLength) {
    maxPlanLength = otherPolicy.maxPlanLength;
  }
}



bool MultiPolicy::empty() const throw() {
	return policy.empty();
}

bool MultiPolicy::finalStateEmpty() const throw() {
  return policyWithFinalState.empty();
}


//helper
void MultiPolicy::plansFromRec(set<AspFluent>& state, vector<vector<AspFluent> >& result, std::vector< set<AspFluent> > visited) const throw() {

  visited.push_back(state); //new state (function does not get called on states that have been visited already)

  std::map<std::set<AspFluent>, std::map<AspFluent, std::set<AspFluent> > >::const_iterator acts = policyWithFinalState.find(state);
  if (acts == policyWithFinalState.end()) {
    result.pop_back(); //discard last plan
    return;
  }
  std::map<AspFluent, std::set<AspFluent> > options = acts->second;
  std::map<AspFluent, std::set<AspFluent> >::iterator option = options.begin();
  bool samePlan = true; //the first child is in the same plan, but every other has to start a new plan

  vector<AspFluent> commonResult = result.back();
  vector<set<AspFluent> > commonVisited = visited;

  //for every possible action in the state
  for (; option != options.end(); ++option) { 

    if (!samePlan) { //get same starting point as siblings
      result.push_back(commonResult); //same starting result
      visited = commonVisited; //visited by siblings do not count, restart from parent's 
    }

    set<AspFluent> nextState = option->second;
    AspFluent nextAction = option->first;

    //goal: save and stop
    if (nextState.empty()) {  
      result.back().push_back(nextAction); //add action to the plan
    }

    //visited already: loop! discard
    else if (std::find_if(visited.begin(), visited.end(), stateEquals(nextState) ) != visited.end()) { 
      result.pop_back(); //discard last plan
    }

    //plan is getting too long, discard
    else if (result.back().size() >= maxPlanLength) {
      result.pop_back();
    }

    //intermediate step: save and continue
    else { 
      result.back().push_back(nextAction); //add action to the plan
      plansFromRec(nextState, result, visited);
    }

    samePlan = false; //only first child stays on parent's plan
  }

}

//gets plans from a given state using policy with final state
std::vector<actasp::AnswerSet> MultiPolicy::plansFrom(set<AspFluent>& state) throw() {

  //the state might not be in the policy
  std::map<std::set<AspFluent>, std::map<AspFluent, std::set<AspFluent> > >::const_iterator acts = policyWithFinalState.find(state);
  if (acts == policyWithFinalState.end()) {
    std::vector<actasp::AnswerSet> nothing;
    return nothing;
  }

  vector<vector<AspFluent> > intermediateResult;
  vector<AspFluent> first; 
  intermediateResult.push_back(first); //the first plan is initialized
  std::vector<set<AspFluent> > visited;
  plansFromRec(state, intermediateResult, visited);

  //get final result
  std::vector<actasp::AnswerSet> results;
  vector<vector<AspFluent> >::iterator result = intermediateResult.begin();
  for (; result != intermediateResult.end(); ++result) {  //for every result in intermediate
    int timestep = 1;
    vector<AspFluent> plan;
    vector<AspFluent>::iterator action = result->begin();
    for (; action != result->end(); ++action) {  //for every action
      //transform to fluent
      action->setTimeStep(timestep);
      plan.push_back(*action);
      timestep++;
    }
    results.push_back(AnswerSet(plan.begin(), plan.end()));
  }

  return results;
}


//to be tested
set<AspFluent> MultiPolicy::nextExpected(set<AspFluent> initial, AspFluent action) {
  action.setTimeStep(0);
  std::map<std::set<AspFluent>, std::map<AspFluent, std::set<AspFluent> > >::iterator entry = policyWithFinalState.find(initial);
  if (entry==policyWithFinalState.end())
    return set<AspFluent>();
  std::map<AspFluent, std::set<AspFluent> > actionState = entry->second;
  std::map<AspFluent, std::set<AspFluent> >::iterator nextEntry = actionState.find(action);
  if (nextEntry==actionState.end())
    return set<AspFluent>();
  return nextEntry->second;
}


}
