#include <actasp/GraphPolicy.h>

#include <actasp/action_utils.h>

#include <algorithm>


using namespace std;

namespace actasp {

GraphPolicy::GraphPolicy(const ActionSet& actions) :  policyWithFinalState(), allActions(actions), maxPlanLength(0) {}

//MultiPolicy::MultiPolicy() {}

struct ActionFromMap {

  AspFluent operator()(const GraphPolicy::ActionStateMap::value_type& element) {
    return element.first;
  }
};

ActionSet GraphPolicy::actions(const std::set<AspFluent>& state) const throw() {

  GraphMap::const_iterator acts = policyWithFinalState.find(state);

  if (acts != policyWithFinalState.end()) {
    ActionSet actions;
    transform(acts->second.begin(),acts->second.end(),inserter(actions,actions.end()),ActionFromMap());
  }

  return ActionSet();
}

void GraphPolicy::merge(const PartialPolicy* otherPolicy) {
  const GraphPolicy *other = dynamic_cast<const GraphPolicy*>(otherPolicy);
  if(other != NULL)
    merge(other);
  
  throw runtime_error("method not implemented for a partial policy other than GraphPolicy");
}

void GraphPolicy::merge(const AnswerSet& plan) throw(logic_error) {

  unsigned int planLength = plan.maxTimeStep();

  if (planLength > maxPlanLength) {
    maxPlanLength = planLength; //update longest
  }

  set<AspFluent> state = plan.getFluentsAtTime(0); //action for state 0 is at timestep 1

  for (int timeStep = 1; timeStep <= planLength; ++timeStep) {

    set<AspFluent> nextState = plan.getFluentsAtTime(timeStep);

    //find the action
    set<AspFluent>::iterator actionIt = find_if(nextState.begin(),nextState.end(),IsAnAction(allActions));

    if (actionIt == nextState.end())
      throw logic_error("MultiPolicy: no action for some state");

    AspFluent action = *actionIt;

    //remove the action from there
    nextState.erase(actionIt);

    set<AspFluent> final;
    if (timeStep!=planLength) { //else it's the goal state, with no final state
      final = nextState; //currently last state
    }
    action.setTimeStep(0);
    ActionStateMap &actstate = policyWithFinalState[state];
    actstate.insert(make_pair(action, final));

    state = nextState;

  }
}

struct GraphMergeActionsFinal { //merges policy with final states
  
  GraphMergeActionsFinal( GraphPolicy::GraphMap &policyWithFinalState) : policyWithFinalState(policyWithFinalState) {}

 void operator()(const GraphPolicy::GraphMap::value_type& stateActionsState) {

   GraphPolicy::GraphMap::iterator found = policyWithFinalState.find(stateActionsState.first);

   if(found == policyWithFinalState.end())
     policyWithFinalState.insert(stateActionsState);

   else {
    //check if action is already there. if it is, update final state, if it isnt add it.
     found->second.insert(stateActionsState.second.begin(),stateActionsState.second.end());
   }

 }
  GraphPolicy::GraphMap &policyWithFinalState;
};


void GraphPolicy::merge(const GraphPolicy* otherPolicy) {

  set_union(otherPolicy->allActions.begin(),otherPolicy->allActions.end(),
                 allActions.begin(),allActions.end(),
                 inserter(allActions,allActions.begin()));

  for_each(otherPolicy->policyWithFinalState.begin(),otherPolicy->policyWithFinalState.end(),GraphMergeActionsFinal(policyWithFinalState));

  if (otherPolicy->maxPlanLength > maxPlanLength) {
    maxPlanLength = otherPolicy->maxPlanLength;
  }
}



bool GraphPolicy::empty() const throw() {
  return policyWithFinalState.empty();
}

//helper
void GraphPolicy::plansFromRec(const set<AspFluent>& state, vector<vector<AspFluent> >& result, std::vector< set<AspFluent> > visited) const throw() {

  visited.push_back(state); //new state (function does not get called on states that have been visited already)

  GraphMap::const_iterator acts = policyWithFinalState.find(state);
  if (acts == policyWithFinalState.end()) {
    result.pop_back(); //discard last plan
    return;
  }
  const ActionStateMap& options = acts->second;
  ActionStateMap::const_iterator option = options.begin();
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
std::vector<actasp::AnswerSet> GraphPolicy::plansFrom(const set<AspFluent>& state) throw() {

  //the state might not be in the policy
  GraphMap::const_iterator acts = policyWithFinalState.find(state);
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

    vector<AspFluent>::iterator action = result->begin();
    for (int timestep = 1; action != result->end(); ++action, ++timestep) {  //for every action
      action->setTimeStep(timestep);
    }
    results.push_back(AnswerSet(result->begin(), result->end()));
  }

  return results;
}

}
