#include <actasp/action_utils.h>


#include <iterator>

using namespace std;

namespace actasp {

struct ActionName {
  string operator()(const AspFluent& action) const {
    return action.getName();
  }

};

IsAnAction::IsAnAction(const ActionSet& actions) {
  transform(actions.begin(), actions.end(), inserter(actionNames, actionNames.begin()), ActionName());
}

bool IsAnAction::operator()(const AspFluent& fluent) const {

  return actionNames.find(fluent.getName()) != actionNames.end();
}

AnswerSet planToAnswerSet(const std::list<std::unique_ptr<Action>> &plan) {
  auto actIt = plan.begin();
  set<AspFluent> fluents;

  for (int timeStep=0; actIt != plan.end(); ++actIt, ++timeStep) {
    fluents.insert((*actIt)->toFluent(timeStep));
  }

  return AnswerSet(fluents.begin(), fluents.end());
}

ActionSet actionMapToSet(const std::map<std::string, ActionFactory>& actionMap) {

  ActionSet fluents;

  for (const auto &pair: actionMap) {
    // Put the real name in and a fake number of parameters
    fluents.insert(AspFluent(pair.first, {2,""}));
  }
  return fluents;
}

ActionSet actionMapToSet(const std::map<std::string, Action *>& actionMap) {

  ActionSet fluents;
  for (const auto &pair: actionMap)
    fluents.insert(AspFluent(pair.second->toFluent(0)));

  return fluents;
}

std::set<AspFluent> removeActions(const std::set<AspFluent>& fluents, const ActionSet& allActions) {
  std::set<AspFluent> noActions;
  remove_copy_if(fluents.begin(),fluents.end(),std::inserter(noActions, noActions.begin()),IsAnAction(allActions));
  return noActions;
}

std::set<AspFluent> extractActions(const std::set<AspFluent>& fluents, const ActionSet& allActions) {
  std::set<AspFluent> actions;
  remove_copy_if(fluents.begin(),fluents.end(),std::inserter(actions, actions.begin()),not1(IsAnAction(allActions)));
  return actions;
}

list<AnswerSet> filterPlans(const list<AnswerSet> unfiltered_plans, const ActionSet& allActions) {

  list<AnswerSet> plans;

  list<AnswerSet>::const_iterator ans = unfiltered_plans.begin();
  for (; ans != unfiltered_plans.end(); ++ans) {
    list<AspFluent> actionsOnly;
    remove_copy_if(ans->getFluents().begin(),ans->getFluents().end(),back_inserter(actionsOnly),not1(IsAnAction(allActions)));

    plans.push_back(AnswerSet(actionsOnly.begin(), actionsOnly.end()));
  }

  return plans;
}

}
