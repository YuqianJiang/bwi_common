#ifndef actasp_MultirobotActionExecutor_h__guard
#define actasp_MultirobotActionExecutor_h__guard


#include <actasp/ActionExecutor.h>

#include <stdexcept>
#include <list>
#include <map>

namespace actasp {

class AspKR;
class Planner;
class Action;
class PlanningObserver;
class AnswerSet;

class MultirobotActionExecutor : public ActionExecutor {

public:
	
	MultirobotActionExecutor(actasp::AspKR* reasoner, 
							 actasp::Planner *planner,
							 const std::map<std::string, Action * > &actionMap
							) throw (std::invalid_argument);
	
  using ActionExecutor::setGoal;
	void setGoal(const std::vector<actasp::AspRule>& goalRules) throw();

	bool goalSet() const throw() {
		return isGoalSet;
	}

	bool goalReached() const throw() {
		return isGoalReached;
	}
	
	bool failed() const throw() {
		return hasFailed;
	}

	void executeActionStep();

	void acceptNewPlan(const double time, const actasp::AnswerSet newPlan);
  
  void addExecutionObserver(ExecutionObserver *observer) throw();
  void removeExecutionObserver(ExecutionObserver *observer) throw();
  
  void addPlanningObserver(PlanningObserver *observer) throw();
  void removePlanningObserver(PlanningObserver *observer) throw();
	
	~MultirobotActionExecutor();
	

private:
	std::vector<actasp::AspRule> goalRules;
	bool isGoalSet;
	bool isGoalReached;
	bool hasFailed;
	std::map<std::string, Action * > actionMap;
	
	std::list<Action *> plan;
  unsigned int actionCounter;
  bool newAction;
	
	AspKR* kr;
	Planner *planner;
  
  std::list<ExecutionObserver*> executionObservers;
  std::list<PlanningObserver*> planningObservers;

  double lastPlanTime;
  
  void computePlan();


};


}
#endif
