#ifndef bwi_krexec_PlanExplainer_h__guard
#define bwi_krexec_PlanExplainer_h__guard

#include "actasp/AnswerSet.h"
#include "actasp/action_utils.h"

namespace bwi_krexec {

class EntityGeneralizer;
class ExplanationState;

struct Predicate{
  Predicate(const actasp::AspFluent& fluent, const bool isTrue) : 
    fluent(fluent),
    grounded(fluent), 
    isTrue(isTrue) {}

  void lift(EntityGeneralizer& entityGeneralizer);

  bool operator<(const Predicate& other) const throw() {

    if (this->isTrue != other.isTrue)
      return other.isTrue;

    /*if (this->fluent.getTimeStep() != other.fluent.getTimeStep()) 
      return this->fluent.getTimeStep() < other.fluent.getTimeStep();

    if (this->fluent.getName() != other.fluent.getName())
      return this->fluent.getName() < other.fluent.getName();*/


    return this->fluent < other.fluent;

  } 
   
  bool operator==(const Predicate& other) const throw() {
    if ((this->fluent == other.fluent) && (this->isTrue == other.isTrue))
      return true;

    return false;
  }

  bool isGrounded() const {
    return (grounded == fluent);
  }

  std::string toString(bool printGrounded = false) const {
    std::string s = fluent.toString(); 
    if (!isGrounded() && printGrounded)
      s = s + "[" + grounded.toString() + "]";
    s = s + "=" + (isTrue ? "true" : "false");
    return s;
  }

  operator std::string() const { return this->toString(); }
  
  actasp::AspFluent fluent;
  actasp::AspFluent grounded;
  bool isTrue;
};

struct PredicateComp {
  bool operator() (const Predicate& lhs, const Predicate& rhs) const {
    if (lhs.isTrue != rhs.isTrue)
      return rhs.isTrue;

    if (lhs.fluent.toString(0) != rhs.fluent.toString(0)) {
      return (lhs.fluent.toString(0) < rhs.fluent.toString(0));
    }

    /*if (lhs.fluent.getName() != rhs.fluent.getName()) {
      return (lhs.fluent.getName() < rhs.fluent.getName());
    }*/

    return false;
  }
};

class PlanExplainer {
public:
  PlanExplainer(const actasp::ActionSet& actions) : isAnAction(actions) {
    //lfd();
  }

  void setPlan(const actasp::AnswerSet& newFluents);

  std::string getAllPairs();
  std::string getRandomExplanation(const int length);
  std::string getRandomExplanation();
  std::string getLearnedExplanation();


private:
  std::map<Predicate, int, PredicateComp> predicateMap;
  
  actasp::IsAnAction isAnAction;
  std::vector<Predicate> plan;
  std::vector<double> weights;

  std::vector<Predicate> getFluentDifference(const std::set<actasp::AspFluent>& set1, 
                                                      const std::set<actasp::AspFluent>& set2);

  std::vector<ExplanationState> buildStateSpace(const std::vector<Predicate>& predicates);

  void liftPlan(std::vector<Predicate>& predicates);

  void lfd();

  void irl(const std::vector<std::vector<ExplanationState> >& mdps, 
          const std::vector<std::vector<double> >& feature_expectation,
          const int n_epochs,
          const int horizon,
          const double learning_rate);

  std::vector<std::vector<double> > calcMaxEntPolicy(const std::vector<ExplanationState>& mdp, 
                                                    const int horizon);

  std::vector<double> calcExpectedStateFreq(const std::vector<ExplanationState>& mdp,
                                            const std::vector<double>& start_dist,
                                            const std::vector<std::vector<double> >& policy,
                                            const int horizon);

};

class EntityGeneralizer{
public:
  EntityGeneralizer() : 
    room_count(0), door_count(0), object_count(0), person_count(0), message_count(0) {}
  std::string operator()(const std::string& variable);

private:
  std::map<std::string, std::string> entityMap;
  int room_count;
  int door_count;
  int object_count;
  int person_count;
  int message_count;

};


}
#endif