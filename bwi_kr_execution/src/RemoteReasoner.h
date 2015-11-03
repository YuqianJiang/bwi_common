
#ifndef bwi_krexec_RemoteReasoner_h__guard
#define bwi_krexec_RemoteReasoner_h__guard

#include "actasp/AspAtom.h"
#include "actasp/AspKR.h"
#include "actasp/reasoners/Clingo.h"

namespace bwi_krexec {
  
  class RemoteReasoner : public actasp::AspKR {
  public:
    
    RemoteReasoner(unsigned int max_n,
         const std::string& queryDir,
         const std::string& domainDir,
         const actasp::ActionSet& actions,
         unsigned int max_time = 0);
    
  actasp::ActionSet availableActions() const throw();
  
  actasp::AnswerSet currentStateQuery(const std::vector<actasp::AspRule>& query) const throw();
  
  bool updateFluents(const std::vector<actasp::AspFluent> &observations) throw();
  
  bool isPlanValid(const actasp::AnswerSet& plan, const std::vector<actasp::AspRule>& goal)  const throw();
  
  std::vector< actasp::AnswerSet > computeAllPlans(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error);
  
  void reset() throw();

  actasp::MultiPolicy computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error);

  actasp::MultiPolicy computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality, bool finalState) const throw (std::logic_error); //filterstate stuff
  actasp::AnswerSet filterState(const std::vector<actasp::AnswerSet>& plans, const std::vector<actasp::AspRule>& goals); //filterstate stuff
  std::set<actasp::AspFluent> actionEffects(const actasp::AspFluent& action, const std::set<actasp::AspFluent>& state); //filterstate stuff
  
  actasp::AnswerSet computePlan(const std::vector<actasp::AspRule>& goal) const throw (std::logic_error);
  
  std::list< std::list<actasp::AspAtom> > query(const std::string &queryString, unsigned int initialTimeStep,
                                   unsigned int finalTimeStep) const throw();
  
  private:
    actasp::Clingo local;
  
  };
}

#endif
