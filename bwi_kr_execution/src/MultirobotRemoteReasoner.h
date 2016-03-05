#ifndef bwi_krexec_MultirobotRemoteReasoner_h__guard
#define bwi_krexec_MultirobotRemoteReasoner_h__guard

#include "actasp/AspKR.h"
#include <actasp/reasoners/FilteringReasoner.h>
#include <actasp/FilteringQueryGenerator.h>

namespace bwi_krexec {

class MultirobotRemoteReasoner : public actasp::AspKR {
public:
	MultirobotRemoteReasoner(std::string name, actasp::FilteringQueryGenerator *actualReasoner, unsigned int max_n, const actasp::ActionSet& allActions);
	actasp::ActionSet availableActions() const throw();
	actasp::AnswerSet currentStateQuery(const std::vector<actasp::AspRule>& query) const throw();
	bool updateFluents(const std::vector<actasp::AspFluent> &observations) throw();
	std::list< std::list<actasp::AspAtom> > query(const std::string &queryString, unsigned int timestep) const throw();
	bool isPlanValid(const actasp::AnswerSet& plan, const std::vector<actasp::AspRule>& goal) const throw();
	void resetCurrentState() throw();
	
	actasp::AnswerSet computePlan(const std::vector<actasp::AspRule>& goal) const throw (std::logic_error);
	std::vector< actasp::AnswerSet > computeAllPlans(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error);
	actasp::PartialPolicy* computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error);

private:
	std::string name;
    actasp::FilteringReasoner local;
};

}

#endif