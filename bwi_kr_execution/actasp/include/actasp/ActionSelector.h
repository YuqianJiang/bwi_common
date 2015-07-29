#ifndef actasp_ActionSelector_h__guard
#define actasp_ActionSelector_h__guard

#include <actasp/AnswerSet.h>
#include <actasp/MultiPolicy.h>
#include <actasp/AspRule.h>

#include <set>

namespace actasp {

struct ActionSelector  {

	virtual ActionSet::const_iterator choose(const ActionSet &options) throw() = 0;

	virtual void policyChanged(MultiPolicy& policy) throw() = 0;
	virtual void goalChanged(std::vector<actasp::AspRule> newGoalRules) throw() = 0;

	virtual ~ActionSelector() {}
};
}

#endif
