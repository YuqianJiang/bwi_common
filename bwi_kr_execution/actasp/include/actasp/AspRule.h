#ifndef actasp_AspRule_h__guard
#define actasp_AspRule_h__guard

#include <actasp/AspFluent.h>

#include <vector>
#include <string>

namespace actasp {


struct AspRule {
	
	AspRule() : head(), body() {}
	
	//implicit cast from fluent to rule
	AspRule(const AspFluent& fluent) throw() {
		body.push_back(fluent);
	}
	
	AspRule& operator<< (AspFluent& fluent) throw (){
		body.push_back(fluent);
		return *this;
	}

	bool operator== (const AspRule other) const throw () {
		if (other.head.size() != this->head.size())
			return false;
		if (other.body.size() != this->body.size())
			return false;
		std::vector<AspFluent>::const_iterator otherHead = other.head.begin();
		std::vector<AspFluent>::const_iterator thisHead = this->head.begin();
		for (; otherHead!=other.head.end(); ++otherHead) {
			std::string otherString = otherHead->toString(0);
			std::string thisString = thisHead->toString(0);
			if (otherString.compare(thisString)!=0)
				return false;
			++thisHead;
		}

		std::vector<AspFluent>::const_iterator otherBody = other.body.begin();
		std::vector<AspFluent>::const_iterator thisBody = this->body.begin();
		for (; otherBody!=other.body.end(); ++otherBody) {
			std::string otherString = otherBody->toString(0);
			std::string thisString = thisBody->toString(0);
			if (otherString.compare(thisString)!=0)
				return false;
			++thisBody;
		}

		//at this point:
		return true; 
	}
	
	std::vector<AspFluent> head;
	std::vector<AspFluent> body;

};
	
}
#endif
