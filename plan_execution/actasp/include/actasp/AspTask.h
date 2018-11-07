#pragma once

#include <actasp/AspRule.h>
#include <iterator>

namespace actasp {

struct AspTask {
	AspTask(const std::vector<AspFluent> &initialFluents, const std::vector<AspRule> &goalRules) :
		initialFluents(initialFluents),
		goalRules(goalRules) {
			std::stringstream ss;

			if (initialFluents.size() == 0) {
				ss << "(),";
			}
			else {
				ss << "(";

				transform(initialFluents.begin(), initialFluents.end()-1, std::ostream_iterator<std::string>(ss, ","),
								[](const AspFluent& fluent){return fluent.toStringNoTimeStep();});
				ss << initialFluents.back().toStringNoTimeStep();
				ss << "),";
			}

			if (goalRules.size() == 0) {
				ss << "()";
			}
			else {
				ss << "(";

				for (const auto& rule : goalRules) {
					if (rule.body.size() == 0) {
						continue;
					}

					ss << "[";
					transform(rule.body.begin(), rule.body.end()-1, std::ostream_iterator<std::string>(ss, ","), 
              		[](const AspFluent& fluent){return fluent.toStringNoTimeStep();});
					ss << rule.body.back().toStringNoTimeStep();
					ss << "]";
				}
				ss << ")";
			}

			name = ss.str();

	}

	std::string getGoalString() {
		std::stringstream ss;

		for (const auto& rule : goalRules) {
			if (rule.body.size() == 0) {
				continue;
			}

			ss << "[";
			transform(rule.body.begin(), rule.body.end()-1, std::ostream_iterator<std::string>(ss, ","), 
          		[](const AspFluent& fluent){return fluent.toStringNoTimeStep();});
			ss << rule.body.back().toStringNoTimeStep();
			ss << "]";
		}

		return ss.str();
	}

	bool operator<(const AspTask& other) const throw() {
		return this->name < other.name;
	}

	bool operator==(const AspTask& other) const throw() {
		return this->name == other.name;
	}

	std::vector<AspFluent> initialFluents;
	std::vector<AspRule> goalRules;
	std::string name;
};

}