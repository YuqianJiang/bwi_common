#include "NavigateTo.h"

#include "plan_execution/msgs_utils.h"

#include "actasp/AnswerSet.h"
#include <knowledge_representation/convenience.h>

#include <ros/ros.h>

#include <algorithm>

using namespace std;
using namespace actasp;

namespace bwi_krexec {


    NavigateTo:: NavigateTo(int location_id, knowledge_rep::LongTermMemoryConduit &ltmc):
LogicalNavigation("navigate_to", ltmc),
            location_id(location_id) {
    }


    void NavigateTo::run()  {
        bwi_krexec::LogicalNavigation::run();
    }

    std::vector<std::string> NavigateTo::getParameters() const {
        std::vector<std::string> parameters;
        parameters.push_back(to_string(location_id));
        return parameters;
    }

    boost::optional<std::vector<std::string> > NavigateTo::prepareGoalParameters() const {
        boost::optional<std::string> name = getLocationName(location_id, ltmc);
        if (name) {
            vector<string> parameters;
            parameters.push_back(*name);
            return parameters;
        }
        
    }

    void NavigateTo::onFinished(bool success, const bwi_msgs::LogicalNavResult &result) {
        // Allow super to update the knowledge base
        LogicalNavigation::onFinished(success, result);
        // TODO: Send speech for unstuck
    }


}
