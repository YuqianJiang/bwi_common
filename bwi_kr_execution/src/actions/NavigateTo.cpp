#include "NavigateTo.h"

#include "plan_execution/msgs_utils.h"

#include "actasp/AnswerSet.h"
#include <knowledge_representation/Entity.h>

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

    boost::optional<std::string> getLocationName(int location_id, knowledge_rep::LongTermMemoryConduit &ltmc) {
        knowledge_rep::Entity location(location_id, ltmc);
        if (!location.is_valid()) {
            return boost::none;
        }

        auto attrs = location.get_attributes("name");
        if (attrs.size() != 1) {
            return boost::none;
        }

        return attrs.at(0).get_string_value();
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

    boost::optional<float> NavigateTo::getEstimatedCost(const actasp::AspFluent & fluent, actasp::ResourceManager &resource_manager) {
        
        ROS_INFO_STREAM("Estimating cost for " << fluent.toStringNoTimeStep());

        auto& resource_manager_cast = dynamic_cast<BwiResourceManager&>(resource_manager);
        boost::optional<std::string> name = getLocationName(std::atoi(fluent.getParameters().at(0).c_str()), resource_manager_cast.ltmc);
        if (! name) {
            return boost::none;
        }

        vector<string> parameters;
        parameters.push_back(*name);

        ros::NodeHandle n;
        ros::ServiceClient pathPlanClient = n.serviceClient<bwi_msgs::LogicalNavPlan>("/get_path_plan");

        pathPlanClient.waitForExistence();

        bwi_msgs::LogicalNavPlan srv;
        srv.request.command.name = fluent.getName();
        srv.request.command.value = parameters;

        if (pathPlanClient.call(srv)) {
            float distance = 0;
            auto& poses = srv.response.plan.poses;

            if (poses.size() <= 1) {
                return boost::none;
            }

            for (int i = 1; i < poses.size(); ++i) {
                distance += sqrt(pow((poses[i].pose.position.x - poses[i-1].pose.position.x), 2) +
                                pow((poses[i].pose.position.y - poses[i-1].pose.position.y), 2));
            }

            float cost = distance * 2;
            ROS_INFO_STREAM("Estimated cost is " << cost);

            return cost;
        }
        else {
            return boost::none;
        }
   
    }


}
