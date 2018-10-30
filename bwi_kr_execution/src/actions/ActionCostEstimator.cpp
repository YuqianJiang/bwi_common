#include "ActionCostEstimator.h"
#include "LogicalNavigation.h"

using namespace std;

namespace bwi_krexec{
	static float failure_cost = 100;

	ActionCostEstimator::ActionCostEstimator(actasp::ResourceManager &resourceManager) :
			resourceManager(resourceManager),
			last_pose(last_pose),
			new_plan(true) {

	}

	float ActionCostEstimator::getCostFromPath(const nav_msgs::Path& plan) {
		float distance = 0;
		auto& poses = plan.poses;

		for (int i = 1; i < poses.size(); ++i) {
        distance += sqrt(pow((poses[i].pose.position.x - poses[i-1].pose.position.x), 2) +
                        pow((poses[i].pose.position.y - poses[i-1].pose.position.y), 2));
    }

    return distance * 2;

	}

	float ActionCostEstimator::getActionCost(const actasp::AspFluent &fluent) {
		if ((fluent.getName() == "navigate_to") || (fluent.getName() == "go_through")) {
			nav_msgs::Path plan = LogicalNavigation::getPathPlan(fluent, last_pose, resourceManager, new_plan);
			auto& poses = plan.poses;

			if (poses.size() == 0) {
				return failure_cost;
			}

      last_pose = poses[poses.size() - 1].pose;
			new_plan = false;
      
      float cost = getCostFromPath(plan);

      ROS_INFO_STREAM("Estimated cost for " << fluent.toStringNoTimeStep() <<  " is " << cost);

      return cost;
			
		}

		
	}

}