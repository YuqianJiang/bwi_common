#pragma once

#include <actasp/ResourceManager.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <actasp/AspFluent.h>

namespace bwi_krexec{

class ActionCostEstimator {
public:
	ActionCostEstimator(actasp::ResourceManager &resourceManager);

	float getCostFromPath(const nav_msgs::Path& plan);

	float getActionCost(const actasp::AspFluent &fluent);

private:
	actasp::ResourceManager &resourceManager;
	geometry_msgs::Pose last_pose;
	bool new_plan;

};

}