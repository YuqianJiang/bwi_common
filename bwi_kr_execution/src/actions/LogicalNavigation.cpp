#include "LogicalNavigation.h"

#include "actasp/AspFluent.h"
#include <plan_execution/AspFluent.h>
#include <knowledge_representation/LTMCEntity.h>
#include <knowledge_representation/LTMCInstance.h>
#include "../BwiResourceManager.h"

#include <ros/ros.h>

#include <sstream>

using namespace ros;
using namespace std;
using namespace actasp;
using namespace knowledge_rep;

namespace bwi_krexec {


LogicalNavigation::LogicalNavigation(const std::string &logical_name, knowledge_rep::LongTermMemoryConduit &ltmc) :
        name(logical_name), ltmc(ltmc),
        LogicalNavigationRosAction("execute_logical_action") {}

boost::optional<bwi_msgs::LogicalNavGoal> LogicalNavigation::prepareGoal() {
    auto params = prepareGoalParameters();
    if (params) {
        bwi_msgs::LogicalNavGoal goal;
        goal.command.name = name;
        goal.command.value = *params;
        return goal;
    }
}

boost::optional<std::string> LogicalNavigation::getLocationName(int location_id, knowledge_rep::LongTermMemoryConduit &ltmc) {
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

void LogicalNavigation::onFinished(bool succeeded, const bwi_msgs::LogicalNavResult &result) {
    // Dump observations somewhere
    Instance self = ltmc.get().get_robot();
    self.remove_attribute("is_in");
    self.remove_attribute("is_near");
    self.remove_attribute("is_facing");

    ROS_INFO_STREAM(result.observations);

    if (!result.observations.room.empty()) {
        vector<Entity> rooms = ltmc.get().get_entities_with_attribute_of_value("name", result.observations.room);
        if (!rooms.empty()) {
            self.add_attribute("is_in", rooms[0]);
        }
    }
    for (int i = 0; i < result.observations.nearby_locations.size(); i++) {
        auto location_name = result.observations.nearby_locations.at(i);
        vector<Entity> locations = ltmc.get().get_entities_with_attribute_of_value("name", location_name);
        if (!locations.empty()) {
            self.add_attribute("is_near", locations[0]);
            if (result.observations.facing.at(i)) {
                self.add_attribute("is_facing", locations[0]);
            }
        } else {
            ROS_WARN_STREAM("Logical navigation state says robot is near " << location_name << " but no location with that map_name exists in the knowledge base.");
        }
    }

    if (!succeeded && name == "navigate_to") {

        //ROS_INFO_STREAM("Sent speech goal for unstuck");
        // TODO: Add speech here
    }
}

void LogicalNavigation::configureWithResources(ResourceManager &resource_manager) {
  auto& cast = dynamic_cast<BwiResourceManager&>(resource_manager);
  this->ltmc = cast.ltmc;
}

nav_msgs::Path LogicalNavigation::getPathPlan(const AspFluent &fluent, const geometry_msgs::Pose &start_pose,
                                            ResourceManager &resource_manager, bool use_robot_pose) {

    auto& resource_manager_cast = dynamic_cast<BwiResourceManager&>(resource_manager);
    boost::optional<std::string> name = getLocationName(std::atoi(fluent.getParameters().at(0).c_str()), resource_manager_cast.ltmc);
    if (! name) {
        ROS_INFO_STREAM("Location does not exist!");
        return nav_msgs::Path();
    }

    vector<string> parameters;
    parameters.push_back(*name);

    ros::NodeHandle n;
    ros::ServiceClient pathPlanClient = n.serviceClient<bwi_msgs::LogicalNavPlan>("/get_path_plan");

    pathPlanClient.waitForExistence();

    bwi_msgs::LogicalNavPlan srv;
    srv.request.command.name = fluent.getName();
    srv.request.command.value = parameters;
    srv.request.start = start_pose;
    srv.request.use_robot_pose = use_robot_pose;

    if (pathPlanClient.call(srv) && (srv.response.success)) {
        return srv.response.plan;
    }

    ROS_INFO_STREAM("Failed to obtain path plan!!!!!!!!!!!!!!!!!!!!");

    return nav_msgs::Path();
}


} //namespace
