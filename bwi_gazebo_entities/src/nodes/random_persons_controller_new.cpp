#include <boost/foreach.hpp>
#include <bwi_mapper/path_finder.h>
#include <bwi_mapper/map_inflator.h>
#include <bwi_mapper/map_loader.h>
#include <bwi_mapper/map_utils.h>
#include <bwi_msgs/AvailableRobotWithLocationArray.h>
#include <bwi_tools/common/RNG.h>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>

using namespace std;

struct Person {
  Person(const string name, const geometry_msgs::Pose pose_1, const geometry_msgs::Pose pose_2) :
        model_name("auto_person_" + name), pose_1(pose_1), pose_2(pose_2) {}
  string model_name;
  geometry_msgs::Pose pose_1;
  geometry_msgs::Pose pose_2;
  geometry_msgs::Pose location;
  bwi_mapper::Point2d goal;
  ros::Subscriber location_subscriber;
  ros::Publisher command_publisher;
  boost::shared_ptr<bwi_mapper::PathFinder> path_finder;
  ros::Time pause_start_times;
  bool active;
  bool paused;
};

vector<Person> persons;

ros::ServiceClient spawn_model_client;
ros::ServiceClient delete_model_client;
int total_random_persons;
int active_random_persons;
std::string map_file;
float person_diameter = 0.5;
float linear_velocity_multiplier = 1.0;
float angular_velocity_multiplier = 1.0;
std::string person_urdf;

nav_msgs::OccupancyGrid map_;
nav_msgs::OccupancyGrid inflated_map_;

ros::Publisher status_publisher;

bwi_mapper::Point2d getPersonLocation(int person_idx, int try_alternates = 0) {
  bwi_mapper::Point2f person_loc_map(persons[person_idx].location.position.x, persons[person_idx].location.position.y);
  bwi_mapper::Point2f current_pt_f = bwi_mapper::toGrid(person_loc_map, map_.info);
  bwi_mapper::Point2d current_pt(current_pt_f.x, current_pt_f.y);
  for (int x = -try_alternates; x <= try_alternates; ++x) {
    for (int y = -try_alternates; y <= try_alternates; ++y) {
      bwi_mapper::Point2d test_diff = bwi_mapper::Point2d(x, y);
      bwi_mapper::Point2d test_pt = current_pt + test_diff;
      int idx = test_pt.y * map_.info.width + test_pt.x;
      if (inflated_map_.data[idx] != 100) {
        return test_pt;
      }
    }
  }
  return current_pt;
}

bool sendVelocityCommand(int person_idx) {
  bwi_mapper::Point2d current_pt;
  int alt = 0;
  while(alt < 4) {
    if (alt == 4) {
      std::cout << "person stuck in obstacle. not sure what to do." << std::endl;
    }
    current_pt = getPersonLocation(person_idx, alt);
    int idx = current_pt.y * map_.info.width + current_pt.x;
    if (inflated_map_.data[idx] != 100) {
      break;
    }
    ++alt;
  }
  for (int i = 0; i < 5; ++i) {
    bwi_mapper::Point2d next_pt;
    if (!persons[person_idx].path_finder->getNextCloserPointToSearchOrigin(current_pt, next_pt)) {
      // Close enough to the goal.
      /* std::cout << "reached goal" << std::endl; */
      persons[person_idx].paused = true;
      persons[person_idx].pause_start_times = ros::Time::now();

      // Publish zero velocity.
      geometry_msgs::Twist twist_msg;
      persons[person_idx].command_publisher.publish(twist_msg);
      return false;
    }
    current_pt = next_pt;
  }

  // Now calculate the difference to current_pt.
  bwi_mapper::Point2f interm_loc = bwi_mapper::toMap(current_pt, map_.info);

  float xdiff = interm_loc.x - persons[person_idx].location.position.x;
  float ydiff = interm_loc.y - persons[person_idx].location.position.y;
  float orientation = tf::getYaw(persons[person_idx].location.orientation);
  /* float adiff = 0; */
  float adiff = atan2f(ydiff, xdiff) - orientation;
  while (adiff <= -M_PI) adiff += 2*M_PI;
  while (adiff > M_PI) adiff -= 2*M_PI;

  float xdiffnew = xdiff * cosf(orientation) + ydiff * sinf(orientation);
  float ydiffnew = ydiff * cosf(orientation) - xdiff * sinf(orientation);
  xdiff = xdiffnew;
  ydiff = ydiffnew;

  geometry_msgs::Twist twist_msg;

  twist_msg.linear.x = linear_velocity_multiplier * xdiff;
  twist_msg.linear.x = std::max(twist_msg.linear.x, -0.5);
  twist_msg.linear.x = std::min(twist_msg.linear.x, 0.5);

  twist_msg.linear.y = linear_velocity_multiplier * ydiff;
  twist_msg.linear.y = std::max(twist_msg.linear.y, -0.5);
  twist_msg.linear.y = std::min(twist_msg.linear.y, 0.5);

  twist_msg.angular.z = angular_velocity_multiplier * adiff;
  twist_msg.angular.z = std::max(twist_msg.angular.z, -0.5);
  twist_msg.angular.z = std::min(twist_msg.angular.z, 0.5);

  persons[person_idx].command_publisher.publish(twist_msg);

  return true;
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom, int person_idx) {
  persons[person_idx].location = odom->pose.pose;
}

int main(int argc, char *argv[]) {
  ros::init (argc, argv, "random_persons_controller");

  ros::NodeHandle nh, private_nh("~");

  spawn_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_urdf_model");
  spawn_model_client.waitForExistence();
  delete_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/delete_model");
  spawn_model_client.waitForExistence();

  private_nh.getParam("num_persons", total_random_persons);
  private_nh.getParam("person_urdf", person_urdf);
  active_random_persons = 0;

  bwi_mapper::MapLoader mapper(map_file);
  mapper.getMap(map_);

  bwi_mapper::inflateMap(person_diameter / 2, map_, inflated_map_);

  status_publisher = private_nh.advertise<bwi_msgs::AvailableRobotWithLocationArray>("status", 1);

  return 0;
}
