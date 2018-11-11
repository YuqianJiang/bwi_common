#include <boost/foreach.hpp>
#include <bwi_mapper/path_finder.h>
#include <bwi_mapper/map_inflator.h>
#include <bwi_mapper/map_loader.h>
#include <bwi_mapper/map_utils.h>
#include "bwi_msgs/AvailableRobotArray.h"
#include <bwi_tools/common/RNG.h>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>

using namespace std;

#define NUM_MODEL 3

struct Robot {
  Robot(const int robot_idx, const string name, const ros::Subscriber odom_subscriber) : 
        robot_idx(robot_idx), name(name), pose(), odom_subscriber(odom_subscriber) {}
  int robot_idx;
  string name;
  geometry_msgs::Pose pose;
  ros::Subscriber odom_subscriber;
};

struct Person {
  Person(const int person_idx, const geometry_msgs::Pose pose_1, const geometry_msgs::Pose pose_2, 
          const ros::Subscriber location_subscriber, const ros::Publisher command_publisher) :
        person_idx(person_idx), model_name("auto_person_" + boost::lexical_cast<std::string>(person_idx)), goal(),
        location_subscriber(location_subscriber), command_publisher(command_publisher), path_finder(),
        pause_start_time(), active(false), paused(false), flashing(false), pausing_pose() {
          poses[0] = pose_1;
          poses[1] = pose_2;
          inactive_pose.position.x = 20.2 + 0.8*person_idx;
          inactive_pose.position.y = 0.8;
          inactive_pose.orientation.w = 1;
        }

  void init();

  bwi_mapper::Point2d getPersonLocation(int try_alternates);

  bool sendVelocityCommand();

  void launch(const int pose_idx);

  void pause();

  bool teleport(const geometry_msgs::Pose& pose);

  int person_idx;
  string model_name;
  geometry_msgs::Pose poses[2];
  geometry_msgs::Pose location;
  geometry_msgs::Pose inactive_pose;
  bwi_mapper::Point2d goal;
  ros::Subscriber location_subscriber;
  ros::Publisher command_publisher;
  boost::shared_ptr<bwi_mapper::PathFinder> path_finder;
  ros::Time pause_start_time;
  bool active;
  bool paused;
  bool flashing;
  geometry_msgs::Pose pausing_pose;
};

vector<Person> persons;
vector<Robot> robots;

ros::ServiceClient spawn_model_client;
ros::ServiceClient set_model_client;
int total_random_persons;
int active_random_persons;
std::string map_file;
float person_diameter = 0.5;
float linear_velocity_multiplier = 2.0;
float angular_velocity_multiplier = 2.0;
std::string person_urdf;

nav_msgs::OccupancyGrid map_;
nav_msgs::OccupancyGrid inflated_map_;

RNG rng(time(NULL));

bool isCloseToRobot(geometry_msgs::Pose pose, bool checkOrientation) {
  for (vector<Robot>::iterator it = robots.begin(); it != robots.end(); ++it) {
    float xdiff = pose.position.x - it->pose.position.x;
    float ydiff = pose.position.y - it->pose.position.y;
    float distance = sqrt(pow(xdiff,2)+pow(ydiff,2));
    if (distance < 2.5) {
      return true;
    }

    if ((distance < 3) && (checkOrientation)) {
      float angle = atan2f(ydiff, xdiff) - tf::getYaw(it->pose.orientation);
      if (abs(angle)<0.5) {
        return true;
      }
    }
  }
  return false;
}

bwi_mapper::Point2d Person::getPersonLocation(int try_alternates = 0) {
  bwi_mapper::Point2f person_loc_map(this->location.position.x, this->location.position.y);
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

void Person::pause() {
  this->paused = true;
  this->pause_start_time = ros::Time::now();
  pausing_pose = location;

  // Publish zero velocity.
  geometry_msgs::Twist twist_msg;
  this->command_publisher.publish(twist_msg);
  ROS_INFO_STREAM(model_name << " is paused");
}

bool Person::teleport(const geometry_msgs::Pose& pose) {
  gazebo_msgs::SetModelState set_srv;
  set_srv.request.model_state.model_name = model_name;
  set_srv.request.model_state.pose = pose;
  set_srv.request.model_state.twist = geometry_msgs::Twist();
  if (set_model_client.call(set_srv)) {
    if (set_srv.response.success) {
      return true;
    } else {
      ROS_WARN_STREAM("Received error message while teleporting object: " << set_srv.response.status_message);
      return false;
    }
  } else {
    ROS_ERROR_STREAM("Unable to teleport due to service request failure: " << model_name);
    return false;
  }
}

bool Person::sendVelocityCommand() {
  if (this->paused) {
    if ((ros::Time::now()-pause_start_time)>ros::Duration(7)) {
      if (teleport(inactive_pose)) {
        ROS_INFO_STREAM(model_name << " is inactive");
        active = false;
        paused = false;
        flashing = false;
        active_random_persons--;
      } 
      return true;
    }
    else {
      /*geometry_msgs::Twist twist_msg;
      this->command_publisher.publish(twist_msg);*/
      if (!flashing) {
        if (teleport(inactive_pose)) {
          flashing = true;
        } 
      }
      else {
        if (teleport(pausing_pose)) {
          flashing = false;
        }
      }
      return true;
    }
  }

  if (isCloseToRobot(location, true)) {
    ROS_INFO_STREAM(model_name << " is too close to robot");
    pause();
    if (teleport(inactive_pose)) {
      ROS_INFO_STREAM(model_name << " is inactive");
      active = false;
      paused = false;
      flashing = false;
      active_random_persons--;
    }
    return true;
  }

  bwi_mapper::Point2d current_pt;
  int alt = 0;
  while(alt < 4) {
    if (alt == 4) {
      std::cout << "person stuck in obstacle. not sure what to do." << std::endl;
    }
    current_pt = this->getPersonLocation(alt);
    int idx = current_pt.y * map_.info.width + current_pt.x;
    if (inflated_map_.data[idx] != 100) {
      break;
    }
    ++alt;
  }

  for (int i = 0; i < NUM_MODEL; ++i) {
    bwi_mapper::Point2d next_pt;
    if (!this->path_finder->getNextCloserPointToSearchOrigin(current_pt, next_pt)) {
      // Close enough to the goal.
      geometry_msgs::Twist twist_msg;
      this->command_publisher.publish(twist_msg);
      if (teleport(inactive_pose)) {
        ROS_INFO_STREAM(model_name << " is inactive");
        active = false;
        paused = false;
        flashing = false;
        active_random_persons--;
      }
      return true;
    }
    current_pt = next_pt;
  }

  // Now calculate the difference to current_pt.
  bwi_mapper::Point2f interm_loc = bwi_mapper::toMap(current_pt, map_.info);

  float xdiff = interm_loc.x - this->location.position.x;
  float ydiff = interm_loc.y - this->location.position.y;
  float orientation = tf::getYaw(this->location.orientation);
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
  twist_msg.linear.x = std::max(twist_msg.linear.x, -0.8);
  twist_msg.linear.x = std::min(twist_msg.linear.x, 0.8);

  twist_msg.linear.y = linear_velocity_multiplier * ydiff;
  twist_msg.linear.y = std::max(twist_msg.linear.y, -0.8);
  twist_msg.linear.y = std::min(twist_msg.linear.y, 0.8);

  twist_msg.angular.z = angular_velocity_multiplier * adiff;
  twist_msg.angular.z = std::max(twist_msg.angular.z, -0.5);
  twist_msg.angular.z = std::min(twist_msg.angular.z, 0.5);

  this->command_publisher.publish(twist_msg);

  return true;
}

void Person::launch(const int pose_idx) {
  ROS_INFO_STREAM("Launching person " << model_name);

  gazebo_msgs::SetModelState set_srv;
  set_srv.request.model_state.model_name = model_name;
  set_srv.request.model_state.pose = poses[pose_idx];
  set_srv.request.model_state.twist = geometry_msgs::Twist();

  if (set_model_client.call(set_srv)) {
    if (set_srv.response.success) {
      bwi_mapper::Point2f goal_map(poses[1-pose_idx].position.x, poses[1-pose_idx].position.y);
      bwi_mapper::Point2f goal_grid = bwi_mapper::toGrid(goal_map, map_.info);
      goal = bwi_mapper::Point2d(goal_grid.x, goal_grid.y);
      path_finder.reset(new bwi_mapper::PathFinder(inflated_map_, goal));
      location = poses[pose_idx];
      paused = false;
      active = true;
      active_random_persons++;
      ROS_INFO_STREAM(model_name << " is active");
    } else {
      ROS_WARN_STREAM("Received error message while teleporting object: " << set_srv.response.status_message);
    }
  } else {
    ROS_ERROR_STREAM("Unable to teleport due to service request failure: " << model_name);
  }
}

void Person::init() {
  gazebo_msgs::SpawnModel spawn;
  spawn.request.model_name = model_name;

  ros::param::set("/" + spawn.request.model_name + "/tf_prefix", spawn.request.model_name);

  spawn.request.model_xml = person_urdf;
  spawn.request.robot_namespace = spawn.request.model_name;
  spawn.request.initial_pose = inactive_pose;

  if (spawn_model_client.call(spawn)) {
    if (spawn.response.success) {
      location = spawn.request.initial_pose;
      ROS_INFO_STREAM("Spawning person" << spawn.request.model_name);
    } else {
      ROS_WARN_STREAM("Received error message while spawning object: " << spawn.response.status_message);
    }
  } else {
    ROS_ERROR_STREAM("Unable to spawn due to service request failure: " << spawn.request.model_name);
  }
}

void personOdometryHandler(const nav_msgs::Odometry::ConstPtr& odom, int person_idx) {
  persons[person_idx].location = odom->pose.pose;
}

void robotOdometryHandler(const nav_msgs::Odometry::ConstPtr& odom, int robot_idx) {
  robots[robot_idx].pose = odom->pose.pose;
}

void addRobots(const bwi_msgs::AvailableRobotArray::ConstPtr& allRobots) {
  ros::NodeHandle nh;
  for (int i = 0; i < allRobots->robots.size(); ++i) {
    if (allRobots->robots[i].type == 1) {
      robots.push_back(Robot(i, allRobots->robots[i].name, nh.subscribe<nav_msgs::Odometry>("/"+allRobots->robots[i].name+"/odom",1,boost::bind(&robotOdometryHandler, _1, i))));
    }
  }
}

void initializePersons();

int main(int argc, char *argv[]) {
  ros::init (argc, argv, "random_persons_node");

  ros::NodeHandle nh, private_nh("~");

  spawn_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_urdf_model");
  spawn_model_client.waitForExistence();
  set_model_client = nh.serviceClient<gazebo_msgs::SetModelState>("gazebo/set_model_state");
  set_model_client.waitForExistence();

  ros::Subscriber start_listener = nh.subscribe("available_robots",1,addRobots);

  private_nh.getParam("map_file", map_file);
  private_nh.getParam("num_persons", total_random_persons);
  private_nh.getParam("person_urdf", person_urdf);
  active_random_persons = 0;

  bwi_mapper::MapLoader mapper(map_file);
  mapper.getMap(map_);

  bwi_mapper::inflateMap(person_diameter / 2, map_, inflated_map_);

  initializePersons();

  robots.push_back(Robot(0, "default", nh.subscribe<nav_msgs::Odometry>("/odom",1,boost::bind(&robotOdometryHandler, _1, 0))));

  ros::Rate r(120);
  while (ros::ok()) {
    ros::spinOnce();

    while (active_random_persons < total_random_persons) {
      int person_idx = rng.randomInt(persons.size()-1);
      int pose_idx = rng.randomInt(1);
      while ((persons[person_idx].active) || isCloseToRobot(persons[person_idx].poses[pose_idx], false)) {
        person_idx = rng.randomInt(persons.size()-1);
        pose_idx = rng.randomInt(1);
      }
      persons[person_idx].launch(pose_idx);
    }

    for (int i = 0; i < persons.size(); ++i) {
      if (persons[i].active) {
        persons[i].sendVelocityCommand();
      }
    }
    r.sleep();
  }

  return 0;
}

void initializePersons() {
  ros::NodeHandle nh;

  float pts[NUM_MODEL][4] = 
  {
    {19.6, 105.55, 23.2, 104.95},
    {24.2, 104.95, 29.2, 105},
    {29.7, 105, 38.2, 105}
  };

  for (int i = 0; i < NUM_MODEL; ++i) {
    geometry_msgs::Pose pose_1;
    pose_1.position.x = pts[i][0];
    pose_1.position.y = pts[i][1];
    geometry_msgs::Pose pose_2;
    pose_2.position.x = pts[i][2];
    pose_2.position.y = pts[i][3];
    pose_1.orientation = tf::createQuaternionMsgFromYaw(atan2f(pose_2.position.y-pose_1.position.y, pose_2.position.x-pose_1.position.x));
    pose_2.orientation = tf::createQuaternionMsgFromYaw(atan2f(pose_1.position.y-pose_2.position.y, pose_1.position.x-pose_2.position.x));
    ros::Subscriber location_subscriber = nh.subscribe<nav_msgs::Odometry>("/auto_person_" + 
                      boost::lexical_cast<std::string>(i) + "/odom", 1, boost::bind(&personOdometryHandler, _1, i));
    ros::Publisher command_publisher = nh.advertise<geometry_msgs::Twist>("/auto_person_" + 
                      boost::lexical_cast<std::string>(i) + "/cmd_vel", 1);
    persons.push_back(Person(i,pose_1,pose_2,location_subscriber,command_publisher));
    persons[i].init();
  }
}

