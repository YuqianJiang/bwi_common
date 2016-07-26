#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

#include <bwi_msgs/RobotTeleporterInterface.h>

#include <boost/filesystem.hpp>
#include <fstream> 

#include <ros/ros.h>
#include <ros/package.h>

sensor_msgs::ImageConstPtr image;
std::string image_dir = ros::package::getPath("collect_gazebo_data") + "/images/";
std::string pose_dir = ros::package::getPath("collect_gazebo_data") + "/poses/";

void callback_image_saver(const sensor_msgs::ImageConstPtr& msg) {
    image = msg;
} 

int main(int argc, char *argv[]) {
  ros::init (argc, argv, "collect_images");

  ros::NodeHandle nh, private_nh("~");

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/nav_kinect/rgb/image_raw", 1, callback_image_saver);

  ros::ServiceClient teleport_client = nh.serviceClient<bwi_msgs::RobotTeleporterInterface>("teleport_robot");
  teleport_client.waitForExistence();

  std::vector<geometry_msgs::Pose> points;
  geometry_msgs::Pose pose;
    pose.position.x = -1;
    pose.position.y = 8;
  points.push_back(pose);
  	pose.position.x = -1.2;
    pose.position.y = 8;
  points.push_back(pose);
  	pose.position.x = -1.2;
    pose.position.y = 8.2;
  points.push_back(pose);

  ros::Rate rate(10); 
  rate.sleep();

  //int count = 0;
  for (int i=0; (ros::ok()) && (i<points.size()); ++i) {
  	for (int angle=0; angle < 360; angle+=10) {
  		bwi_msgs::RobotTeleporterInterface teleport_srv;
  		teleport_srv.request.pose = points[i];
  		teleport_srv.request.pose.orientation = tf::createQuaternionMsgFromYaw(angles::from_degrees(angle));
  		if (teleport_client.call(teleport_srv)) {
		    if (! teleport_srv.response.success) {
		      ROS_WARN_STREAM("Received error message while teleporting robot.");
		    }
		  } else {
		    ROS_ERROR_STREAM("Unable to teleport due to service request failure.");
		  }

		rate.sleep(); 
  		ros::spinOnce();

  		cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
        if (! boost::filesystem::is_directory(image_dir)) {
            boost::filesystem::path tmp_path(image_dir);
            boost::filesystem::create_directory(tmp_path);
        }

        std::stringstream image_path;
        //image_path << image_dir << count << ".png";
        image_path << image_dir << "pose_" << points[i].position.x << "_" << points[i].position.y << "_" << angle << ".png";
        cv::imwrite(image_path.str(), cv_ptr->image);

        /*if (! boost::filesystem::is_directory(pose_dir)) {
            boost::filesystem::path tmp_path(pose_dir);
            boost::filesystem::create_directory(tmp_path);
        }
        std::stringstream pose_path;
        pose_path << pose_dir << count << ".txt";
        std::ofstream poseFile(pose_path.str().c_str());
        poseFile << points[i].position.x << "," << points[i].position.y << "," << angle;
        poseFile.close();*/

        //count++;
  	}
  }

  
  return 0;
}
