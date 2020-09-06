#include <iostream>

#include <fetch_auto_dock/perception.h>
// STL Includes.
#include <math.h>
#include <algorithm>
// ROS Includes.
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class dock_estimator{
public:
  dock_estimator():
  find_dock_rate(50),
  perception_(n_)
  {
    dock_pose.header.frame_id = "base_link";
    perception_.start(dock_pose);
    print_estimate();

  }

  void print_estimate(){
    ROS_INFO("Finding Dock!");

    while (!perception_.getPose(wanted_dock_pose, "base_link")&&ros::ok()){
        //reset internal pose to enable continuous checking
        perception_.start(dock_pose);
        ROS_INFO("Still Finding Dock! Please adjust robot till this message disappears");
        find_dock_rate.sleep();
        ros::spinOnce();
    }
    ROS_INFO("FOUND DOCK!!!");
    while(ros::ok()){
    perception_.getPose(wanted_dock_pose, "map");
    std::cout<<"wrt map ";
    std::cout<<"x: "<<wanted_dock_pose.pose.position.x<<" y: "<<wanted_dock_pose.pose.position.y;
    std::cout<<" z: "<<wanted_dock_pose.pose.orientation.z<<" w: "<<wanted_dock_pose.pose.orientation.w<<"\n";

      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = "map";
      transformStamped.child_frame_id = "estimated_dock_frame";
      transformStamped.transform.translation.x = wanted_dock_pose.pose.position.x;
      transformStamped.transform.translation.y = wanted_dock_pose.pose.position.y;
      transformStamped.transform.translation.z = 0.0;
      transformStamped.transform.rotation.x = 0.0;
      transformStamped.transform.rotation.y = 0.0;
      transformStamped.transform.rotation.z = wanted_dock_pose.pose.orientation.z;
      transformStamped.transform.rotation.w = wanted_dock_pose.pose.orientation.w;
      br.sendTransform(transformStamped);

    find_dock_rate.sleep();
    ros::spinOnce();


    }


  }

  //destructor
  ~dock_estimator(){
    perception_.stop();

    std::cout<<"FINAL MAP TO DOCK TRANSFORM\n";
    std::cout<<"x: "<<wanted_dock_pose.pose.position.x<<" y: "<<wanted_dock_pose.pose.position.y;
    std::cout<<" z: "<<wanted_dock_pose.pose.orientation.z<<" w: "<<wanted_dock_pose.pose.orientation.w<<"\n";

  }


private:
  ros::NodeHandle n_;
  ros::Rate find_dock_rate;

  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  tf2::Quaternion q;

  DockPerception perception_;  // Used to detect dock pose.
  geometry_msgs::PoseStamped dock_pose; //initial dock pose
  geometry_msgs::PoseStamped wanted_dock_pose;



};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "dock_estimator");
  dock_estimator estimate;
  ros::spin();

  return 0;
}
