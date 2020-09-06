#include <iostream>

#include <fetch_auto_dock/perception.h>
// STL Includes.
#include <math.h>
#include <algorithm>
// ROS Includes.
#include <ros/ros.h>


class dock_estimator{
public:
  dock_estimator():
  find_dock_rate(50),
  perception_(n_)
  {
    dock_pose.header.frame_id = "base_link";

    print_estimate();
  }

  //method here
  void print_estimate(){

    //something wrong with the start function
    perception_.start(dock_pose);


    ROS_INFO("Finding Dock!");

    while (!perception_.getPose(wanted_dock_pose, "base_link")&&ros::ok()){
        //reset internal pose to enable continuous checking
        perception_.start(dock_pose);
        ROS_INFO("Still Finding Dock! Please adjust robot till this message disapprears");
        find_dock_rate.sleep();
        ros::spinOnce();
    }
    ROS_INFO("FOUND DOCK!!!");
    while(ros::ok()){
    perception_.getPose(wanted_dock_pose, "map");
    std::cout<<"wrt map ";
    std::cout<<"x: "<<wanted_dock_pose.pose.position.x<<" y: "<<wanted_dock_pose.pose.position.y;
    std::cout<<" z: "<<wanted_dock_pose.pose.orientation.z<<" w: "<<wanted_dock_pose.pose.orientation.w<<"\n";
    find_dock_rate.sleep();
    ros::spinOnce();


    }







    perception_.stop();

  }






private:
  ros::NodeHandle n_;
  ros::Rate find_dock_rate;

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
/*
TODO:
-Firstly, be able to detect the dock, and then print out the transform from base link to the dock on the screen.




*/
