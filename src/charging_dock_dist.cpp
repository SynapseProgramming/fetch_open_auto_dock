#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <iostream>

ros::Publisher pub;

void lasercb(const sensor_msgs::LaserScan::ConstPtr &msg){
double sum=0;
double avg=0;

//calculate the average value of data points between +-5 degrees of lidar heading
for(unsigned int i=390; i<=420;i++){
if(std::isnan(msg->ranges[i])){continue;}
sum+=(msg->ranges[i]);
}
avg=sum/30.0;
std::cout<<"The average distance in metres is :"<<avg<<std::endl;


std_msgs::Float32 dock_distance;
dock_distance.data=avg;
//publish the average heading distance.
pub.publish(dock_distance);

}


int main(int argc, char**argv){

//run the ros init function.
ros::init(argc,argv,"chargindock_dist_measure");
//run the ros nodehandle.
ros::NodeHandle n;
//subscribe to the lidar /scan topic
ros::Subscriber sub=n.subscribe("scan",20,lasercb);
//publish the average heading distance on the topic of dock dist
pub=n.advertise<std_msgs::Float32>("dock_dist",20);


ros::spin();


//std::cin.get();
return 0;
}
