#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iostream>



class master_controller{
public:
master_controller():
sub(n.subscribe<std_msgs::Float32>("battery_voltage",5,&master_controller::updatevoltage,this))
{
current_voltage=30;
ROS_INFO("master controller initalized.");
}

void updatevoltage(const std_msgs::Float32::ConstPtr &msg){
current_voltage=msg->data;
print_current_voltage();
}

void print_current_voltage(){
std::cout<<"The current battery voltage is: "<<current_voltage<<std::endl;
}

private:
ros::NodeHandle n;
ros::Subscriber sub;
double current_voltage;

};


int main(int argc, char **argv){
ros::init(argc,argv,"master_docking_controller");

master_controller object;


ros::spin();
return 0;}
