#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <actionlib/client/simple_action_client.h>

#include <fetch_open_auto_dock/DockAction.h>
#include <fetch_open_auto_dock/UndockAction.h>


class docking_undocking_interface{
  typedef actionlib::SimpleActionClient<fetch_open_auto_dock::DockAction> dock_client_type;
  typedef actionlib::SimpleActionClient<fetch_open_auto_dock::UndockAction> undock_client_type;

public:
docking_undocking_interface():
dock_client("dock",true),
undock_client("undock",true)
{
docked=false;
//wait for both servers to be running first, before we can do anything
ROS_INFO("waiting for both dock and undock server to start!");
dock_client.waitForServer();
undock_client.waitForServer();
ROS_INFO("both dock and undock server have started!");
}

//method sends a goal to dock the robot at the charging dock.
void dock_robot(){
//we will create a goal
fetch_open_auto_dock::DockGoal goal;
ROS_INFO("sending robot to dock!");
//populate the dock goal with data.
goal.dock_pose.header.frame_id = "base_link";
goal.dock_pose.pose.position.x = 2.0; //TODO maybe make this dynamically reconfigurable
goal.dock_pose.pose.orientation.x = 0.0;
goal.dock_pose.pose.orientation.y = 0.0;
goal.dock_pose.pose.orientation.z = 0.0;
goal.dock_pose.pose.orientation.w = 0.0;

dock_client.sendGoal(goal, boost::bind(&docking_undocking_interface::dock_result, this, _1, _2),
dock_client_type::SimpleActiveCallback(), dock_client_type::SimpleFeedbackCallback());
}

void dock_result(const actionlib::SimpleClientGoalState& state,const fetch_open_auto_dock::DockResultConstPtr &result){
std::cout<<"The status of docking is: "<< state.toString().c_str() <<std::endl;
if(result->docked==true){
std::cout <<"Successfully Docked!"<<std::endl;
docked=true;
}
else{
std::cout<<"Unsuccessfully Docked!"<<std::endl;
docked=false;
}
  }


void undock_robot(){
if(docked==false){
ROS_WARN("Will not attempt to undock robot as robot has not docked yet.");
return;
}
else{
ROS_INFO("Undocking Robot!");
//we will create a goal
fetch_open_auto_dock::UndockGoal goal;
//we will want the robot to rotate 180 degrees to point away from the dock.
goal.rotate_in_place=true;

//send the goal to the undock server.
undock_client.sendGoal(goal, boost::bind(&docking_undocking_interface::undock_result, this, _1, _2),
undock_client_type::SimpleActiveCallback(), undock_client_type::SimpleFeedbackCallback());
  }
}

void undock_result(const actionlib::SimpleClientGoalState& state,const fetch_open_auto_dock::UndockResultConstPtr &result){
std::cout<<"The status of undocking is: "<< state.toString().c_str() <<std::endl;
if(result->undocked==true){
std::cout <<"Successfully Unocked!"<<std::endl;
docked=false;
}
else{
std::cout<<"Unsuccessfully Docked!"<<std::endl;
docked=false;
}
  }








private:
  dock_client_type dock_client;
  undock_client_type undock_client;
  bool docked;

};




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
int input;
docking_undocking_interface dud;
//master_controller object;
while(ros::ok()){
std::cout<<"please press and enter 1 to send a dock goal to the dock server."<<std::endl;
std::cout<<"please press and enter 2 to send a dock goal to the dock server."<<std::endl;
std::cin >>input;
if(input==1){
dud.dock_robot();
}
else if(input==2){
dud.undock_robot();
}
ros::spinOnce();
}



//ros::spin();
return 0;}
