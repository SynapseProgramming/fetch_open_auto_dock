#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <actionlib/client/simple_action_client.h>

#include <fetch_open_auto_dock/DockAction.h>
#include <fetch_open_auto_dock/UndockAction.h>
#include <move_base_msgs/MoveBaseAction.h>

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
goal.dock_pose.pose.position.x = 1.0; //TODO maybe make this dynamically reconfigurable
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
std::cout <<"Successfully Undocked!"<<std::endl;
docked=false;
}
else{
std::cout<<"Unsuccessfully Docked!"<<std::endl;
docked=false;
}
  }

//method returns the current dock status. true being docked, false being undocked.
bool get_dock_status(){
return docked;
}

private:
  dock_client_type dock_client;
  undock_client_type undock_client;
  bool docked;

};

class move_base_controller{

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> movebAC;
public:
move_base_controller():
ac("move_base",true)
{
g_reached=false;
std::cout<<"move base controller initialised!\nWaiting for move_base server to start!"<<std::endl;
ac.waitForServer();
ROS_INFO("move base server is online!");
}

void movebase_send_goal(double x, double y, double z,double w){
  //create the move base goal variable.
  move_base_msgs::MoveBaseGoal goal;
  //populate the goal with data.
  goal.target_pose.header.frame_id="map";
  goal.target_pose.header.stamp=ros::Time::now();

  goal.target_pose.pose.position.x=x;
  goal.target_pose.pose.position.y=y;
  goal.target_pose.pose.orientation.w=w;
  goal.target_pose.pose.orientation.z=z;

  //send the goal to move base!
  ac.sendGoal(goal,boost::bind(&move_base_controller::donecb, this, _1, _2),movebAC::SimpleActiveCallback(), movebAC::SimpleFeedbackCallback());
  g_reached=false;
}//bracket of movebase_send_goal

void donecb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result){
//Print out the current state.
std::cout<<"Status:"<<state.toString()<<std::endl;
//Statements to print upon sucessfully reaching the goal.
if(state.toString()=="SUCCEEDED"){
std::cout<<"The goal has been reached!"<<std::endl;
//declare goal reached as true.
g_reached=true;
}
else{g_reached=false;
std::cout<<"Something went wrong! the bot could not reach the near goal!"<<std::endl;
}

}//bracket of donecb

bool get_goal_state(){
return g_reached;
}

private:
//create the action client object,specifying the name of the action server.
movebAC ac;
bool g_reached;

};



class master_controller{
public:
master_controller():
sub(n.subscribe<std_msgs::Float32>("battery_voltage",5,&master_controller::updatevoltage,this))
{
current_voltage=30; //will start off with a high initial charge.
low_battery_threshold=23; //TODO add yaml reconfigure options for these variables later.
upper_battery_threshold=25;
charged=true;
stage=0;
ROS_INFO("master controller initalized.");
check_logic();
}

void updatevoltage(const std_msgs::Float32::ConstPtr &msg){
current_voltage=msg->data;
if(current_voltage>=upper_battery_threshold){charged=true;}
else if(current_voltage<low_battery_threshold){charged=false;}
//print_current_voltage();

}

//print function mostly used for debugging.
void print_current_voltage(){
  std::cout<<"The current battery voltage is: "<<current_voltage<<std::endl;
  if(charged==true){std::cout<<"charge is true"<<std::endl;}
  else{std::cout<<"charge is false."<<std::endl;}
}

//main control and logic found here
void check_logic(){
  ros::Rate loop_rate(60);
  while(ros::ok()){
    //if the robot is running around and has not docked, and its battery is low, we will move to the near goal first.
    if(charged==false&&dock_undock_bot.get_dock_status()==false&&stage==0){
    ROS_INFO("battery is low. sending robot to near goal for docking!");
    move_base.movebase_send_goal(-0.159,-0.700,-0.700,0.714);
    stage=1;
    }
    // if the robot has reached the near goal, wait for 1 sec for robot to fully stop. Then send in the robot to dock.
    else if(stage==1&&move_base.get_goal_state()==true){
    ros::Duration(1.0).sleep();
    dock_undock_bot.dock_robot();
    stage=2;
    }
    //around this point the robot should begin to start charging its batteries. will wait here till the voltage has reached the upper threshold. hehe.
    //once the robot has docked and charged up its batteries to the upper threshold, then we will undock the robot.
    else if(stage==2&&dock_undock_bot.get_dock_status()==true&&charged==true){
    dock_undock_bot.undock_robot();
    ROS_INFO("docking sequence completed. Awaiting low battery to repeat.");
    stage=0; //reset stage so we an rinse and repeat.
    }




    loop_rate.sleep();
    ros::spinOnce();
    }

}

private:


move_base_controller move_base;
docking_undocking_interface dock_undock_bot;

ros::NodeHandle n;
ros::Subscriber sub;

double current_voltage;
double low_battery_threshold;
double upper_battery_threshold;
bool charged; // charge status of battery. true=charged false=low charge
int stage;
};


int main(int argc, char **argv){
ros::init(argc,argv,"master_docking_controller");

master_controller object;

return 0;
}
