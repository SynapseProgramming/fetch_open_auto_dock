# Lidar Based Auto Docking

General autodocking package for ROS based robots. hehe. :)
Note: your dock must have the same cross sectional dimensions as the fetch charging dock.stl file.  Front face is 300mm long, Each side is 100mm long, at 45 degree angle

# Setup/Installation
 * Firstly, clone this package into your catkin/src folder using git clone. 
 * Next, use catkin_make to compile the package.
 * Once the package has been compiled, open auto_dock.launch(which is located in the launch folder) and modify the remap statements as neccessary. 
 * **to="scan" (scan could be replaced by the name of your laserscan topic ie. laser_scan)**
 * **to="cmd_vel" (cmd_vel could be replaced by a custom name for sending velocity commands ie. dock_cmd_vel)**
 * Next, in the params folder, open *docking_parameters.yaml* and tune the parameters as neccessary. Detailed descriptions of the    parameters can be found in the same document.
 * The docking program subscribes to the topic **battery_voltage** of type **std_msgs/Float32** to obtain the current batteries    charge state in (Volts). Do ensure you have a source to publish to this topic.
 
 * To run the autodock program, type **roslaunch fetch_open_auto_dock auto_dock.launch**
 
# General Testing
* To test the docking program without having to publish to **battery_voltage** and set the far goal parameters, drive/place the robot about 1 metre away from the dock with the robot pointing towards it and run the following lines:

* **roslaunch fetch_open_auto_dock auto_dock.launch**

 * Followed by **rosrun fetch_open_auto_dock dock_on_button.py**
 
* To test the undocking program by itself **(warning: robot will reverse and spin 180 degrees)** run the following lines:

* **roslaunch fetch_open_auto_dock auto_dock.launch**
 
* **rosrun fetch_open_auto_dock undock_on_button.py**

# General Notes
* It is recommended to place the near goal about 1-1.5 metres away from the dock and have the robot point towards the dock. 
* Obstacle avoidance is not implemented here. Ensure dock area is clear before docking robot. 
