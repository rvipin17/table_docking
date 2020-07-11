# Table Docking with Node Diagnostics
1) The first stage begins with an omni-directional robot parked in front of a table. The four legs of the table are visible to the Lidar sensor at the centre of the robot.
2) In the second stage, the robot will then find its way to go underneath the table, and it will align and center itself within 4 legs of the table as shown in the picture attached. Additionally the robot will response to the movement of the table accordingly.
3) Robot will stop moving if the node being monitored is killed or no longer exists and once node is restarted, robot will start docking. 

### Prerequisites

1) Install ROS Kinetic with ubuntu 16.04.

### Instructions

1) create a workspace <br />
$ mkdir -p ~/catkin_ws/src <br />
$ cd ~/catkin_ws/src <br />
$ catkin_init_workspace <br />
$ cd .. <br />
$ catkin_make <br />
   
2) Copy the project to src <br />
$ cd ~/catkin_ws/src <br />
$ cd .. <br />
$ rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
$ catkin_make <br />
$ source ~/catkin_ws/devel/setup.bash <br />
$ roslaunch table_docking table_docking.launch <br />

3) Now you will see robot getting inside the robot. Please change the table position, the robot will respond accordingly. 

4) Kill robot_state_publisher node, robot will stop moving
$ rosnode kill /robot_state_publisher

5) Restart robot_state_publisher node, then robot will start docking
$ rosrun robot_state_publisher robot_state_publisher

## Diagnostics Monitoring

A monitoring system for tracking the node alive status and report error under /monitoring topic. 

### Node monitor

A Node monitor that pings every node from the list of monitoring nodes (from the parameter file). 

**Monitored Values:**

|     values      | unit  |          Comment                   |
|-----------------|-------|------------------------------------|
|     isAvailable    |       |         True, if the node is registered to the master                          |

**Warning/Errors:**

* Error if, 
     * Node is not available

**Parameters:**


	frequency: 1					# Frequency at which diagnostics updates node errors
	filter_type: 1                                  # 0 (default: monitors all nodes registered to ros) or 
                                                      1 (only monitor nodes provided in the nodes param) or
                                                      2 (monitor all registered nodes except for the nodes in the param)
    nodes: [node_name]                              # List of nodes to be monitored or blacklisted based on filter type

## Rubric Points

- The Project uses Object Oriented Programming techniques. The classes TableDock, NodeMonitor and PingMonitor are declared in table_docking.h, node_monitor.h and ping_monitor.h respectively, and defined in table_docking.cpp, node_monitor.cpp and ping_monitor.cpp respectively. 
- In table_docking.h, node_monitor.h and ping_monitor.h, appropriate access specifiers are properly defined for class members.
- In the respective constructors in node_monitor.cpp and ping_monitor.cpp, the constructors are initialised with initialisation list.
- All the diagnostics related features are implemented in NodeMonitor and PingMonitor, and docking related implementations are in TableDocking, hense encapsulated in their respective classes. 
- All ros callbacks uses pass by reference.
- In table_docking.h, at line no. 43, node monitor object is created using shared_ptr and also in node_monitor and ping_monitor, appropriate shared_ptr is used to create instance of a class.
- In node_monitor.h (line no. 53) and in node_monitor.cpp (line no. 59), individual node monitors are created using unique_ptr.
- In node_monitor.cpp (line no. 60), node monitors are moved using std::move to the ping_monitor list. 

