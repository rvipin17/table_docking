#ifndef TABLE_DOCKING_H_
#define TABLE_DOCKING_H_

// ROS Headers
#include "ros/ros.h"
#include "ros/console.h"

// System Headers
#include <memory>

// Other Libraries
#include <stdio.h>
#include <math.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "table_docking/node_monitor.h"

/**
 * @class TableDock
 * @brief A table docking algorithm for omni directional robot, with integrated node diagnostics
 */

class TableDock
{

public:
    /**
     * @brief Default constructor, reads required params from parameter file 
     */
    TableDock(ros::NodeHandle &nh);

    /**
     * @brief Default destructor, deletes all pointers and objects
     */
    ~TableDock();

private:
    ros::NodeHandle nh_; ///< ROS Node Handler

    std::shared_ptr<NodeMonitor> node_monitor_; ///< Node monitor object which monitors the liveliness of node

    geometry_msgs::Twist vel_; ///< Velocity command

    sensor_msgs::LaserScan laser_; ///< Laser feedback

    monitoring_msgs::MonitoringArray monitor_; ///< A diagnostics error message reposted by node monitor

    ros::Publisher pub_; ///< A velocity publisher

    ros::Subscriber odom_sub_; ///< Robot odometry subscriber

    ros::Subscriber laser_sub_; ///< Robot laser feedback subscriber

    ros::Subscriber monitor_sub_; ///< Robot node diagnostics subscriber

    float robot_x_{0.0}; ///< Robot position in x direction

    float robot_y_{0.0}; ///< Robot position in y direction
    
    /**
     * @brief A ros callback to receive odometry message from robot
     * 
     * @param msg Odometry feedback
     */
    void OdomCallback(const nav_msgs::OdometryConstPtr &msg);

    /**
     * @brief A ros callback for laser sensor feedback
     * 
     * @param msg laser sensor feedback
     */
    void LaserCallback(const sensor_msgs::LaserScanConstPtr &msg);

    /**
     * @brief A ros callback for diagnostics monitor of node
     * 
     * @param msg Node diagnostics message
     */
    void MonitorCallback(const monitoring_msgs::MonitoringArrayConstPtr &msg);
};

#endif /*TABLE_DOCKING_H_*/