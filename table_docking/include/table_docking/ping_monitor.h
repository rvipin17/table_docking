#ifndef PING_MONITOR_H_
#define PING_MONITOR_H_

// ROS Headers
#include "time.h"
#include "ros/ros.h"
#include "ros/console.h"

// System Headers
#include "fstream"
#include "sstream"
#include "memory"

// Other Libraries
#include "monitoring_core/monitor.h"

/**
 * @class PingMonitor
 * @brief A Class for monitoring a node's ping status
 */

class PingMonitor
{

public:
    /**
     * @brief Default constructor, calls the timer function to update ping status of node
     * @param nh A nodehandler
     * @param node_name Name of the node to be pinged
     * @param frequency Frequency of publishing node ping rate
     * @param monitor A monitor object for publishing node ping rate
     */
    PingMonitor(ros::NodeHandle &nh, std::string node_name, double frequency, std::shared_ptr<Monitor> monitor);

    /**
     * @brief Default destructor, deletes all pointers and objects
     */
    ~PingMonitor();

private:
    std::string node_name_; ///< Name of the node to be monitored

    std::string node_pid_; ///< Pid of the node

    double frequency_; ///< frequency of publishing node statistics

    ros::NodeHandle nh_; ///< ROS node handler

    std::shared_ptr<Monitor> monitor_; ///< An object to monitor class

    bool isPublish_; ///< Parameter to check whether to publish the ping rate over monitoring topic

    ros::Timer node_ping_updater_; ///< A timer object to update ping rate of the node

    /**
     * @brief Function to check if a node is still registered to the master
     * @param e A timer event
     */
    void getPingStatus(const ros::TimerEvent &e);

    /**
     * @brief Function to get ping rate of a node and publish 
     * @param key Analyser name for the monitoring message
     */
    void getPingInfo(std::string key);

    /**
     * @brief Function to obtain Xmlrpc URI of the node
     */
    std::string getNodeXmlrpcURI();
};

#endif /*PING_MONITOR_H_*/