#ifndef NODE_MONITOR_H_
#define NODE_MONITOR_H_

// ROS Headers
#include "ros/ros.h"
#include "ros/console.h"

// System Libraries
#include "memory"

// Other Libraries
#include "monitoring_core/monitor.h"
#include "table_docking/ping_monitor.h"

// Enums
enum FilterType
{
    DEFAULT,
    LIMITED,
    REMOVE
};

/**
 * @class NodeMonitor
 * @brief A Class to monitor the list of node's ping status
 */

class NodeMonitor
{

public:
    /**
     * @brief Default constructor, reads required params from parameter file 
     */
    NodeMonitor(ros::NodeHandle nh);

    /**
     * @brief Default destructor, deletes all pointers and objects
     */
    ~NodeMonitor();

private:
    ros::NodeHandle nh_; ///< ROS Node Handler

    std::vector<std::string> node_list_; ///< Stores the list of topics

    double frequency_; ///< Frequency at which node monitor publishes monitor status

    int mode_; ///< Setting mode will filter node names to be monitored, for intance, default mode will capture status of all the nodes registered with master

    std::shared_ptr<Monitor> monitor_; ///< A monitor class object for publishing ping status of each node

    std::vector<std::unique_ptr<PingMonitor>> ping_monitor_list_; ///< Stores list of statistic monitor instances for monitoring ping rate of each node

    /**
     * @brief Function to update node statistics
     */
    void updateNodeListAndStatistics();
};

#endif /*NODE_MONITOR_H_*/