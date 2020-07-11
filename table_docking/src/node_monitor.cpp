#include "table_docking/node_monitor.h"

NodeMonitor::NodeMonitor(ros::NodeHandle nh) : nh_(nh)
{

    ROS_INFO("Reading Parameters.");

    /* Read all the parameters required for node monitor */

    nh_.param("/filter_type", mode_, int(FilterType::DEFAULT));

    if (!nh_.getParam("/nodes", node_list_) && mode_ != FilterType::DEFAULT)
    {
        node_list_.push_back("/rosout");
    }

    nh_.param("/frequency", frequency_, 1.0);

    monitor_ = std::make_shared<Monitor>(nh_, "Node Ping Monitor", frequency_);

    updateNodeListAndStatistics();
}

NodeMonitor::~NodeMonitor()
{
}

void NodeMonitor::updateNodeListAndStatistics()
{

    std::vector<std::string> node_array_temp;
    ros::master::getNodes(node_array_temp);
    std::unique_ptr<PingMonitor> ping_monitor;
    std::string topic_name;

    /* Check for namespace */
    std::string ns = ros::this_node::getNamespace();
    ns = (ns != "/") ? ns.substr(1, ns.size() - 1) + "/" : std::string();

    /* If mode is default, then take all the nodes registered with master */
    if (mode_ == FilterType::DEFAULT)
    {
        node_list_ = node_array_temp;
    }

    /* If mode is REMOVE, then remove all the nodes from the list of nodes registered with master and monitor the remaining nodes */
    else if (mode_ == FilterType::REMOVE)
    {
        for (std::vector<std::string>::iterator it(node_list_.begin()); it != node_list_.end(); ++it)
        {
            node_array_temp.erase(std::remove(begin(node_array_temp), end(node_array_temp), *it), end(node_array_temp));
        }
        node_list_ = node_array_temp;
    }

    /* For each node, call a statistics monitor object to monitor */
    for (std::vector<std::string>::iterator i(node_list_.begin()); i != node_list_.end(); ++i)
    {
        ping_monitor = std::make_unique<PingMonitor>(nh_, ns + *i, frequency_, monitor_);
        ping_monitor_list_.push_back(std::move(ping_monitor));
    }
}