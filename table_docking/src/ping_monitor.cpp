#include "table_docking/ping_monitor.h"

PingMonitor::PingMonitor(ros::NodeHandle &nh, std::string node_name, double frequency, std::shared_ptr<Monitor> monitor) : nh_(nh), node_name_(node_name), frequency_(frequency), monitor_(monitor)
{
    /* Read isPublish param to check whether to publish ping status over monitoring topic */
    nh_.param("/publish_ping_rate", isPublish_, false);

    /* Call a timer function to update ping rate */
    node_ping_updater_ = nh_.createTimer(ros::Duration(1 / frequency_), &PingMonitor::getPingStatus, this);
}

PingMonitor::~PingMonitor()
{
}

void PingMonitor::getPingStatus(const ros::TimerEvent &e)
{
    try
    {
        /* Check if node is registered to the master */
        std::string key = node_name_ + "/ping";
        std::string value;
        std::vector<std::string> node_list = {};
        ros::master::getNodes(node_list);
        bool isAvailable = false;
        for (auto node : node_list)
        {
            if (node == node_name_)
            {
                isAvailable = true;
            }
        }
        if (!isAvailable)
        {
            /* If node is not registered with the master, update node is unavailable */
            value = node_name_ + " is unavailable";
            monitor_->addValue(key, value, "ms", 0.9, AggregationStrategies::FIRST);
        }
        else
        {
            if (isPublish_)
            {
                /* If isPublish_ is true, it will publish ping messages */
                getPingInfo(key);
            }
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

void PingMonitor::getPingInfo(std::string key)
{
    /* Getting Node xmlrpc URI, required for obtaining ping rate of the node */
    std::string xmlrpcURI = getNodeXmlrpcURI();
    std::string line;
    char buffer[BUFSIZ];
    std::string chars = "\t\n";
    if (xmlrpcURI != "")
    {
        std::string cmd = "rosnode ping -a | grep " + xmlrpcURI;
        FILE *data = popen(cmd.c_str(), "r");
        if (!feof(data))
        {
            if (fgets(buffer, 128, data) != NULL)
            {
                line += buffer;
            }
            /* Removing unwanted characters from the string */
            for (char c : chars)
                line.erase(std::remove(line.begin(), line.end(), c), line.end());
            monitor_->addValue(key, line, "ms", 0.0, AggregationStrategies::FIRST);
        }
    }
}

std::string PingMonitor::getNodeXmlrpcURI()
{
    /* Fetching xmlrpc URI of node using rosnode command and return xmlrpc uri */
    std::string cmd = "rosnode list -a | grep " + node_name_;
    std::string line;
    char buffer[BUFSIZ];
    std::string value;

    FILE *data = popen(cmd.c_str(), "r");
    if (!feof(data))
    {
        if (fgets(buffer, 128, data) != NULL)
        {
            line += buffer;
        }
        if (line.find(node_name_))
        {
            for (char c : line)
            {
                if (c != ' ')
                {
                    value += c;
                }
                else
                {
                    break;
                }
            }
        }
        return value;
    }

    return "";
}