#include "table_docking/table_docking.h"

TableDock::TableDock(ros::NodeHandle &nh) : nh_(nh)
{
    pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 1, &TableDock::OdomCallback, this);
    laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, &TableDock::LaserCallback, this);
	node_monitor_ = std::make_shared<NodeMonitor>(nh_);
	monitor_sub_ = nh_.subscribe<monitoring_msgs::MonitoringArray>("/monitoring", 1, &TableDock::MonitorCallback, this);
}

TableDock::~TableDock()
{
}

void TableDock::OdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    robot_x_ = msg->pose.pose.position.x;
	robot_y_ = msg->pose.pose.position.y;
}

void TableDock::MonitorCallback(const monitoring_msgs::MonitoringArrayConstPtr &msg)
{
	monitor_.info = msg->info;
}

void TableDock::LaserCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
	if (monitor_.info[0].values.empty())
	{
		int size = int((-msg->angle_min + msg->angle_max) / msg->angle_increment);
		float x_[size];
		float y_[size];
		float distance = 0.0;
		float distance_max = 0.0;
		int max_dist_point = 0;
		int point_index = 0;

		for (int i = 0; i < size; i++)
		{
			if ((msg->ranges[i] > msg->range_min) && (msg->ranges[i] < 75.0))
			{
				x_[point_index] = (msg->ranges[i] * cos(msg->angle_min + (i * msg->angle_increment)));
				y_[point_index] = (msg->ranges[i] * sin(msg->angle_min + (i * msg->angle_increment)));
				point_index++;
			}
		}

		for (int j = 0; j < point_index; j++)
		{
			distance = (sqrt(pow((x_[0] - x_[j]), 2) + pow((y_[0] - y_[j]), 2)));
			if (distance > distance_max)
			{
				max_dist_point = j;
				distance_max = distance;
			}
		}

		vel_.linear.y = ((((y_[0] + y_[max_dist_point]) / 2) + robot_y_) - robot_y_);
		vel_.linear.x = ((((x_[0] + x_[max_dist_point]) / 2) + robot_x_) - robot_x_) / 10;

	}
	else
	{
		vel_.linear.x = 0.0;
		vel_.linear.y = 0.0;
		vel_.angular.z = 0.0;
	}
	
	pub_.publish(vel_);

}
