#include "table_docking/table_docking.h"

int main(int argc, char **argv)
{

    /* Initialise the ROS Node */
    ros::init(argc, argv, "table_docking", ros::init_options::AnonymousName);

    /* ROS Node Handler */
    ros::NodeHandle nh;

    /* Initialise the constructor */
    std::shared_ptr<TableDock> dock = std::make_shared<TableDock>(nh);

    ros::spin();

    return 0;
}