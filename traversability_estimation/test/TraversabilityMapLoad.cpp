#include <grid_map_ros/grid_map_ros.hpp>

using namespace grid_map;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "loading_traversability_map");
    ros::NodeHandle nh_;

    ros::Publisher traversabilityMapPublisher_ = nh_.advertise<grid_map_msgs::GridMap>("/traversability_estimation/traversability_map", 1);

    std::string topic =
            "/traversability_estimation/traversability_map";
    std::string path_file =
            "/home/eric/Documents/ElevationMap/bestTraversabilityMap.bag";
    GridMap gridMap;
    grid_map_msgs::GridMap message_;
    ROS_INFO("test");
    GridMapRosConverter::loadFromBag(path_file, topic, gridMap);

    ROS_INFO_ONCE("Created map with size %f x %f m (%i x %i cells).",
      gridMap.getLength().x(), gridMap.getLength().y(),
      gridMap.getSize()(0), gridMap.getSize()(1));

    GridMapRosConverter::toMessage(gridMap, message_);

    ros::Rate loop_rate(1);

    while(ros::ok())
    {
        traversabilityMapPublisher_.publish(message_);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;

}
