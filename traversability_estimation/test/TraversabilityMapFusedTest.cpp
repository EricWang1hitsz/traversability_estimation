#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>


using namespace grid_map;

int main(int argc, char** argv)
{
    // Initialize node and publisher.
    ros::init(argc, argv, "grid_map_simple_demo");
    ros::NodeHandle nh("~");
    ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    ros::Publisher publisher_ = nh.advertise<grid_map_msgs::GridMap>("fused_map", 1, true);

    GridMap map({"traversability", "elevation"});
    map.setFrameId("odom");
    map.setGeometry(Length(3.0, 3.0), 0.03);
    GridMap fusedMap({"traversability", "elevation"});
    fusedMap.setFrameId("odom");
    fusedMap.setGeometry(Length(3.0, 3.0), 0.03);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
      map.getLength().x(), map.getLength().y(),
      map.getSize()(0), map.getSize()(1));
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
      fusedMap.getLength().x(), fusedMap.getLength().y(),
      fusedMap.getSize()(0), fusedMap.getSize()(1));
    //Setting a constant value to all cells of a leyer.
    map["traversability"].setConstant(1.0);
    //map["elevation"].setConstant(0.0);
    //fusedMap["elevation"].setConstant(0.0);
    Index submapStartIndex(0, 0);
    Index submapBufferSize(20, 20);

    for (grid_map::SubmapIterator iterator(fusedMap, submapStartIndex, submapBufferSize);
        !iterator.isPastEnd(); ++iterator) {
      fusedMap.at("traversability", *iterator) = 0.5;
    }

    for(grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
    {
        //Position position;
        //map.getPosition(*iterator, position);
        Index index = *iterator;
        auto& traversability = fusedMap.at("traversability", index);
        // Attention: set layer or set basic layer
        if(!fusedMap.isValid(index, "traversability"))
        // No prior information before, fuse.
        {
            ROS_WARN_STREAM_ONCE("Index" << index << std::endl);
            traversability = map.at("traversability", index);
        }
    }
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    grid_map_msgs::GridMap messgaeFused;
    GridMapRosConverter::toMessage(fusedMap, messgaeFused);

    ros::Rate rate(100);

    while(ros::ok())
    {
        ros::Time time = ros::Time::now();
        map.setTimestamp(time.toNSec());
        fusedMap.setTimestamp(time.toNSec());

        publisher.publish(message);
        publisher_.publish(messgaeFused);

        rate.sleep();
    }

    return 0;


}
