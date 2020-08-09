#include <ros/ros.h>
#include <traversability_estimation/TraversabilityMapFused.hpp>

using namespace traversability_estimation;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traversability_map_fused");
    ros::NodeHandle nh_;

    TraversabilityMapFused fuseMap(nh_);

    ros::spin();

//    ros::Rate rate_(2);

//    while(ros::ok())
//    {
//        ros::spinOnce();

//        ROS_WARN("spinOnce");

//        rate_.sleep();
//    }

    return 0;
}


//GridMap fusedTraversabilityMap_, initialTraversabilityMap_;

//bool GetMap;

//void initialTraversabilityMapCallback(const grid_map_msgs::GridMap &message)
//{
//    ROS_INFO_ONCE("Receive initial traversability map");
//    GridMap gridMap;
//    GridMapRosConverter::fromMessage(message, gridMap);
//    ROS_INFO_ONCE("Created map with size %f x %f m (%i x %i cells).",
//      gridMap.getLength().x(), gridMap.getLength().y(),
//      gridMap.getSize()(0), gridMap.getSize()(1));
//    initialTraversabilityMap_ = gridMap;


//}

//void localTraversabilityMapCallback(const grid_map_msgs::GridMapConstPtr &message)
//{
//    ROS_INFO("Receive local traversability map");
//    GridMap gridMap;
//    GridMapRosConverter::fromMessage(*message, gridMap);
////    if(!gridMap.exists("traversability"))
////        gridMap.add("traversability", 1);

//    ROS_INFO_ONCE("Created map with size %f x %f m (%i x %i cells).",
//      gridMap.getLength().x(), gridMap.getLength().y(),
//      gridMap.getSize()(0), gridMap.getSize()(1));
//    fusedTraversabilityMap_ = gridMap;
//    GetMap = true;
//}

//void fuseMap()
//{
//    while(GetMap)
//    {
//        GridMap gridMapCopy;
//        gridMapCopy = initialTraversabilityMap_;
//        GridMap gridMapFused;
//        gridMapFused = fusedTraversabilityMap_;
//        for(grid_map::GridMapIterator iterator(gridMapCopy); !iterator.isPastEnd(); ++iterator)
//        {
//            Index index;
//            ROS_WARN_ONCE("Starting fusing map");
//            auto& traversability =gridMapFused.at("elevation", index);
//            ROS_WARN_ONCE("Get traversability layer of the fused map");
//            if(!gridMapFused.isValid(index, "traversability")){
//                ROS_WARN_STREAM_ONCE("Started Index: " << std::endl << index << std::endl);
//                traversability = gridMapCopy.at("traversability", index);
//                ROS_WARN_ONCE("Get traversability layer of the initial map");
//            }
//        }
//        // fuse finish
//        fusedTraversabilityMap_ = gridMapFused;
//    }

//}

//int main(int argc, char** argv)
//{
//    ros::init(argc, argv, "traversability_map_fused");
//    ros::NodeHandle nh_;

//    ros::Subscriber localMapSubscriber_ = nh_.subscribe(
//                "/grid_map_simple_demo/grid_map", 100, localTraversabilityMapCallback);
//    ros::Subscriber initialMapSubscriber_ = nh_.subscribe(
//                "/traversability_estimation/traversability_map", 100, initialTraversabilityMapCallback);
//    ros::Publisher fusedMapPublisher_ = nh_.advertise<grid_map_msgs::GridMap>("fused_map", 100);

//    boost::thread server(fuseMap);

//    ros::Rate rate(100);


//    while(ros::ok())
//    {
//        ros::spinOnce();

//        grid_map_msgs::GridMap message;
//        GridMapRosConverter::toMessage(fusedTraversabilityMap_, message);
//        fusedMapPublisher_.publish(message);


//        rate.sleep();
//    }

//    return 0;
//}
