#include <traversability_estimation/TraversabilityMapFusion.hpp>

using namespace grid_map;

namespace traversability_estimation{

TraversabilityMapFused::TraversabilityMapFused(ros::NodeHandle& nh)
    :nodehandle_(nh)
{
    //initialTraversabilityMapSubscriber_ = nodehandle_.subscribe(
    //            "/traversability_estimation/traversability_map", 100, &TraversabilityMapFused::initialTraversabilityMapCallback, this);
    localTraversabilityMapSubscriber_ = nodehandle_.subscribe(
                "/grid_map_filter_demo/filtered_map", 1, &TraversabilityMapFused::localTraversabilityMapCallback, this);
//    localTraversabilityMapSubscriber_ = nodehandle_.subscribe(
//                "/grid_map_simple_demo/grid_map", 100, &TraversabilityMapFused::localTraversabilityMapCallback, this);
    fusedTraversabilityMapPublisher_ = nodehandle_.advertise<grid_map_msgs::GridMap>("fused_traversability_map", 100);
    initialTraversabilityMapPublisher_ = nodehandle_.advertise<grid_map_msgs::GridMap>("initial_traversability_map", 100);
    localTraversabilityMapPublisher_ = nodehandle_.advertise<grid_map_msgs::GridMap>("local_traversability_map", 100);

    loadInitialTraversabilityMap();

//    loadLocalTraversabilityMap();
    // Wait subscribe map info.
    sleep(1);
    //fuseMapThread_ = boost::thread(boost::bind(&TraversabilityMapFused::fuseMap, this));
    //ros::Timer timer = nodehandle_.createTimer(ros::Duration(2), boost::bind(&TraversabilityMapFused::fuseMapCallback, this, _1), false);
    ros::TimerOptions control_time_option(ros::Duration(1), // 1 Hz
                                          boost::bind(&TraversabilityMapFused::fuseMapCallback, this, _1),
                                          &callback_queue_, false, false);
    update_timer = nodehandle_.createTimer(control_time_option);

    timer_thread = boost::thread(boost::bind(&TraversabilityMapFused::timerThread, this));

    update_timer.start();

    ROS_INFO("Traversability map fused class constructed");
}

TraversabilityMapFused::~TraversabilityMapFused()
{

}

bool TraversabilityMapFused::loadInitialTraversabilityMap()
{
    ROS_WARN("Loading initial traversability map");
    boost::recursive_mutex::scoped_lock scopedLock(mutex_);

    std::string topic =
            "/traversability_estimation/traversability_map";
    std::string path_file =
            "/home/eric/traversability_map.bag";
    GridMap gridMap;

    GridMapRosConverter::loadFromBag(path_file, topic, gridMap);

    ROS_INFO_ONCE("Created map with size %f x %f m (%i x %i cells).",
      gridMap.getLength().x(), gridMap.getLength().y(),
      gridMap.getSize()(0), gridMap.getSize()(1));

    initialTraversabilityMap_ = gridMap;

    return true;
}

bool TraversabilityMapFused::loadLocalTraversabilityMap()
{
    ROS_WARN("Loading local traversability map");
    boost::recursive_mutex::scoped_lock scopedLock(mutex_);

    std::string topic =
            "/traversability_estimation/traversability_map";
    std::string path_file =
            "/home/eric/local_traversability_map.bag";
    GridMap gridMap;

    GridMapRosConverter::loadFromBag(path_file, topic, gridMap);

    ROS_INFO_ONCE("Created map with size %f x %f m (%i x %i cells).",
      gridMap.getLength().x(), gridMap.getLength().y(),
      gridMap.getSize()(0), gridMap.getSize()(1));

    localTraversabilityMap_ = gridMap;

    return true;
}



void TraversabilityMapFused::initialTraversabilityMapCallback(const grid_map_msgs::GridMap &message)
{
    ROS_INFO_ONCE("Receive local traversability map");
    GridMap gridMap;
    GridMapRosConverter::fromMessage(message, gridMap);

    ROS_INFO_ONCE("Created initial map with size %f x %f m (%i x %i cells).",
      gridMap.getLength().x(), gridMap.getLength().y(),
      gridMap.getSize()(0), gridMap.getSize()(1));

    initialTraversabilityMap_ = gridMap;

}

void TraversabilityMapFused::localTraversabilityMapCallback(const grid_map_msgs::GridMapConstPtr &message)
{
    ROS_INFO_ONCE("Receive local traversability map");
    GridMap gridMap;
    GridMapRosConverter::fromMessage(*message, gridMap);
//    if(!gridMap.exists("traversability"))
//        gridMap.add("traversability", 1);

    ROS_INFO_ONCE("Created local map with size %f x %f m (%i x %i cells).",
      gridMap.getLength().x(), gridMap.getLength().y(),
      gridMap.getSize()(0), gridMap.getSize()(1));

    localTraversabilityMap_ = gridMap;
}

void TraversabilityMapFused::fuseMap()
{
    //* Use local map to update global map
    while(ros::ok())
    {
        GridMap gridMapGlobal;
        gridMapGlobal = initialTraversabilityMap_;
        GridMap gridMapLocal;
        gridMapLocal = localTraversabilityMap_;
        grid_map::Matrix& data = gridMapGlobal["traversability"];
        grid_map::Matrix& dataToAdd = gridMapLocal["traversability"];
        for(grid_map::GridMapIterator iterator(gridMapLocal); !iterator.isPastEnd(); ++iterator)
        {
            const Index index(*iterator);
            //ROS_WARN("Starting fusing map");
            //auto& traversability = gridMapGlobal.at("traversability", index);
            //auto& elevation = gridMapGlobal.at("elevation", index);
            //ROS_WARN("Get traversability layer of the fused map");
            if(!gridMapLocal.isValid(index, "traversability"))
                continue; // ignore cells without data in local map
            //ROS_WARN_STREAM_ONCE("Started Index: " << std::endl << index << std::endl);
            //traversability = gridMapLocal.at("traversability", index);
            data(index(0), index(1)) = dataToAdd(index(0), index(1));
            //elevation = gridMapLocal.at("elevation", index);
            //ROS_WARN_ONCE("Get traversability layer of the initial map");
        }
        // fuse finish
        fusedTraversabilityMap_ = gridMapGlobal;
        publishFusedTraversabilityMap();
        publishInitialTraversabilityMap();
        publishLocalTraversabilityMap();
    }

}

void TraversabilityMapFused::fuseMapCallback(const ros::TimerEvent&)
{
    ROS_WARN("spinOnce");
    ros::Time start_time = ros::Time::now();
    boost::recursive_mutex::scoped_lock lock(mutex_);
    GridMap gridMapGlobal;
    gridMapGlobal = initialTraversabilityMap_;
    GridMap gridMapLocal;
    gridMapLocal = localTraversabilityMap_;
    grid_map::Matrix& data1= gridMapGlobal["elevation"];
    grid_map::Matrix& dataToAdd1 = gridMapLocal["elevation"];
    grid_map::Matrix& data2 = gridMapGlobal["traversability"];
    grid_map::Matrix& dataToAdd2 = gridMapLocal["traversability"];
    for(grid_map::GridMapIterator iterator(gridMapLocal); !iterator.isPastEnd(); ++iterator)
    {
        const Index index(*iterator);
        if(!gridMapLocal.isValid(index, "elevation"))
            continue; // ignore cells without data in local map
        data1(index(0), index(1)) = dataToAdd1(index(0), index(1));
        data2(index(0), index(1)) = dataToAdd2(index(0), index(1));
    }
    // fuse finish
    fusedTraversabilityMap_ = gridMapGlobal;
    lock.unlock();
    publishFusedTraversabilityMap();
    publishInitialTraversabilityMap();
    publishLocalTraversabilityMap();
    ros::Time end_time = ros::Time::now();
    ros::Duration duration_ = (end_time - start_time);
    ROS_WARN_STREAM("Duration time for spin once: " << duration_ << " s;" << std::endl);
}

void TraversabilityMapFused::timerThread()
{
    static const double timeout = 1;
    while(nodehandle_.ok())
    {
        callback_queue_.callAvailable(ros::WallDuration(timeout));
    }
}

void TraversabilityMapFused::publishFusedTraversabilityMap()
{
    grid_map_msgs::GridMap message;
    GridMap gridMap;
    gridMap = fusedTraversabilityMap_;
    GridMapRosConverter::toMessage(gridMap, message);
    fusedTraversabilityMapPublisher_.publish(message);
}

void TraversabilityMapFused::publishInitialTraversabilityMap()
{
    grid_map_msgs::GridMap message;
    GridMap gridMap;
    gridMap = initialTraversabilityMap_;
    GridMapRosConverter::toMessage(gridMap, message);
    initialTraversabilityMapPublisher_.publish(message);
}

void TraversabilityMapFused::publishLocalTraversabilityMap()
{
    grid_map_msgs::GridMap message;
    GridMap gridMap;
    gridMap = localTraversabilityMap_;
    GridMapRosConverter::toMessage(gridMap, message);
    localTraversabilityMapPublisher_.publish(message);
}

}

