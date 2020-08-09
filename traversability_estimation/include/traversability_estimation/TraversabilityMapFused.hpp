
#pragma once
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <traversability_estimation/TraversabilityMap.hpp>

using namespace grid_map;
namespace traversability_estimation {


class TraversabilityMapFused
{
public:

    TraversabilityMapFused(ros::NodeHandle& nh);

    ~TraversabilityMapFused();

    bool loadInitialTraversabilityMap();

    bool loadLocalTraversabilityMap();

    void initialTraversabilityMapCallback(const grid_map_msgs::GridMap& message);

    void localTraversabilityMapCallback(const grid_map_msgs::GridMapConstPtr& message);

    void fuseMap();

    void fuseMapCallback(const ros::TimerEvent&);

    void timerThread();

    void publishFusedTraversabilityMap();

    void publishInitialTraversabilityMap();

    void publishLocalTraversabilityMap();

private:

    ros::NodeHandle nodehandle_;

    ros::CallbackQueue callback_queue_;
    //! WHY must initial it as class member
    ros::Timer update_timer;

    ros::Subscriber initialTraversabilityMapSubscriber_;

    ros::Subscriber localTraversabilityMapSubscriber_;

    ros::Publisher fusedTraversabilityMapPublisher_;

    ros::Publisher initialTraversabilityMapPublisher_;

    ros::Publisher localTraversabilityMapPublisher_;

    GridMap initialTraversabilityMap_;

    GridMap localTraversabilityMap_;

    GridMap fusedTraversabilityMap_;

    boost::thread timer_thread;

    boost::recursive_mutex mutex_;

};

}
