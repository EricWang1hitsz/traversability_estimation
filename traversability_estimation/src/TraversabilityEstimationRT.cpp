#include <traversability_estimation/TraversabilityEstimationRT.hpp>

namespace traversability_estimation {

TraversabilityEstimationRT::TraversabilityEstimationRT(ros::NodeHandle& nodeHandle)
    : nodehandle_(nodeHandle),
      traversabilityMapRT_(nodeHandle),
      traversabilityType_("traversability"),
      slopeType_("traversability_slope"),
      stepType_("traversability_step"),
      roughnessType_("traversability_roughness"),
      robotSlopeType_("robot_slope") {
    // Initilize map layers.
    elevationMapLayers_.push_back("elevation");
    //elevationMapLayers_.push_back("variance");
    //elevationMapLayers_.push_back("horizontal_variance_x");
    //elevationMapLayers_.push_back("horizontal_variance_y");
    //elevationMapLayers_.push_back("horizontal_variance_xy");
    //elevationMapLayers_.push_back("time");
    // Topics and servicers.
    elevationMapSubscriber_ = nodehandle_.subscribe("/elevation_mapping/elevation_map", 100,
                                                    &TraversabilityEstimationRT::elevationMapToTraversabilityMapCallback, this);
    ROS_INFO("Real-Time traversability estimation node initilized. ");
}


void TraversabilityEstimationRT::elevationMapToTraversabilityMapCallback(const grid_map_msgs::GridMap &message)
{
    ROS_INFO_ONCE("Get elevation map fot traversability estimation. ");
    grid_map::GridMap gridMap;
    grid_map::GridMapRosConverter::fromMessage(message, gridMap);

    //check map layers.
    grid_map::GridMap mapWithCheckedLayers = gridMap;
    for (const auto& layer : elevationMapLayers_) {
      if (!mapWithCheckedLayers.exists(layer)) {
        mapWithCheckedLayers.add(layer, 0.0);
        ROS_INFO_STREAM_ONCE("[TraversabilityEstimation::initializeTraversabilityMapFromGridMap]: Added layer '" << layer << "'.");
      }
    }
    grid_map_msgs::GridMap messageWithCheckedLayers;
    grid_map::GridMapRosConverter::toMessage(mapWithCheckedLayers, messageWithCheckedLayers);
    // Call traversability map functions.
    traversabilityMapRT_.setElevationMap(messageWithCheckedLayers);
    if(!traversabilityMapRT_.computeTraversability()) {
        ROS_WARN("Computer traversability failed. ");
    }
}

}
