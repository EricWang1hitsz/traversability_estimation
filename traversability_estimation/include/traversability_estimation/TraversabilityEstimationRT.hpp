/********************************************************

@File TraversabilityEstimationRT.hpp

@Description Real-time traversability estimation for elevation map.

@Author  Eric Wang

@Date:   2020-09-18

@Association: Harbin Institute of Technology, Shenzhen.

*********************************************************/

#pragma once

#include "traversability_estimation/TraversabilityMap.hpp"
#include "grid_map_msgs/GridMap.h"

namespace  traversability_estimation {

class TraversabilityEstimationRT {
public:

    TraversabilityEstimationRT(ros::NodeHandle& nodehandle);


    void elevationMapToTraversabilityMapCallback(const grid_map_msgs::GridMap& message);

private:

    ros::NodeHandle nodehandle_;
    ros::Subscriber elevationMapSubscriber_;
    TraversabilityMap traversabilityMapRT_;
    //! Traversability map types.
    const std::string traversabilityType_;
    const std::string slopeType_;
    const std::string stepType_;
    const std::string roughnessType_;
    const std::string robotSlopeType_;
    //! Requested elevation map layers.
    std::vector<std::string> elevationMapLayers_;

};

}
