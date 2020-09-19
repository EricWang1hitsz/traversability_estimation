#include <ros/ros.h>
#include "traversability_estimation/TraversabilityEstimationRT.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "traversability_estimation_rt_node");
  ros::NodeHandle nodeHandle("~");
  traversability_estimation::TraversabilityEstimationRT traversabilityEstimationRT_(nodeHandle);

  // Spin
  ros::AsyncSpinner spinner(0);  // Use n threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
