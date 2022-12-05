#include <ros/ros.h>

#include <octomap_filter/octomap_filter.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "octomap_filter_node");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("Initialiasing the octomap_filter_node");
  octomap_filter::OctomapFilter octo_filter(nh);
  ros::spin();

  return 0;
}