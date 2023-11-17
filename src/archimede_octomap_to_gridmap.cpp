/*
 * octomap_to_gridmap_demo_node.cpp
 *
 *  Created on: May 03, 2017
 *      Author: Jeff Delmerico
 *   Institute: University of Zürich, Robotics and Perception Group
 */

#include <ros/ros.h>
#include "archimede_octomap_to_gridmap/archimede_octomap_to_gridmap.hpp"

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "archimede_octomap_to_gridmap");
  ros::NodeHandle nh("~");
  archimede_octomap_to_gridmap::OctomapToGridmap octomapToGridmap(nh);
  ros::Duration(2.0).sleep();

  ros::Rate r(0.1); // 1 hz
  while (ros::ok())
  {
    octomapToGridmap.convertAndPublishMap();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}