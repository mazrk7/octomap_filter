#ifndef OCTOMAP_FILTER_HPP
#define OCTOMAP_FILTER_HPP

#include <string>
#include <vector>

#include <ros/ros.h>

#include <boost/thread.hpp>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>

#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Header.h>

namespace octomap_filter
{
    class OctomapFilter
    {
        public:
            OctomapFilter(ros::NodeHandle nh);
            ~OctomapFilter();

            bool getOctomapProperties(const std::string &robot_frame,
                                    octomap_msgs::Octomap &octomap,
                                    geometry_msgs::TransformStamped &transform);
            bool addObjectToOctoFilter(const shapes::ShapeMsg &current_shapes,
                                    const geometry_msgs::PoseStamped &shapes_pose);
            bool addObjectToOctoFilter(const std::vector<shapes::ShapeMsg> &current_shapes,
                                    const std::vector<geometry_msgs::PoseStamped> &shapes_poses);

        private:
            // Filtered octomap properties
            bool octomap_received_;
            octomap::OcTree *tree_;
            std_msgs::Header octo_filter_header_;

            // Objects to filter from the octomap
            std::vector<std::pair<shapes::ShapeMsg, geometry_msgs::Pose>> filter_objects_;

            // Transforms to correctly filter objects from octomap (irrespective of frame differences)
            tf2_ros::Buffer buffer_;
            tf2_ros::TransformListener listener_;

            boost::mutex tree_mutex_;

            // ROS stuff
            ros::NodeHandle nh_;

            ros::Subscriber octomap_sub_;
            ros::Publisher octomap_pub_;
            ros::ServiceClient clear_octo_client_;

            /// Octomap and change in cells callback
            void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg);
    };
}

#endif