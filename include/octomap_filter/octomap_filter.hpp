#ifndef OCTOMAP_FILTER_HPP
#define OCTOMAP_FILTER_HPP

#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <ros/ros.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>

namespace octomap_filter
{
    class OctomapFilter
    {
        public:
            OctomapFilter(ros::NodeHandle nh);
            ~OctomapFilter();

            bool setOctomapPose(const Eigen::Affine3d &wT1);
            bool getOctomapProperties(octomap_msgs::Octomap &octomap, Eigen::Affine3d &octomap_pose);
            bool filterObjectFromOctomap(const shapes::ShapeMsg &current_shapes,
                                        const geometry_msgs::Pose &shapes_pose);
            bool filterObjectFromOctomap(const std::vector<shapes::ShapeMsg> &current_shapes,
                                        const std::vector<geometry_msgs::Pose> &shapes_poses);

        private:
            bool octomap_received_;
            octomap::OcTree *tree_;

            Eigen::Affine3d octomap_pose_wrt_world_;

            // Filtered octomap header
            std_msgs::Header octo_filter_header_;

            boost::mutex tree_mutex_;

            ros::NodeHandle nh_;
            ros::Subscriber octomap_sub_;
            ros::Publisher octomap_pub_;

            /// Octomap callback
            void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg);
    };
}

#endif