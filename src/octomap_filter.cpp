#include <geometric_shapes/bodies.h>
#include <geometric_shapes/body_operations.h>

#include <octomap/OcTreeKey.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_srvs/Empty.h>

#include <octomap_filter/octomap_filter.hpp>

namespace octomap_filter
{
    OctomapFilter::OctomapFilter(ros::NodeHandle nh)
        : nh_(nh), listener_(buffer_), octomap_received_(false), tree_(NULL), clear_octo_map_(false)
    {
        octomap_sub_ = nh_.subscribe("octomap_full", 1, &OctomapFilter::octomapCallback, this);
        octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap_filtered", 10);
        clear_octo_client_ = nh_.serviceClient<std_srvs::Empty>("/octomap_server/reset");

        octo_filter_header_.frame_id = "map";
    }

    OctomapFilter::OctomapFilter(ros::NodeHandle nh, std::string out_frame, bool clear_octo_state)
        : nh_(nh), listener_(buffer_), octomap_received_(false), tree_(NULL), clear_octo_map_(clear_octo_state)
    {
        octomap_sub_ = nh_.subscribe("octomap_full", 1, &OctomapFilter::octomapCallback, this);
        octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap_filtered", 10);
        clear_octo_client_ = nh_.serviceClient<std_srvs::Empty>("/octomap_server/reset");

        octo_filter_header_.frame_id = out_frame;
    }

    OctomapFilter::~OctomapFilter()
    {
        if (tree_ != NULL)
        {
            delete tree_;
        }
    }

    void OctomapFilter::octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
    {
        boost::mutex::scoped_lock lock(tree_mutex_);

        // Clear octomap tracked state
        if (clear_octo_map_)
        {
            std_srvs::Empty srv;
            if (!clear_octo_client_.call(srv))
            {
                ROS_ERROR("Failed to call service to clear octomap!");
                return;
            }
        }

        octomap::AbstractOcTree *abs_tree = octomap_msgs::msgToMap(*msg);
        // In case we receive a new octomap before we processed the last one
        if (tree_ != NULL)
        {
            delete tree_;
        }

        tree_ = dynamic_cast<octomap::OcTree *>(abs_tree);
        octo_filter_header_.stamp = msg->header.stamp;
        octomap_received_ = true;
    }

    bool OctomapFilter::getOctomapProperties(const std::string &target_frame,
                                             octomap_msgs::Octomap &octomap,
                                             geometry_msgs::TransformStamped &world_to_base_trans)
    {
        if (octomap_received_ && (tree_ != NULL))
        {
            octomap_msgs::fullMapToMsg(*tree_, octomap);

            // Get octomap pose w.r.t. desired target frame
            try
            {
                world_to_base_trans = buffer_.lookupTransform(
                    target_frame,
                    octo_filter_header_.frame_id,
                    ros::Time(0));
            }
            catch (const tf2::TransformException &ex)
            {
                ROS_ERROR("Error during transform: %s", ex.what());
                return false;
            }

            // Clear up
            delete tree_;
            tree_ = NULL;
            octomap_received_ = false;

            return true;
        }

        return false;
    }

    bool OctomapFilter::filterObjectFromOctomap(const shapes::ShapeMsg &current_shapes,
                                                const geometry_msgs::PoseStamped &shapes_pose)
    {
        if (!octomap_received_)
        {
            return false;
        }

        // Make sure poses are in same frame as octomap
        geometry_msgs::TransformStamped transform;
        try
        {
            transform = buffer_.lookupTransform(
                octo_filter_header_.frame_id,
                shapes_pose.header.frame_id,
                ros::Time(0));
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_ERROR("Error during transform: %s", ex.what());
            return false;
        }

        geometry_msgs::PoseStamped trans_pose;
        tf2::doTransform(shapes_pose, trans_pose, transform);

        octomap::OcTreeKey minKey, maxKey;
        bodies::AABB bbox;
        if (current_shapes.which() == 0)
        {
            shape_msgs::SolidPrimitive s1 = boost::get<shape_msgs::SolidPrimitive>(current_shapes);
            bodies::Body *body = bodies::constructBodyFromMsg(s1, trans_pose.pose);
            body->computeBoundingBox(bbox);
        }
        else if (current_shapes.which() == 1)
        {
            shape_msgs::Mesh s1 = boost::get<shape_msgs::Mesh>(current_shapes);
            bodies::Body *body = bodies::constructBodyFromMsg(s1, trans_pose.pose);
            body->computeBoundingBox(bbox);
        }
        else
        {
            ROS_WARN("Only supports MESH and SOLID Pimitives");
            return false;
        }

        tree_->coordToKeyChecked(bbox.min().x(), bbox.min().y(), bbox.min().z(), minKey);
        tree_->coordToKeyChecked(bbox.max().x(), bbox.max().y(), bbox.max().z(), maxKey);

        std::vector<std::pair<octomap::OcTreeKey, unsigned int>> keys;
        for (octomap::OcTree::leaf_iterator it = tree_->begin_leafs(), end = tree_->end_leafs(); it != end; ++it)
        {
            octomap::OcTreeKey k = it.getKey();
            if (k[0] >= minKey[0] && k[1] >= minKey[1] && k[2] >= minKey[2] && k[0] <= maxKey[0] && k[1] <= maxKey[1] && k[2] <= maxKey[2])
            {
                keys.push_back(std::make_pair(k, it.getDepth())); // add to a stack
            }
        }

        // delete nodes which are in bounding box
        for (auto k : keys)
            tree_->deleteNode(k.first, k.second);

        octomap_msgs::Octomap filtered_map;
        octomap_msgs::fullMapToMsg(*tree_, filtered_map);
        filtered_map.header = octo_filter_header_;
        octomap_pub_.publish(filtered_map);

        return true;
    }

    bool OctomapFilter::filterObjectFromOctomap(const std::vector<shapes::ShapeMsg> &current_shapes,
                                                const std::vector<geometry_msgs::PoseStamped> &shapes_poses)
    {
        if (!octomap_received_)
        {
            return false;
        }

        // Make sure poses are in same frame as octomap
        geometry_msgs::TransformStamped transform;
        try
        {
            transform = buffer_.lookupTransform(
                octo_filter_header_.frame_id,
                shapes_poses[0].header.frame_id,
                ros::Time(0));
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_ERROR("Error during transform: %s", ex.what());
            return false;
        }

        octomap::OcTreeKey minKey, maxKey;
        std::vector<std::pair<octomap::OcTreeKey, unsigned int>> keys;
        for (int var = 0; var < current_shapes.size(); ++var)
        {
            geometry_msgs::PoseStamped trans_pose;
            tf2::doTransform(shapes_poses[var], trans_pose, transform);
            bodies::AABB bbox;
            if (current_shapes[var].which() == 0)
            {
                shape_msgs::SolidPrimitive s1 = boost::get<shape_msgs::SolidPrimitive>(current_shapes[var]);
                bodies::Body *body = bodies::constructBodyFromMsg(s1, trans_pose.pose);
                body->computeBoundingBox(bbox);
            }
            else if (current_shapes[var].which() == 1)
            {
                shape_msgs::Mesh s1 = boost::get<shape_msgs::Mesh>(current_shapes[var]);
                bodies::Body *body = bodies::constructBodyFromMsg(s1, trans_pose.pose);
                body->computeBoundingBox(bbox);
            }
            else
            {
                ROS_WARN("Only supports MESH and SOLID Pimitives");
                return false;
            }

            tree_->coordToKeyChecked(bbox.min().x(), bbox.min().y(), bbox.min().z(), minKey);
            tree_->coordToKeyChecked(bbox.max().x(), bbox.max().y(), bbox.max().z(), maxKey);

            for (octomap::OcTree::leaf_iterator it = tree_->begin_leafs(), end = tree_->end_leafs(); it != end; ++it)
            {
                octomap::OcTreeKey k = it.getKey();
                if (k[0] >= minKey[0] && k[1] >= minKey[1] && k[2] >= minKey[2] && k[0] <= maxKey[0] && k[1] <= maxKey[1] && k[2] <= maxKey[2])
                {
                    keys.push_back(std::make_pair(k, it.getDepth())); // add to a stack
                }
            }
        }
        // delete nodes which are in bounding box
        for (auto k : keys)
            tree_->deleteNode(k.first, k.second);

        octomap_msgs::Octomap filtered_map;
        octomap_msgs::fullMapToMsg(*tree_, filtered_map);
        filtered_map.header = octo_filter_header_;
        octomap_pub_.publish(filtered_map);

        return true;
    }
}