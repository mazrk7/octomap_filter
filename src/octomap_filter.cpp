#include <geometric_shapes/bodies.h>
#include <geometric_shapes/body_operations.h>

#include <octomap/OcTreeKey.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_srvs/Empty.h>

#include <octomap_filter/octomap_filter.hpp>

namespace octomap_filter
{
    OctomapFilter::OctomapFilter(ros::NodeHandle nh)
        : nh_(nh), listener_(buffer_), octomap_received_(false), tree_(NULL), filter_objects_{}
    {
        octomap_sub_ = nh_.subscribe("octomap_full", 1, &OctomapFilter::octomapCallback, this);
        clear_octo_client_ = nh_.serviceClient<std_srvs::Empty>("/octomap_server/reset");

        octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap_filtered", 10);
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

        std_srvs::Empty srv;
        if (!clear_octo_client_.call(srv))
        {
            ROS_ERROR("Failed to call service to clear octomap!");
            return;
        }

        octomap::AbstractOcTree *abs_tree = octomap_msgs::msgToMap(*msg);

        // In case we receive a new octomap before we processed the last one
        if (tree_ != NULL)
        {
            delete tree_;
        }
        
        tree_ = dynamic_cast<octomap::OcTree *>(abs_tree);
        octo_filter_header_ = msg->header;
        octomap_received_ = true;

        // Filtering objects
        if (!filter_objects_.empty())
        {
            std::vector<std::pair<octomap::OcTreeKey, unsigned int>> keys;
            for (int i = 0; i < filter_objects_.size(); i++)
            {
                octomap::OcTreeKey minKey, maxKey;
                bodies::AABB bbox;
                if (filter_objects_[i].first.which() == 0)
                {
                    shape_msgs::SolidPrimitive s1 = boost::get<shape_msgs::SolidPrimitive>(filter_objects_[i].first);
                    bodies::Body *body = bodies::constructBodyFromMsg(s1, filter_objects_[i].second);
                    body->computeBoundingBox(bbox);
                }
                else if (filter_objects_[i].first.which() == 1)
                {
                    shape_msgs::Mesh s1 = boost::get<shape_msgs::Mesh>(filter_objects_[i].first);
                    bodies::Body *body = bodies::constructBodyFromMsg(s1, filter_objects_[i].second);
                    body->computeBoundingBox(bbox);
                }
                else
                {
                    ROS_WARN("Only supports MESH and SOLID Pimitives");
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
        }

        octomap_msgs::Octomap filtered_map;
        octomap_msgs::fullMapToMsg(*tree_, filtered_map);
        filtered_map.header = octo_filter_header_;
        octomap_pub_.publish(filtered_map);
    }

    bool OctomapFilter::getOctomapProperties(const std::string &target_frame,
                                             octomap_msgs::Octomap &octomap,
                                             geometry_msgs::TransformStamped &world_to_base_trans)
    {
        if (octomap_received_)
        {
            boost::mutex::scoped_lock lock(tree_mutex_);

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

    bool OctomapFilter::addObjectToOctoFilter(const shapes::ShapeMsg &current_shapes,
                                              const geometry_msgs::PoseStamped &shapes_pose)
    {
        if (octomap_received_)
        {
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

            filter_objects_.push_back(std::make_pair(current_shapes, trans_pose.pose));
            
            return true;
        }

        return false;
    }

    bool OctomapFilter::addObjectToOctoFilter(const std::vector<shapes::ShapeMsg> &current_shapes,
                                              const std::vector<geometry_msgs::PoseStamped> &shapes_poses)
    {
        if (octomap_received_)
        {
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

            for (int i = 0; i < current_shapes.size(); i++)
            {
                geometry_msgs::PoseStamped trans_pose;
                tf2::doTransform(shapes_poses[i], trans_pose, transform);

                filter_objects_.push_back(std::make_pair(current_shapes[i], trans_pose.pose));
            }
            return true;
        }

        return false;
    }
}