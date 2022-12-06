#include <string>
#include <vector>

#include <geometric_shapes/shape_to_marker.h>

#include <geometry_msgs/PoseStamped.h>
#include <shape_msgs/SolidPrimitive.h>
#include <visualization_msgs/Marker.h>

#include <octomap_filter/octomap_filter.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_filter_test");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    std::string out_frame;
    bool clear_map;
    nh_priv.param<std::string>("output_frame", out_frame, "map");
    nh_priv.param<bool>("clear_octo_state", clear_map, false);
    octomap_filter::OctomapFilter octo_filter(nh, out_frame, clear_map);
    ros::Publisher mkr_pub = nh.advertise<visualization_msgs::Marker>("/test_shapes_markers", 3);

    ROS_INFO("Test octomap_filtering with shape objects");

    int seq_id = 0;

    ros::Rate r(20);
    while (ros::ok())
    {
        std::vector<shape_msgs::SolidPrimitive> current_shapes;
        std::vector<geometry_msgs::PoseStamped> shapes_poses;

        shape_msgs::SolidPrimitive sphere_ignored, sphere_filtered, box_filtered;

        sphere_ignored.dimensions.resize(1);
        sphere_ignored.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = 0.15;
        sphere_ignored.type = shape_msgs::SolidPrimitive::SPHERE;

        geometry_msgs::PoseStamped sphere_ignored_pose;
        sphere_ignored_pose.header.stamp = ros::Time::now();
        sphere_ignored_pose.header.frame_id = out_frame;
        sphere_ignored_pose.pose.position.x = 0.4;
        sphere_ignored_pose.pose.position.y = 1.0;
        sphere_ignored_pose.pose.position.z = 0.05;
        sphere_ignored_pose.pose.orientation.x = 0.0;
        sphere_ignored_pose.pose.orientation.y = 0.0;
        sphere_ignored_pose.pose.orientation.z = 0.0;
        sphere_ignored_pose.pose.orientation.w = 1.0;

        sphere_filtered.dimensions.resize(1);
        sphere_filtered.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = 0.15;
        sphere_filtered.type = shape_msgs::SolidPrimitive::SPHERE;
        geometry_msgs::PoseStamped sphere_filtered_pose;
        sphere_filtered_pose.header = sphere_ignored_pose.header;
        sphere_filtered_pose.pose = sphere_ignored_pose.pose;
        sphere_filtered_pose.pose.position.y -= 0.7;

        box_filtered.dimensions.resize(3);
        box_filtered.type = shape_msgs::SolidPrimitive::BOX;
        box_filtered.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.15;
        box_filtered.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.2;
        box_filtered.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.2;
        geometry_msgs::PoseStamped box_pose;
        box_pose.header = sphere_ignored_pose.header;
        box_pose.pose =  sphere_ignored_pose.pose;
        box_pose.pose.position.x += 0.5;

        current_shapes.push_back(sphere_filtered);
        current_shapes.push_back(sphere_ignored);
        current_shapes.push_back(box_filtered);
        shapes_poses.push_back(sphere_filtered_pose);
        shapes_poses.push_back(sphere_ignored_pose);
        shapes_poses.push_back(box_pose);

        // Ignore odd shapes
        for (int i = 0; i < current_shapes.size(); i++)
        {
            visualization_msgs::Marker shape_mkr;
            shape_mkr.ns = "test_shapes";
            shape_mkr.header = shapes_poses[i].header;
            shape_mkr.action = visualization_msgs::Marker::ADD;
            shape_mkr.id = seq_id;
            seq_id++;

            // Convert shape to marker
            geometric_shapes::constructMarkerFromShape(current_shapes[i], shape_mkr);
            shape_mkr.pose = shapes_poses[i].pose;

            // If even, then filter
            if (i % 2 == 0)
            {
                octo_filter.filterObjectFromOctomap(current_shapes[i], shapes_poses[i]);
                shape_mkr.color.r = 1.0;
                shape_mkr.color.b = 1.0;
                shape_mkr.color.g = 0.0;
                shape_mkr.color.a = 0.2;
            }
            else
            {
                shape_mkr.color.r = 1.0;
                shape_mkr.color.b = 0.0;
                shape_mkr.color.g = 0.0;
                shape_mkr.color.a = 0.2;
            }

            mkr_pub.publish(shape_mkr);
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}