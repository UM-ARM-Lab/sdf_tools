#include <ros/ros.h>
#include <arc_utilities/voxel_grid.hpp>
#include "sdf_tools/collision_map.hpp"
#include "sdf_tools/CollisionMap.h"
#include "sdf_tools/sdf.hpp"
#include "sdf_tools/SDF.h"
#include "sdf_tools/sdf_builder.hpp"

int main(int argc, char** argv)
{
    // Make a ROS node, which we'll use to publish copies of the data in the CollisionMap and SDF
    // and Rviz markers that allow us to visualize them.
    ros::init(argc, argv, "sdf_tools_tutorial");
    // Get a handle to the current node
    ros::NodeHandle nh;
    // Make a publisher for visualization messages
    ros::Publisher visualization_pub = nh.advertise<visualization_msgs::Marker>("sdf_tools_tutorial_visualization", 1, true);
    // Make a publisher for serialized CollisionMaps
    ros::Publisher collision_map_pub = nh.advertise<sdf_tools::CollisionMap>("collision_map_pub", 1, true);
    // Make a publisher for serialized SDFs
    ros::Publisher sdf_pub = nh.advertise<sdf_tools::SDF>("sdf_pub", 1, true);
    // In preparation, we want to set a couple common paramters
    double resolution = 0.25;
    double x_size = 10.0;
    double y_size = 10.0;
    double z_size = 10.0;
    Eigen::Translation3d origin_translation(0.0, 0.0, 0.0);
    Eigen::Quaterniond origin_rotation(1.0, 0.0, 0.0, 0.0);
    Eigen::Affine3d origin_transform = origin_translation * origin_rotation;
    std::string frame = "tutorial_frame";
    ///////////////////////////////////
    //// Let's make a CollisionMap ////
    ///////////////////////////////////
    // We pick a reasonable out-of-bounds value
    sdf_tools::collision_cell oob_cell;
    oob_cell.occupancy = 0.0;
    oob_cell.component = 0; // This should ALWAYS be zero, unless you know exactly what you're doing
    // First, let's make the container
    sdf_tools::CollisionMapGrid collision_map(origin_transform, frame, resolution, x_size, y_size, z_size, oob_cell);
    // Let's set some values
    ;
    // Let's get some values
    ;
    // Let's compute connected components
    ;
    // Let's display the results to Rviz
    ;
    // Let's export the CollisionMap
    collision_map_pub.publish(collision_map.GetMessageRepresentation());
    ///////////////////////////
    //// Let's make an SDF ////
    ///////////////////////////
    // We pick a reasonable out-of-bounds value
    float oob_value = INFINITY;
    // We start by extracting the SDF from the CollisionMap
    sdf_tools::SignedDistanceField sdf = collision_map.ExtractSignedDistanceField(oob_value);
    // We lock the SDF to prevent unintended changes that would invalidate it
    sdf.Lock();
    // Let's get some values
    ;
    // Let's get some gradients
    ;
    // Let's get the LocalMaximaMap
    ;
    // Let's display the results to Rviz
    ;
    // Let's export the SDF
    sdf_pub.publish(sdf.GetMessageRepresentation());
    std::cout << "...done" << std::endl;
    return 0;
}
