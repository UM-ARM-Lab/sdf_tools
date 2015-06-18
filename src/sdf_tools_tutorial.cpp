#include <ros/ros.h>
#include <arc_utilities/voxel_grid.hpp>
#include "sdf_tools/collision_map.hpp"
#include "sdf_tools/CollisionMap.h"
#include "sdf_tools/sdf.hpp"
#include "sdf_tools/SDF.h"
#include "sdf_tools/sdf_builder.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sdf_tools_tutorial");
    return 0;
}
