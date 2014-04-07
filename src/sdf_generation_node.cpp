#include <ros/ros.h>
#include "sdf_tools/voxel_grid.hpp"
#include "sdf_tools/sdf_tools.hpp"

int main(int argc, char** argv)
{
    VOXEL_GRID::VoxelGrid<double> test_grid(0.1, 1.0, 1.0, 1.0, INFINITY);
    std::vector<u_int32_t> test_index = test_grid.LocationToGridIndex(-0.45, 0.0, 0.45);
    std::cout << "Test index " << test_index[0] << "," << test_index[1] << "," << test_index[2] << std::endl;
    std::vector<double> test_location = test_grid.GridIndexToLocation(test_index[0], test_index[1], test_index[2]);
    std::cout << "Test location " << test_location[0] << "," << test_location[1] << "," << test_location[2] << std::endl;
    std::vector<u_int32_t> check_index = test_grid.LocationToGridIndex(test_location[0], test_location[1], test_location[2]);
    std::cout << "Check index " << check_index[0] << "," << check_index[1] << "," << check_index[2] << std::endl;
    /*
    ros::init(argc, argv, "image_sdf");
    ROS_INFO("Starting SDF from image generator...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string binary_base_topic;
    std::string sdf_preview_topic;
    std::string sdf_raw_topic;
    nhp.param(std::string("binary_base_topic"), binary_base_topic, std::string("camera/rgb/binary"));
    nhp.param(std::string("sdf_preview_topic"), sdf_preview_topic, std::string("camera/rgb/sdf"));
    nhp.param(std::string("sdf_raw_topic"), sdf_raw_topic, std::string("camera/rgb/sdf_raw"));
    ImageSDF processor(nh, binary_base_topic, sdf_preview_topic, sdf_raw_topic);
    ROS_INFO("...startup complete");
    processor.loop();
    */
    return 0;
}
