#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <zlib.h>
#include <octomap_msgs/conversions.h>
#include <sdf_tools/sdf.hpp>
#include <sdf_tools/sdf_generation.hpp>
#include <sdf_tools/sdf_builder.hpp>
#include <sdf_tools/SDF.h>

using namespace sdf_tools;

SDF_Builder::SDF_Builder(ros::NodeHandle& nh, const Eigen::Isometry3d& origin_transform, const std::string& frame, const double x_size, const double y_size, const double z_size, const double resolution, const float OOB_value, const std::string& planning_scene_service) : nh_(nh)
{
    origin_transform_ = origin_transform;
    frame_ = frame;
    x_size_ = x_size;
    y_size_ = y_size;
    z_size_ = z_size;
    resolution_ = resolution;
    OOB_value_ = OOB_value;
    if (!BuildInternalPlanningScene())
    {
        throw std::invalid_argument("Unable to construct internal planning scene");
    }
    initialized_ = true;
    has_cached_sdf_ = false;
    has_cached_collmap_ = false;
    has_planning_scene_ = false;
    planning_scene_client_ = nh.serviceClient<moveit_msgs::GetPlanningScene>(planning_scene_service);
}

SDF_Builder::SDF_Builder(ros::NodeHandle& nh, const std::string& frame, const double x_size, const double y_size, const double z_size, const double resolution, const float OOB_value, const std::string& planning_scene_service) : nh_(nh)
{
    const Eigen::Translation3d origin_translation(-x_size * 0.5, -y_size * 0.5, -z_size * 0.5);
    const Eigen::Quaterniond origin_rotation = Eigen::Quaterniond::Identity();
    origin_transform_ = origin_translation * origin_rotation;
    frame_ = frame;
    x_size_ = x_size;
    y_size_ = y_size;
    z_size_ = z_size;
    resolution_ = resolution;
    OOB_value_ = OOB_value;
    if (!BuildInternalPlanningScene())
    {
        throw std::invalid_argument("Unable to construct internal planning scene");
    }
    initialized_ = true;
    has_cached_sdf_ = false;
    has_cached_collmap_ = false;
    has_planning_scene_ = false;
    planning_scene_client_ = nh.serviceClient<moveit_msgs::GetPlanningScene>(planning_scene_service);
}

bool SDF_Builder::BuildInternalPlanningScene()
{
    /* Builds a planning scene from XML string urdf and srdf descriptions */
    // Make the URDF model
    auto urdf_model = std::make_shared<urdf::Model>();
    urdf_model->initString(GenerateSDFComputeBotURDFString());
    // Make the SRDF model
    auto srdf_model = std::make_shared<srdf::Model>();
    srdf_model->initString(*urdf_model, GenerateSDFComputeBotSRDFString());
    // Make the planning scene
    planning_scene_ptr_ = std::make_shared<planning_scene::PlanningScene>(urdf_model, srdf_model);
    return true;
}

std::string SDF_Builder::GenerateSDFComputeBotURDFString() const
{
    // Figure out the minimum+maximum X,Y,Z values (for safety, we pad them out to make sure)
    double min_x_limit = origin_transform_.translation().x() - fabs(2 * x_size_);
    double max_x_limit = origin_transform_.translation().x() + fabs(2 * x_size_);
    double min_y_limit = origin_transform_.translation().y() - fabs(2 * y_size_);
    double max_y_limit = origin_transform_.translation().y() + fabs(2 * y_size_);
    double min_z_limit = origin_transform_.translation().z() - fabs(2 * z_size_);
    double max_z_limit = origin_transform_.translation().z() + fabs(2 * z_size_);
    // Make the limits into strings
    std::ostringstream mnxls_strm;
    mnxls_strm << min_x_limit;
    std::string min_x_limit_str = mnxls_strm.str();
    std::ostringstream mxxls_strm;
    mxxls_strm << max_x_limit;
    std::string max_x_limit_str = mxxls_strm.str();
    std::ostringstream mnyls_strm;
    mnyls_strm << min_y_limit;
    std::string min_y_limit_str = mnyls_strm.str();
    std::ostringstream mxyls_strm;
    mxyls_strm << max_y_limit;
    std::string max_y_limit_str = mxyls_strm.str();
    std::ostringstream mnzls_strm;
    mnzls_strm << min_z_limit;
    std::string min_z_limit_str = mnzls_strm.str();
    std::ostringstream mxzls_strm;
    mxzls_strm << max_z_limit;
    std::string max_z_limit_str = mxzls_strm.str();
    // Figure out the cell resolution
    std::ostringstream crs_strm;
    crs_strm << resolution_;
    std::string cell_resolution_str = crs_strm.str();
    // Make the URDF xml string
    std::string urdf_string;
    urdf_string = "<?xml version=\"1.0\" ?>\n<robot name=\"sdf_compute_bot\">\n<link name=\"" + frame_ + "\">\n</link>\n<joint name=\"virtual_x\" type=\"prismatic\">\n<parent link=\"" + frame_ + "\"/>\n<child link=\"x_stage\"/>\n<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n<axis xyz=\"1 0 0\"/>\n<limit effort=\"10.0\" lower=\"" + min_x_limit_str + "\" upper=\"10.0\" velocity=\"" + max_x_limit_str + "\"/>\n</joint>\n<link name=\"x_stage\">\n</link>\n<joint name=\"virtual_y\" type=\"prismatic\">\n<parent link=\"x_stage\"/>\n<child link=\"y_stage\"/>\n<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n<axis xyz=\"0 1 0\"/>\n<limit effort=\"10.0\" lower=\"" + min_y_limit_str + "\" upper=\"" + max_y_limit_str + "\" velocity=\"10.0\"/>\n</joint>\n<link name=\"y_stage\">\n</link>\n<joint name=\"virtual_z\" type=\"prismatic\">\n<parent link=\"y_stage\"/>\n<child link=\"sdf_bot\"/>\n<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n<axis xyz=\"0 0 1\"/>\n<limit effort=\"10.0\" lower=\"" + min_z_limit_str + "\" upper=\"" + max_z_limit_str + "\" velocity=\"10.0\"/>\n</joint>\n<link name=\"sdf_bot\">\n<visual>\n<geometry>\n<box size=\"" + cell_resolution_str + " " + cell_resolution_str + " " + cell_resolution_str + "\"/>\n</geometry>\n<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n<material name=\"\">\n<color rgba=\"0.3 0.3 0.3 1.0\"/>\n</material>\n</visual>\n</link>\n</robot>";
    return urdf_string;
}

std::string SDF_Builder::GenerateSDFComputeBotSRDFString() const
{
    std::string srdf_string;
    srdf_string = "<?xml version=\"1.0\" ?>\n<robot name=\"sdf_compute_bot\">\n<group name=\"platform\">\n<joint name=\"virtual_x\" />\n<joint name=\"virtual_y\" />\n<joint name=\"virtual_z\" />\n</group>\n<disable_collisions link1=\"" + frame_ + "\" link2=\"x_stage\" reason=\"Adjacent\" />\n<disable_collisions link1=\"" + frame_ + "\" link2=\"y_stage\" reason=\"Adjacent\" />\n<disable_collisions link1=\"" + frame_ + "\" link2=\"sdf_bot\" reason=\"Adjacent\" />\n<disable_collisions link1=\"x_stage\" link2=\"y_stage\" reason=\"Adjacent\" />\n<disable_collisions link1=\"x_stage\" link2=\"sdf_bot\" reason=\"Adjacent\" />\n<disable_collisions link1=\"y_stage\" link2=\"sdf_bot\" reason=\"Adjacent\" />\n</robot>";
    return srdf_string;
}

SignedDistanceField SDF_Builder::UpdateSDF(const uint8_t update_mode)
{
    if (!initialized_)
    {
        throw std::invalid_argument("SDF Builder has not been initialized");
    }
    if (update_mode == USE_CACHED)
    {
        if (has_planning_scene_)
        {
            // Build the SDF
            return UpdateSDFFromPlanningScene();
        }
        else
        {
            ROS_ERROR("No planning scene available");
            throw std::invalid_argument("No planning scene available");
        }
    }
    else
    {
        if (update_mode == USE_ONLY_COLLISION_OBJECTS)
        {
            // Update the planning scene
            moveit_msgs::GetPlanningSceneRequest ps_req;
            ps_req.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
            moveit_msgs::GetPlanningSceneResponse ps_res;
            planning_scene_client_.call(ps_req, ps_res);
            moveit_msgs::PlanningScene& planning_scene_state = ps_res.scene;
            planning_scene_ptr_->usePlanningSceneMsg(planning_scene_state);
            has_planning_scene_ = true;
            // Build the SDF
            return UpdateSDFFromPlanningScene();
        }
        else if (update_mode == USE_ONLY_OCTOMAP)
        {
            // Update the planning scene
            moveit_msgs::GetPlanningSceneRequest ps_req;
            ps_req.components.components = moveit_msgs::PlanningSceneComponents::OCTOMAP;
            moveit_msgs::GetPlanningSceneResponse ps_res;
            planning_scene_client_.call(ps_req, ps_res);
            moveit_msgs::PlanningScene& planning_scene_state = ps_res.scene;
            planning_scene_ptr_->usePlanningSceneMsg(planning_scene_state);
            has_planning_scene_ = true;
            // Build the SDF
            return UpdateSDFFromPlanningScene();
        }
        else if (update_mode == USE_FULL_PLANNING_SCENE)
        {
            // Update the planning scene
            moveit_msgs::GetPlanningSceneRequest ps_req;
            ps_req.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY | moveit_msgs::PlanningSceneComponents::OCTOMAP;
            moveit_msgs::GetPlanningSceneResponse ps_res;
            planning_scene_client_.call(ps_req, ps_res);
            moveit_msgs::PlanningScene& planning_scene_state = ps_res.scene;
            planning_scene_ptr_->usePlanningSceneMsg(planning_scene_state);
            has_planning_scene_ = true;
            // Build the SDF
            return UpdateSDFFromPlanningScene();
        }
        else
        {
            ROS_ERROR("Invalid update mode (mode not recognized)");
            throw std::invalid_argument("Invalid update mode (mode not recognized)");
        }
    }
}

const SignedDistanceField& SDF_Builder::GetCachedSDF() const
{
    if (has_cached_sdf_)
    {
        return cached_sdf_;
    }
    else
    {
        ROS_ERROR("No cached SDF available");
        throw std::invalid_argument("No cached SDF available");
    }
}

VoxelGrid::VoxelGrid<uint8_t> SDF_Builder::UpdateCollisionMap(const uint8_t update_mode)
{
    if (!initialized_)
    {
        throw std::invalid_argument("SDF Builder has not been initialized");
    }
    if (update_mode == USE_CACHED)
    {
        if (has_planning_scene_)
        {
            // Build the collision map
            return UpdateCollisionMapFromPlanningScene();
        }
        else
        {
            ROS_ERROR("No planning scene available");
            throw std::invalid_argument("No planning scene available");
        }
    }
    else
    {
        if (update_mode == USE_ONLY_COLLISION_OBJECTS)
        {
            // Update the planning scene
            moveit_msgs::GetPlanningSceneRequest ps_req;
            ps_req.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
            moveit_msgs::GetPlanningSceneResponse ps_res;
            planning_scene_client_.call(ps_req, ps_res);
            moveit_msgs::PlanningScene& planning_scene_state = ps_res.scene;
            planning_scene_ptr_->usePlanningSceneMsg(planning_scene_state);
            has_planning_scene_ = true;
            // Build the collision map
            return UpdateCollisionMapFromPlanningScene();
        }
        else if (update_mode == USE_ONLY_OCTOMAP)
        {
            // Update the planning scene
            moveit_msgs::GetPlanningSceneRequest ps_req;
            ps_req.components.components = moveit_msgs::PlanningSceneComponents::OCTOMAP;
            moveit_msgs::GetPlanningSceneResponse ps_res;
            planning_scene_client_.call(ps_req, ps_res);
            moveit_msgs::PlanningScene& planning_scene_state = ps_res.scene;
            planning_scene_ptr_->usePlanningSceneMsg(planning_scene_state);
            has_planning_scene_ = true;
            // Build the collision map
            return UpdateCollisionMapFromPlanningScene();
        }
        else if (update_mode == USE_FULL_PLANNING_SCENE)
        {
            // Update the planning scene
            moveit_msgs::GetPlanningSceneRequest ps_req;
            ps_req.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY | moveit_msgs::PlanningSceneComponents::OCTOMAP;
            moveit_msgs::GetPlanningSceneResponse ps_res;
            planning_scene_client_.call(ps_req, ps_res);
            moveit_msgs::PlanningScene& planning_scene_state = ps_res.scene;
            planning_scene_ptr_->usePlanningSceneMsg(planning_scene_state);
            has_planning_scene_ = true;
            // Build the collision map
            return UpdateCollisionMapFromPlanningScene();
        }
        else
        {
            ROS_ERROR("Invalid update mode (mode not recognized)");
            throw std::invalid_argument("Invalid update mode (mode not recognized)");
        }
    }
}

const VoxelGrid::VoxelGrid<uint8_t>& SDF_Builder::GetCachedCollisionMap() const
{
    if (has_cached_collmap_)
    {
        return cached_collmap_;
    }
    else
    {
        ROS_ERROR("No cached Collision Map available");
        throw std::invalid_argument("No cached Collision Map available");
    }
}

VoxelGrid::VoxelGrid<uint8_t> SDF_Builder::UpdateCollisionMapFromPlanningScene()
{
    // Make a collision field for debugging
    VoxelGrid::VoxelGrid<uint8_t> collision_field(origin_transform_, resolution_, x_size_, y_size_, z_size_, 0);
    // Loop through the planning scene to populate the voxel grids
    std::string x_joint("virtual_x");
    std::string y_joint("virtual_y");
    std::string z_joint("virtual_z");
    collision_detection::CollisionRequest col_req;
    collision_detection::CollisionResult col_res;
    robot_state::RobotState& sdf_compute_bot_state = planning_scene_ptr_->getCurrentStateNonConst();
    for (int64_t x_index = 0; x_index < collision_field.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < collision_field.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < collision_field.GetNumZCells(); z_index++)
            {
                // Convert SDF indices into a real-world location
                const Eigen::Vector4d location = collision_field.GridIndexToLocation(x_index, y_index, z_index);
                // Set them
                sdf_compute_bot_state.setJointPositions(x_joint, &location(0));
                sdf_compute_bot_state.setJointPositions(y_joint, &location(1));
                sdf_compute_bot_state.setJointPositions(z_joint, &location(2));
                col_res.clear();
                planning_scene_ptr_->checkCollision(col_req, col_res);
                if (col_res.collision)
                {
                    // Mark as filled
                    //std::cout << "Collision" << std::endl;
                    uint8_t status = 1;
                    collision_field.SetValue(x_index, y_index, z_index, status);
                }
                else
                {
                    // Mark as free space
                    //std::cout << "No collision" << std::endl;
                    uint8_t status = 0;
                    collision_field.SetValue(x_index, y_index, z_index, status);
                }
            }
        }
    }
    // Export the collision map
    cached_collmap_ = collision_field;
    has_cached_collmap_ = true;
    return collision_field;
}

SignedDistanceField SDF_Builder::UpdateSDFFromPlanningScene()
{
    // Make a temporary grid to perform index<->location mapping
    const VoxelGrid::VoxelGrid<uint8_t> temp_grid(origin_transform_, resolution_, x_size_, y_size_, z_size_, OOB_value_);
    const std::string x_joint("virtual_x");
    const std::string y_joint("virtual_y");
    const std::string z_joint("virtual_z");
    collision_detection::CollisionRequest col_req;
    collision_detection::CollisionResult col_res;
    robot_state::RobotState& sdf_compute_bot_state = planning_scene_ptr_->getCurrentStateNonConst();
    std::function<bool(const VoxelGrid::GRID_INDEX&)> is_filled_fn = [&] (const VoxelGrid::GRID_INDEX& index)
    {
        // Convert SDF indices into a real-world location
        const Eigen::Vector4d location = temp_grid.GridIndexToLocation(index);
        sdf_compute_bot_state.setJointPositions(x_joint, &location(0));
        sdf_compute_bot_state.setJointPositions(y_joint, &location(1));
        sdf_compute_bot_state.setJointPositions(z_joint, &location(2));
        col_res.clear();
        planning_scene_ptr_->checkCollision(col_req, col_res);
        if (col_res.collision)
        {
            // Mark as filled
            return true;
        }
        else
        {
            // Mark as free space
            return false;
        }
    };
    cached_sdf_ = sdf_generation::ExtractSignedDistanceField(temp_grid, is_filled_fn, OOB_value_, frame_, false).first;
    has_cached_sdf_ = true;
    // Export the SDF
    return cached_sdf_;
}

void SDF_Builder::UpdatePlanningSceneFromMessage(const moveit_msgs::PlanningScene& planning_scene)
{
    planning_scene_ptr_->usePlanningSceneMsg(planning_scene);
    has_planning_scene_ = true;
}
