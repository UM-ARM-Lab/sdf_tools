#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <zlib.h>
#include <ros/ros.h>
#include <list>
#include <unordered_map>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/zlib_helpers.hpp>
#include <sdf_tools/collision_map.hpp>
#include <sdf_tools/CollisionMap.h>
#include <sdf_tools/topology_computation.hpp>

using namespace sdf_tools;

bool CollisionMapGrid::SaveToFile(const std::string &filepath)
{
    // Convert to message representation
    sdf_tools::CollisionMap message_rep = GetMessageRepresentation();
    // Save message to file
    try
    {
        std::ofstream output_file(filepath.c_str(), std::ios::out|std::ios::binary);
        uint32_t serialized_size = ros::serialization::serializationLength(message_rep);
        std::unique_ptr<uint8_t> ser_buffer(new uint8_t[serialized_size]);
        ros::serialization::OStream ser_stream(ser_buffer.get(), serialized_size);
        ros::serialization::serialize(ser_stream, message_rep);
        output_file.write((char*)ser_buffer.get(), serialized_size);
        output_file.close();
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool CollisionMapGrid::LoadFromFile(const std::string& filepath)
{
    try
    {
        // Load message from file
        std::ifstream input_file(filepath.c_str(), std::ios::in|std::ios::binary);
        input_file.seekg(0, std::ios::end);
        std::streampos end = input_file.tellg();
        input_file.seekg(0, std::ios::beg);
        std::streampos begin = input_file.tellg();
        uint32_t serialized_size = end - begin;
        std::unique_ptr<uint8_t> deser_buffer(new uint8_t[serialized_size]);
        input_file.read((char*) deser_buffer.get(), serialized_size);
        ros::serialization::IStream deser_stream(deser_buffer.get(), serialized_size);
        sdf_tools::CollisionMap new_message;
        ros::serialization::deserialize(deser_stream, new_message);
        // Load state from the message
        bool success = LoadFromMessageRepresentation(new_message);
        return success;
    }
    catch (...)
    {
        return false;
    }
}

std::vector<uint8_t> CollisionMapGrid::PackBinaryRepresentation(const std::vector<COLLISION_CELL>& raw) const
{
    std::vector<uint8_t> packed(raw.size() * 8);
    for (size_t field_idx = 0, binary_index = 0; field_idx < raw.size(); field_idx++, binary_index+=8)
    {
        COLLISION_CELL raw_cell = raw[field_idx];
        std::vector<uint8_t> packed_cell = CollisionCellToBinary(raw_cell);
        packed[binary_index] = packed_cell[0];
        packed[binary_index + 1] = packed_cell[1];
        packed[binary_index + 2] = packed_cell[2];
        packed[binary_index + 3] = packed_cell[3];
        packed[binary_index + 4] = packed_cell[4];
        packed[binary_index + 5] = packed_cell[5];
        packed[binary_index + 6] = packed_cell[6];
        packed[binary_index + 7] = packed_cell[7];
    }
    return packed;
}

std::vector<COLLISION_CELL> CollisionMapGrid::UnpackBinaryRepresentation(const std::vector<uint8_t>& packed) const
{
    if ((packed.size() % 8) != 0)
    {
        std::cerr << "Invalid binary representation - length is not a multiple of 8" << std::endl;
        return std::vector<COLLISION_CELL>();
    }
    uint64_t data_size = packed.size() / 8;
    std::vector<COLLISION_CELL> unpacked(data_size);
    for (size_t field_idx = 0, binary_index = 0; field_idx < unpacked.size(); field_idx++, binary_index+=8)
    {
        std::vector<uint8_t> binary_block{packed[binary_index], packed[binary_index + 1], packed[binary_index + 2], packed[binary_index + 3], packed[binary_index + 4], packed[binary_index + 5], packed[binary_index + 6], packed[binary_index + 7]};
        unpacked[field_idx] = CollisionCellFromBinary(binary_block);
    }
    return unpacked;
}

sdf_tools::CollisionMap CollisionMapGrid::GetMessageRepresentation()
{
    sdf_tools::CollisionMap message_rep;
    // Populate message
    message_rep.header.frame_id = frame_;
    const Eigen::Isometry3d& origin_transform = collision_field_.GetOriginTransform();
    message_rep.origin_transform.translation.x = origin_transform.translation().x();
    message_rep.origin_transform.translation.y = origin_transform.translation().y();
    message_rep.origin_transform.translation.z = origin_transform.translation().z();
    const Eigen::Quaterniond origin_transform_rotation(origin_transform.rotation());
    message_rep.origin_transform.rotation.x = origin_transform_rotation.x();
    message_rep.origin_transform.rotation.y = origin_transform_rotation.y();
    message_rep.origin_transform.rotation.z = origin_transform_rotation.z();
    message_rep.origin_transform.rotation.w = origin_transform_rotation.w();
    message_rep.dimensions.x = GetXSize();
    message_rep.dimensions.y = GetYSize();
    message_rep.dimensions.z = GetZSize();
    message_rep.cell_size = GetResolution();
    message_rep.OOB_occupancy_value = collision_field_.GetDefaultValue().occupancy;
    message_rep.OOB_component_value = collision_field_.GetDefaultValue().component;
    message_rep.number_of_components = number_of_components_;
    message_rep.components_valid = components_valid_;
    message_rep.initialized = initialized_;
    const std::vector<COLLISION_CELL> raw_data = collision_field_.GetImmutableRawData();
    const std::vector<uint8_t> binary_data = PackBinaryRepresentation(raw_data);
    message_rep.data = ZlibHelpers::CompressBytes(binary_data);
    return message_rep;
}

bool CollisionMapGrid::LoadFromMessageRepresentation(sdf_tools::CollisionMap& message)
{
    // Make a new voxel grid inside
    const Eigen::Translation3d origin_translation(message.origin_transform.translation.x, message.origin_transform.translation.y, message.origin_transform.translation.z);
    const Eigen::Quaterniond origin_rotation(message.origin_transform.rotation.w, message.origin_transform.rotation.x, message.origin_transform.rotation.y, message.origin_transform.rotation.z);
    const Eigen::Isometry3d origin_transform = origin_translation * origin_rotation;
    COLLISION_CELL OOB_value;
    OOB_value.occupancy = message.OOB_occupancy_value;
    OOB_value.component = message.OOB_component_value;
    VoxelGrid::VoxelGrid<COLLISION_CELL> new_field(origin_transform, message.cell_size, message.dimensions.x, message.dimensions.y, message.dimensions.z, OOB_value);
    // Unpack the binary data
    const std::vector<uint8_t> binary_representation = ZlibHelpers::DecompressBytes(message.data);
    const std::vector<COLLISION_CELL> unpacked = UnpackBinaryRepresentation(binary_representation);
    if (unpacked.empty())
    {
        std::cerr << "Unpack returned an empty CollisionMapGrid" << std::endl;
        return false;
    }
    bool success = new_field.SetRawData(unpacked);
    if (!success)
    {
        std::cerr << "Unable to set internal representation of the CollisionMapGrid" << std::endl;
        return false;
    }
    // Set it
    collision_field_ = new_field;
    frame_ = message.header.frame_id;
    number_of_components_ = message.number_of_components;
    components_valid_ = message.components_valid;
    initialized_ = message.initialized;
    return true;
}

visualization_msgs::Marker CollisionMapGrid::ExportForDisplay(const std_msgs::ColorRGBA& collision_color, const std_msgs::ColorRGBA& free_color, const std_msgs::ColorRGBA& unknown_color) const
{
    // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
    visualization_msgs::Marker display_rep;
    // Populate the header
    display_rep.header.frame_id = frame_;
    // Populate the options
    display_rep.ns = "collision_map_display";
    display_rep.id = 1;
    display_rep.type = visualization_msgs::Marker::CUBE_LIST;
    display_rep.action = visualization_msgs::Marker::ADD;
    display_rep.lifetime = ros::Duration(0.0);
    display_rep.frame_locked = false;
    const Eigen::Isometry3d base_transform = Eigen::Isometry3d::Identity();
    display_rep.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(base_transform);
    display_rep.scale.x = GetResolution();
    display_rep.scale.y = GetResolution();
    display_rep.scale.z = GetResolution();
    // Add all the cells of the SDF to the message
    for (int64_t x_index = 0; x_index < collision_field_.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < collision_field_.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < collision_field_.GetNumZCells(); z_index++)
            {
                // Convert grid indices into a real-world location
                const Eigen::Vector4d location = collision_field_.GridIndexToLocation(x_index, y_index, z_index);
                geometry_msgs::Point new_point;
                new_point.x = location(0);
                new_point.y = location(1);
                new_point.z = location(2);
                if (collision_field_.GetImmutable(x_index, y_index, z_index).first.occupancy > 0.5)
                {
                    if (collision_color.a > 0.0)
                    {
                        display_rep.points.push_back(new_point);
                        display_rep.colors.push_back(collision_color);
                    }
                }
                else if (collision_field_.GetImmutable(x_index, y_index, z_index).first.occupancy < 0.5)
                {
                    if (free_color.a > 0.0)
                    {
                        display_rep.points.push_back(new_point);
                        display_rep.colors.push_back(free_color);
                    }
                }
                else
                {
                    if (unknown_color.a > 0.0)
                    {
                        display_rep.points.push_back(new_point);
                        display_rep.colors.push_back(unknown_color);
                    }
                }
            }
        }
    }
    return display_rep;
}

visualization_msgs::MarkerArray CollisionMapGrid::ExportForSeparateDisplay(const std_msgs::ColorRGBA& collision_color, const std_msgs::ColorRGBA& free_color, const std_msgs::ColorRGBA& unknown_color) const
{
    const std_msgs::ColorRGBA no_color = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(0.0, 0.0, 0.0, 0.0);
    visualization_msgs::Marker collision_only_marker = ExportForDisplay(collision_color, no_color, no_color);
    collision_only_marker.ns = "collision_only";
    visualization_msgs::Marker free_only_marker = ExportForDisplay(no_color, free_color, no_color);
    free_only_marker.ns = "free_only";
    visualization_msgs::Marker unknown_only_marker = ExportForDisplay(no_color, no_color, unknown_color);
    unknown_only_marker.ns = "unknown_only";
    visualization_msgs::MarkerArray display_messages;
    display_messages.markers = {collision_only_marker, free_only_marker, unknown_only_marker};
    return display_messages;
}

visualization_msgs::Marker CollisionMapGrid::ExportSurfacesForDisplay(const std_msgs::ColorRGBA& collision_color, const std_msgs::ColorRGBA& free_color, const std_msgs::ColorRGBA& unknown_color) const
{
    // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
    visualization_msgs::Marker display_rep;
    // Populate the header
    display_rep.header.frame_id = frame_;
    // Populate the options
    display_rep.ns = "collision_map_display";
    display_rep.id = 1;
    display_rep.type = visualization_msgs::Marker::CUBE_LIST;
    display_rep.action = visualization_msgs::Marker::ADD;
    display_rep.lifetime = ros::Duration(0.0);
    display_rep.frame_locked = false;
    const Eigen::Isometry3d base_transform = Eigen::Isometry3d::Identity();
    display_rep.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(base_transform);
    display_rep.scale.x = GetResolution();
    display_rep.scale.y = GetResolution();
    display_rep.scale.z = GetResolution();
    // Add all the cells of the SDF to the message
    for (int64_t x_index = 0; x_index < collision_field_.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < collision_field_.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < collision_field_.GetNumZCells(); z_index++)
            {
                if (IsSurfaceIndex(x_index, y_index, z_index))
                {
                    // Convert grid indices into a real-world location
                    const Eigen::Vector4d location = collision_field_.GridIndexToLocation(x_index, y_index, z_index);
                    geometry_msgs::Point new_point;
                    new_point.x = location(0);
                    new_point.y = location(1);
                    new_point.z = location(2);
                    if (collision_field_.GetImmutable(x_index, y_index, z_index).first.occupancy > 0.5)
                    {
                        if (collision_color.a > 0.0)
                        {
                            display_rep.points.push_back(new_point);
                            display_rep.colors.push_back(collision_color);
                        }
                    }
                    else if (collision_field_.GetImmutable(x_index, y_index, z_index).first.occupancy < 0.5)
                    {
                        if (free_color.a > 0.0)
                        {
                            display_rep.points.push_back(new_point);
                            display_rep.colors.push_back(free_color);
                        }
                    }
                    else
                    {
                        if (unknown_color.a > 0.0)
                        {
                            display_rep.points.push_back(new_point);
                            display_rep.colors.push_back(unknown_color);
                        }
                    }
                }
            }
        }
    }
    return display_rep;
}

visualization_msgs::MarkerArray CollisionMapGrid::ExportSurfacesForSeparateDisplay(const std_msgs::ColorRGBA& collision_color, const std_msgs::ColorRGBA& free_color, const std_msgs::ColorRGBA& unknown_color) const
{
    const std_msgs::ColorRGBA no_color = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(0.0, 0.0, 0.0, 0.0);
    visualization_msgs::Marker collision_only_marker = ExportSurfacesForDisplay(collision_color, no_color, no_color);
    collision_only_marker.ns = "collision_surfaces_only";
    visualization_msgs::Marker free_only_marker = ExportSurfacesForDisplay(no_color, free_color, no_color);
    free_only_marker.ns = "free_surfaces_only";
    visualization_msgs::Marker unknown_only_marker = ExportSurfacesForDisplay(no_color, no_color, unknown_color);
    unknown_only_marker.ns = "unknown_surfaces_only";
    visualization_msgs::MarkerArray display_messages;
    display_messages.markers = {collision_only_marker, free_only_marker, unknown_only_marker};
    return display_messages;
}

visualization_msgs::Marker CollisionMapGrid::ExportConnectedComponentsForDisplay(bool color_unknown_components) const
{
    // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
    visualization_msgs::Marker display_rep;
    // Populate the header
    display_rep.header.frame_id = frame_;
    // Populate the options
    display_rep.ns = "connected_components_display";
    display_rep.id = 1;
    display_rep.type = visualization_msgs::Marker::CUBE_LIST;
    display_rep.action = visualization_msgs::Marker::ADD;
    display_rep.lifetime = ros::Duration(0.0);
    display_rep.frame_locked = false;
    const Eigen::Isometry3d base_transform = Eigen::Isometry3d::Identity();
    display_rep.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(base_transform);
    display_rep.scale.x = GetResolution();
    display_rep.scale.y = GetResolution();
    display_rep.scale.z = GetResolution();
    // Add all the cells of the SDF to the message
    for (int64_t x_index = 0; x_index < collision_field_.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < collision_field_.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < collision_field_.GetNumZCells(); z_index++)
            {
                // Convert grid indices into a real-world location
                const Eigen::Vector4d location = collision_field_.GridIndexToLocation(x_index, y_index, z_index);
                geometry_msgs::Point new_point;
                new_point.x = location(0);
                new_point.y = location(1);
                new_point.z = location(2);
                display_rep.points.push_back(new_point);
                COLLISION_CELL current_cell = collision_field_.GetImmutable(x_index, y_index, z_index).first;
                if (current_cell.occupancy != 0.5)
                {
                    std_msgs::ColorRGBA color = GenerateComponentColor(current_cell.component);
                    display_rep.colors.push_back(color);
                }
                else
                {
                    if (color_unknown_components)
                    {
                        std_msgs::ColorRGBA color = GenerateComponentColor(current_cell.component);
                        display_rep.colors.push_back(color);
                    }
                    else
                    {
                        std_msgs::ColorRGBA color;
                        color.a = 1.0;
                        color.r = 0.5;
                        color.g = 0.5;
                        color.b = 0.5;
                        display_rep.colors.push_back(color);
                    }
                }
            }
        }
    }
    return display_rep;
}

uint32_t CollisionMapGrid::UpdateConnectedComponents()
{
    // If the connected components are already valid, skip computing them again
    if (components_valid_)
    {
        return number_of_components_;
    }
    components_valid_ = false;
    // Make the helper functions
    const std::function<bool(const VoxelGrid::GRID_INDEX&, const VoxelGrid::GRID_INDEX&)> are_connected_fn
        = [&] (const VoxelGrid::GRID_INDEX& index1, const VoxelGrid::GRID_INDEX& index2)
    {
        auto query1 = collision_field_.GetImmutable(index1);
        auto query2 = collision_field_.GetImmutable(index2);
        assert(query1.second);
        assert(query2.second);
        if ((query1.first.occupancy > 0.5) == (query2.first.occupancy > 0.5))
        {
            return true;
        }
        else
        {
            return false;
        }
    };
    const std::function<int64_t(const VoxelGrid::GRID_INDEX&)> get_component_fn = [&] (const VoxelGrid::GRID_INDEX& index)
    {
        auto query = collision_field_.GetImmutable(index);
        if (query.second)
        {
            return (int64_t)query.first.component;
        }
        else
        {
            return (int64_t)-1;
        }
    };
    const std::function<void(const VoxelGrid::GRID_INDEX&, const uint32_t)> mark_component_fn = [&] (const VoxelGrid::GRID_INDEX& index, const uint32_t component)
    {
        auto query = collision_field_.GetMutable(index);
        if (query.second)
        {
            collision_field_.SetValue(index, COLLISION_CELL(query.first.occupancy, component));
        }
    };
    number_of_components_ = topology_computation::ComputeConnectedComponents(collision_field_, are_connected_fn, get_component_fn, mark_component_fn);
    components_valid_ = true;
    return number_of_components_;
}

std::map<uint32_t, std::pair<int32_t, int32_t>> CollisionMapGrid::ComputeComponentTopology(bool ignore_empty_components, bool recompute_connected_components, bool verbose)
{
    // Recompute the connected components if need be
    if (recompute_connected_components)
    {
        UpdateConnectedComponents();
    }
    const std::function<int64_t(const VoxelGrid::GRID_INDEX&)> get_component_fn = [&] (const VoxelGrid::GRID_INDEX& index)
    {
        auto query = collision_field_.GetImmutable(index);
        if (query.second)
        {
            return (int64_t)query.first.component;
        }
        else
        {
            return (int64_t)-1;
        }
    };
    const std::function<bool(const VoxelGrid::GRID_INDEX&)> is_surface_index_fn = [&] (const VoxelGrid::GRID_INDEX& index)
    {
        if (ignore_empty_components)
        {
            const COLLISION_CELL& current_cell = collision_field_.GetImmutable(index).first;
            if (current_cell.occupancy > 0.5)
            {
                if (IsConnectedComponentSurfaceIndex(index.x, index.y, index.z))
                {
                    return true;
                }
            }
        }
        else
        {
            if (IsConnectedComponentSurfaceIndex(index.x, index.y, index.z))
            {
                return true;
            }
        }
        return false;
    };
    return topology_computation::ComputeComponentTopology(collision_field_, get_component_fn, is_surface_index_fn, verbose);
}
