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
#include "arc_utilities/zlib_helpers.hpp"
#include "sdf_tools/reachability_map.hpp"
#include "sdf_tools/ReachabilityMap.h"

using namespace sdf_tools;

bool ReachabilityMapGrid::SaveToFile(const std::string& filepath) const
{
    // Convert to message representation
    sdf_tools::ReachabilityMap message_rep = GetMessageRepresentation();
    // Save message to file
    try
    {
        std::ofstream output_file(filepath.c_str(), std::ios::out|std::ios::binary);
        u_int32_t serialized_size = ros::serialization::serializationLength(message_rep);
        std::unique_ptr<u_int8_t> ser_buffer(new u_int8_t[serialized_size]);
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

bool ReachabilityMapGrid::LoadFromFile(const std::string& filepath)
{
    try
    {
        // Load message from file
        std::ifstream input_file(filepath.c_str(), std::ios::in|std::ios::binary);
        input_file.seekg(0, std::ios::end);
        std::streampos end = input_file.tellg();
        input_file.seekg(0, std::ios::beg);
        std::streampos begin = input_file.tellg();
        u_int32_t serialized_size = end - begin;
        std::unique_ptr<u_int8_t> deser_buffer(new u_int8_t[serialized_size]);
        input_file.read((char*) deser_buffer.get(), serialized_size);
        ros::serialization::IStream deser_stream(deser_buffer.get(), serialized_size);
        sdf_tools::ReachabilityMap new_message;
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

std::vector<u_int8_t> ReachabilityMapGrid::PackBinaryRepresentation(const std::vector<REACHABILITY_MAP_CELL_TYPE>& raw) const
{
    std::vector<u_int8_t> packed(raw.size() * sizeof(REACHABILITY_MAP_CELL_TYPE));
    for (size_t field_idx = 0, binary_index = 0; field_idx < raw.size(); field_idx++, binary_index+=sizeof(REACHABILITY_MAP_CELL_TYPE))
    {
        const REACHABILITY_MAP_CELL_TYPE raw_reachability = raw[field_idx];
        std::vector<u_int8_t> packed_cell = ReachabilityToBinary(raw_reachability);
        memcpy(&packed[binary_index], &packed_cell.front(), sizeof(REACHABILITY_MAP_CELL_TYPE));
    }
    return packed;
}

std::vector<REACHABILITY_MAP_CELL_TYPE> ReachabilityMapGrid::UnpackBinaryRepresentation(const std::vector<u_int8_t>& packed) const
{
    if ((packed.size() % sizeof(REACHABILITY_MAP_CELL_TYPE)) != 0)
    {
        std::cerr << "Invalid binary representation - length is not a multiple of " << sizeof(REACHABILITY_MAP_CELL_TYPE) << std::endl;
        return std::vector<REACHABILITY_MAP_CELL_TYPE>();
    }
    u_int64_t data_size = packed.size() / sizeof(REACHABILITY_MAP_CELL_TYPE);
    std::vector<REACHABILITY_MAP_CELL_TYPE> unpacked(data_size);
    for (size_t field_idx = 0, binary_index = 0; field_idx < unpacked.size(); field_idx++, binary_index+=sizeof(REACHABILITY_MAP_CELL_TYPE))
    {
        std::vector<u_int8_t> binary_block(sizeof(REACHABILITY_MAP_CELL_TYPE));
        memcpy(&binary_block.front(), &packed[binary_index], sizeof(REACHABILITY_MAP_CELL_TYPE));
        unpacked[field_idx] = ReachabilityFromBinary(binary_block);
    }
    return unpacked;
}

sdf_tools::ReachabilityMap ReachabilityMapGrid::GetMessageRepresentation() const
{
    sdf_tools::ReachabilityMap message_rep;
    // Populate message
    message_rep.header.frame_id = frame_;
    Eigen::Affine3d origin_transform = reachability_map_.GetOriginTransform();
    message_rep.origin_transform.translation.x = origin_transform.translation().x();
    message_rep.origin_transform.translation.y = origin_transform.translation().y();
    message_rep.origin_transform.translation.z = origin_transform.translation().z();
    Eigen::Quaterniond origin_transform_rotation(origin_transform.rotation());
    message_rep.origin_transform.rotation.x = origin_transform_rotation.x();
    message_rep.origin_transform.rotation.y = origin_transform_rotation.y();
    message_rep.origin_transform.rotation.z = origin_transform_rotation.z();
    message_rep.origin_transform.rotation.w = origin_transform_rotation.w();
    message_rep.dimensions.x = GetXSize();
    message_rep.dimensions.y = GetYSize();
    message_rep.dimensions.z = GetZSize();
    message_rep.cell_size = GetResolution();
    message_rep.initialized = initialized_;
    message_rep.default_value = ReachabilityToBinary(GetOOBValue());
    const std::vector<REACHABILITY_MAP_CELL_TYPE>& raw_data = reachability_map_.GetRawData();
    std::vector<u_int8_t> binary_data = PackBinaryRepresentation(raw_data);
    message_rep.data = ZlibHelpers::CompressBytes(binary_data);
    return message_rep;
}

bool ReachabilityMapGrid::LoadFromMessageRepresentation(const sdf_tools::ReachabilityMap& message)
{
    // Make a new voxel grid inside
    Eigen::Translation3d origin_translation(message.origin_transform.translation.x, message.origin_transform.translation.y, message.origin_transform.translation.z);
    Eigen::Quaterniond origin_rotation(message.origin_transform.rotation.w, message.origin_transform.rotation.x, message.origin_transform.rotation.y, message.origin_transform.rotation.z);
    Eigen::Affine3d origin_transform = origin_translation * origin_rotation;
    REACHABILITY_MAP_CELL_TYPE default_value = ReachabilityFromBinary(message.default_value);
    VoxelGrid::VoxelGrid<REACHABILITY_MAP_CELL_TYPE> new_field(origin_transform, message.cell_size, message.dimensions.x, message.dimensions.y, message.dimensions.z, default_value);
    // Unpack the binary data
    std::vector<u_int8_t> binary_representation = ZlibHelpers::DecompressBytes(message.data);
    std::vector<REACHABILITY_MAP_CELL_TYPE> unpacked = UnpackBinaryRepresentation(binary_representation);
    if (unpacked.empty())
    {
        std::cerr << "Unpack returned an empty ReachabilityMapGrid" << std::endl;
        return false;
    }
    bool success = new_field.SetRawData(unpacked);
    if (!success)
    {
        std::cerr << "Unable to set internal representation of the ReachabilityMapGrid" << std::endl;
        return false;
    }
    // Set it
    reachability_map_ = new_field;
    frame_ = message.header.frame_id;
    initialized_ = message.initialized;
    return true;
}
