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
#include "sdf_tools/collision_map.hpp"
#include "sdf_tools/CollisionMap.h"

using namespace sdf_tools;

std::vector<u_int8_t> CollisionMapGrid::decompress_bytes(std::vector<u_int8_t>& compressed)
{
    z_stream strm;
    std::vector<u_int8_t> buffer;
    const size_t BUFSIZE = 1024 * 1024;
    uint8_t temp_buffer[BUFSIZE];
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    int ret = inflateInit(&strm);
    if (ret != Z_OK)
    {
        (void)inflateEnd(&strm);
        std::cerr << "ZLIB unable to init inflate stream" << std::endl;
        throw std::invalid_argument("ZLIB unable to init inflate stream");
    }
    strm.avail_in = compressed.size();
    strm.next_in = reinterpret_cast<uint8_t *>(compressed.data());
    do
    {
        strm.next_out = temp_buffer;
        strm.avail_out = BUFSIZE;
        ret = inflate(&strm, Z_NO_FLUSH);
        if (buffer.size() < strm.total_out)
        {
            buffer.insert(buffer.end(), temp_buffer, temp_buffer + BUFSIZE - strm.avail_out);
        }
    }
    while (ret == Z_OK);
    if (ret != Z_STREAM_END)
    {
        (void)inflateEnd(&strm);
        std::cerr << "ZLIB unable to inflate stream with ret=" << ret << std::endl;
        throw std::invalid_argument("ZLIB unable to inflate stream");
    }
    (void)inflateEnd(&strm);
    std::vector<u_int8_t> decompressed(buffer);
    return decompressed;
}

std::vector<u_int8_t> CollisionMapGrid::compress_bytes(std::vector<u_int8_t>& uncompressed)
{
    z_stream strm;
    std::vector<u_int8_t> buffer;
    const size_t BUFSIZE = 1024 * 1024;
    uint8_t temp_buffer[BUFSIZE];
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    strm.avail_in = uncompressed.size();
    strm.next_in = reinterpret_cast<u_int8_t *>(uncompressed.data());
    strm.next_out = temp_buffer;
    strm.avail_out = BUFSIZE;
    int ret = deflateInit(&strm, Z_BEST_SPEED);
    if (ret != Z_OK)
    {
        (void)deflateEnd(&strm);
        std::cerr << "ZLIB unable to init deflate stream" << std::endl;
        throw std::invalid_argument("ZLIB unable to init deflate stream");
    }
    while (strm.avail_in != 0)
    {
        ret = deflate(&strm, Z_NO_FLUSH);
        if (ret != Z_OK)
        {
            (void)deflateEnd(&strm);
            std::cerr << "ZLIB unable to deflate stream" << std::endl;
            throw std::invalid_argument("ZLIB unable to deflate stream");
        }
        if (strm.avail_out == 0)
        {
            buffer.insert(buffer.end(), temp_buffer, temp_buffer + BUFSIZE);
            strm.next_out = temp_buffer;
            strm.avail_out = BUFSIZE;
        }
    }
    int deflate_ret = Z_OK;
    while (deflate_ret == Z_OK)
    {
        if (strm.avail_out == 0)
        {
            buffer.insert(buffer.end(), temp_buffer, temp_buffer + BUFSIZE);
            strm.next_out = temp_buffer;
            strm.avail_out = BUFSIZE;
        }
        deflate_ret = deflate(&strm, Z_FINISH);
    }
    if (deflate_ret != Z_STREAM_END)
    {
        (void)deflateEnd(&strm);
        std::cerr << "ZLIB unable to deflate stream" << std::endl;
        throw std::invalid_argument("ZLIB unable to deflate stream");
    }
    buffer.insert(buffer.end(), temp_buffer, temp_buffer + BUFSIZE - strm.avail_out);
    (void)deflateEnd(&strm);
    std::vector<u_int8_t> compressed(buffer);
    return compressed;
}

CollisionMapGrid::CollisionMapGrid(std::string frame, double resolution, double x_size, double y_size, double z_size, u_int8_t OOB_value)
{
    frame_ = frame;
    VOXEL_GRID::VoxelGrid<u_int8_t> new_field(resolution, x_size, y_size, z_size, OOB_value);
    collision_field_ = new_field;
}

CollisionMapGrid::CollisionMapGrid(Transformation origin_transform, std::string frame, double resolution, double x_size, double y_size, double z_size, u_int8_t OOB_value)
{
    frame_ = frame;
    VOXEL_GRID::VoxelGrid<u_int8_t> new_field(origin_transform, resolution, x_size, y_size, z_size, OOB_value);
    collision_field_ = new_field;
}

bool CollisionMapGrid::SaveToFile(std::string& filepath)
{
    // Convert to message representation
    sdf_tools::CollisionMap message_rep = GetMessageRepresentation();
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

bool CollisionMapGrid::LoadFromFile(std::string& filepath)
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

sdf_tools::CollisionMap CollisionMapGrid::GetMessageRepresentation()
{
    sdf_tools::CollisionMap message_rep;
    // Populate message
    message_rep.header.frame_id = frame_;
    Transformation origin_transform = collision_field_.GetOriginTransform();
    message_rep.origin_transform.translation.x = origin_transform.translation().x();
    message_rep.origin_transform.translation.y = origin_transform.translation().y();
    message_rep.origin_transform.translation.z = origin_transform.translation().z();
    Eigen::Quaterniond origin_transform_rotation(origin_transform.rotation());
    message_rep.origin_transform.rotation.x = origin_transform_rotation.x();
    message_rep.origin_transform.rotation.y = origin_transform_rotation.y();
    message_rep.origin_transform.rotation.z = origin_transform_rotation.z();
    message_rep.origin_transform.rotation.w = origin_transform_rotation.w();
    message_rep.dimensions.x = collision_field_.GetXSize();
    message_rep.dimensions.y = collision_field_.GetYSize();
    message_rep.dimensions.z = collision_field_.GetZSize();
    message_rep.cell_size = collision_field_.GetCellSize();
    message_rep.OOB_value = collision_field_.GetDefaultValue();
    std::vector<u_int8_t> binary_data = collision_field_.GetRawData();
    message_rep.data = compress_bytes(binary_data);
    return message_rep;
}

bool CollisionMapGrid::LoadFromMessageRepresentation(sdf_tools::CollisionMap& message)
{
    // Make a new voxel grid inside
    Eigen::Translation3d origin_translation(message.origin_transform.translation.x, message.origin_transform.translation.y, message.origin_transform.translation.z);
    Eigen::Quaterniond origin_rotation(message.origin_transform.rotation.w, message.origin_transform.rotation.x, message.origin_transform.rotation.y, message.origin_transform.rotation.z);
    Transformation origin_transform = origin_translation * origin_rotation;
    VOXEL_GRID::VoxelGrid<u_int8_t> new_field(origin_transform, message.cell_size, message.dimensions.x, message.dimensions.y, message.dimensions.z, message.OOB_value);
    // Unpack the binary data
    std::vector<u_int8_t> unpacked = decompress_bytes(message.data);
    if (unpacked.empty())
    {
        std::cerr << "Unpack returned an empty SDF" << std::endl;
        return false;
    }
    bool success = new_field.SetRawData(unpacked);
    if (!success)
    {
        std::cerr << "Unable to set internal representation of the SDF" << std::endl;
        return false;
    }
    // Set it
    collision_field_ = new_field;
    frame_ = message.header.frame_id;
    return true;
}

visualization_msgs::Marker CollisionMapGrid::ExportForDisplay(std_msgs::ColorRGBA color)
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
    display_rep.scale.x = collision_field_.GetCellSize();
    display_rep.scale.y = collision_field_.GetCellSize();
    display_rep.scale.z = collision_field_.GetCellSize();
    // Add all the cells of the SDF to the message
    for (int64_t x_index = 0; x_index < collision_field_.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < collision_field_.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < collision_field_.GetNumZCells(); z_index++)
            {
                // Convert grid indices into a real-world location
                std::vector<double> location = collision_field_.GridIndexToLocation(x_index, y_index, z_index);
                geometry_msgs::Point new_point;
                new_point.x = location[0];
                new_point.y = location[1];
                new_point.z = location[2];
                display_rep.points.push_back(new_point);
                display_rep.colors.push_back(color);
            }
        }
    }
    return display_rep;
}
