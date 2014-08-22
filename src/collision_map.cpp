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

CollisionMapGrid::CollisionMapGrid(std::string frame, double resolution, double x_size, double y_size, double z_size, collision_cell OOB_value)
{
    frame_ = frame;
    VOXEL_GRID::VoxelGrid<collision_cell> new_field(resolution, x_size, y_size, z_size, OOB_value);
    collision_field_ = new_field;
}

CollisionMapGrid::CollisionMapGrid(Transformation origin_transform, std::string frame, double resolution, double x_size, double y_size, double z_size, collision_cell OOB_value)
{
    frame_ = frame;
    VOXEL_GRID::VoxelGrid<collision_cell> new_field(origin_transform, resolution, x_size, y_size, z_size, OOB_value);
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

std::vector<u_int8_t> CollisionMapGrid::PackBinaryRepresentation(std::vector<collision_cell>& raw)
{
    std::vector<u_int8_t> packed(raw.size() * 8);
    for (size_t field_idx = 0, binary_index = 0; field_idx < raw.size(); field_idx++, binary_index+=8)
    {
        collision_cell raw_cell = raw[field_idx];
        std::vector<u_int8_t> packed_cell = CollisionCellToBinary(raw_cell);
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

std::vector<collision_cell> CollisionMapGrid::UnpackBinaryRepresentation(std::vector<u_int8_t>& packed)
{
    if ((packed.size() % 8) != 0)
    {
        std::cerr << "Invalid binary representation - length is not a multiple of 8" << std::endl;
        return std::vector<collision_cell>();
    }
    u_int64_t data_size = packed.size() / 8;
    std::vector<collision_cell> unpacked(data_size);
    for (size_t field_idx = 0, binary_index = 0; field_idx < unpacked.size(); field_idx++, binary_index+=8)
    {
        std::vector<u_int8_t> binary_block{packed[binary_index], packed[binary_index + 1], packed[binary_index + 2], packed[binary_index + 3], packed[binary_index + 4], packed[binary_index + 5], packed[binary_index + 6], packed[binary_index + 7]};
        unpacked[field_idx] = CollisionCellFromBinary(binary_block);
    }
    return unpacked;
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
    message_rep.OOB_occupancy_value = collision_field_.GetDefaultValue().occupancy;
    message_rep.OOB_component_value = collision_field_.GetDefaultValue().component;
    std::vector<collision_cell> raw_data = collision_field_.GetRawData();
    std::vector<u_int8_t> binary_data = PackBinaryRepresentation(raw_data);
    message_rep.data = compress_bytes(binary_data);
    return message_rep;
}

bool CollisionMapGrid::LoadFromMessageRepresentation(sdf_tools::CollisionMap& message)
{
    // Make a new voxel grid inside
    Eigen::Translation3d origin_translation(message.origin_transform.translation.x, message.origin_transform.translation.y, message.origin_transform.translation.z);
    Eigen::Quaterniond origin_rotation(message.origin_transform.rotation.w, message.origin_transform.rotation.x, message.origin_transform.rotation.y, message.origin_transform.rotation.z);
    Transformation origin_transform = origin_translation * origin_rotation;
    collision_cell OOB_value;
    OOB_value.occupancy = message.OOB_occupancy_value;
    OOB_value.component = message.OOB_component_value;
    VOXEL_GRID::VoxelGrid<collision_cell> new_field(origin_transform, message.cell_size, message.dimensions.x, message.dimensions.y, message.dimensions.z, OOB_value);
    // Unpack the binary data
    std::vector<u_int8_t> binary_representation = decompress_bytes(message.data);
    std::vector<collision_cell> unpacked = UnpackBinaryRepresentation(binary_representation);
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

visualization_msgs::Marker CollisionMapGrid::ExportForDisplay(std_msgs::ColorRGBA collision_color, std_msgs::ColorRGBA free_color, std_msgs::ColorRGBA unknown_color)
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
                if (collision_field_.Get(x_index, y_index, z_index).first.occupancy > 0.5)
                {
                    display_rep.colors.push_back(collision_color);
                }
                else if (collision_field_.Get(x_index, y_index, z_index).first.occupancy < 0.5)
                {
                    display_rep.colors.push_back(free_color);
                }
                else
                {
                    display_rep.colors.push_back(unknown_color);
                }
            }
        }
    }
    return display_rep;
}

std_msgs::ColorRGBA CollisionMapGrid::GenerateComponentColor(u_int32_t component)
{
    // For component < 22, we pick from a table
    if (component == 0)
    {
        std_msgs::ColorRGBA default_color;
        default_color.a = 1.0;
        default_color.r = 1.0;
        default_color.g = 1.0;
        default_color.b = 1.0;
        return default_color;
    }
    else if (component <= 20)
    {
        std_msgs::ColorRGBA color;
        color.a = 1.0;
        if (component == 1)
        {
            color.r = ColorChannelFromHex(0xff);
            color.b = ColorChannelFromHex(0xb3);
            color.g = ColorChannelFromHex(0x00);
        }
        else if (component == 2)
        {
            color.r = ColorChannelFromHex(0x80);
            color.b = ColorChannelFromHex(0x3e);
            color.g = ColorChannelFromHex(0x75);
        }
        else if (component == 3)
        {
            color.r = ColorChannelFromHex(0xff);
            color.b = ColorChannelFromHex(0x68);
            color.g = ColorChannelFromHex(0x00);
        }
        else if (component == 4)
        {
            color.r = ColorChannelFromHex(0xa6);
            color.b = ColorChannelFromHex(0xbd);
            color.g = ColorChannelFromHex(0xd7);
        }
        else if (component == 5)
        {
            color.r = ColorChannelFromHex(0xc1);
            color.b = ColorChannelFromHex(0x00);
            color.g = ColorChannelFromHex(0x20);
        }
        else if (component == 6)
        {
            color.r = ColorChannelFromHex(0xce);
            color.b = ColorChannelFromHex(0xa2);
            color.g = ColorChannelFromHex(0x62);
        }
        else if (component == 7)
        {
            color.r = ColorChannelFromHex(0x81);
            color.b = ColorChannelFromHex(0x70);
            color.g = ColorChannelFromHex(0x66);
        }
        else if (component == 8)
        {
            color.r = ColorChannelFromHex(0x00);
            color.b = ColorChannelFromHex(0x7d);
            color.g = ColorChannelFromHex(0x34);
        }
        else if (component == 9)
        {
            color.r = ColorChannelFromHex(0xf6);
            color.b = ColorChannelFromHex(0x76);
            color.g = ColorChannelFromHex(0x8e);
        }
        else if (component == 10)
        {
            color.r = ColorChannelFromHex(0x00);
            color.b = ColorChannelFromHex(0x53);
            color.g = ColorChannelFromHex(0x8a);
        }
        else if (component == 11)
        {
            color.r = ColorChannelFromHex(0xff);
            color.b = ColorChannelFromHex(0x7a);
            color.g = ColorChannelFromHex(0x5c);
        }
        else if (component == 12)
        {
            color.r = ColorChannelFromHex(0x53);
            color.b = ColorChannelFromHex(0x37);
            color.g = ColorChannelFromHex(0x7a);
        }
        else if (component == 13)
        {
            color.r = ColorChannelFromHex(0xff);
            color.b = ColorChannelFromHex(0x8e);
            color.g = ColorChannelFromHex(0x00);
        }
        else if (component == 14)
        {
            color.r = ColorChannelFromHex(0xb3);
            color.b = ColorChannelFromHex(0x28);
            color.g = ColorChannelFromHex(0x51);
        }
        else if (component == 15)
        {
            color.r = ColorChannelFromHex(0xf4);
            color.b = ColorChannelFromHex(0xc8);
            color.g = ColorChannelFromHex(0x00);
        }
        else if (component == 16)
        {
            color.r = ColorChannelFromHex(0x7f);
            color.b = ColorChannelFromHex(0x18);
            color.g = ColorChannelFromHex(0x0d);
        }
        else if (component == 17)
        {
            color.r = ColorChannelFromHex(0x93);
            color.b = ColorChannelFromHex(0xaa);
            color.g = ColorChannelFromHex(0x00);
        }
        else if (component == 18)
        {
            color.r = ColorChannelFromHex(0x59);
            color.b = ColorChannelFromHex(0x33);
            color.g = ColorChannelFromHex(0x15);
        }
        else if (component == 19)
        {
            color.r = ColorChannelFromHex(0xf1);
            color.b = ColorChannelFromHex(0x3a);
            color.g = ColorChannelFromHex(0x13);
        }
        else
        {
            color.r = ColorChannelFromHex(0x23);
            color.b = ColorChannelFromHex(0x2c);
            color.g = ColorChannelFromHex(0x16);
        }
        return color;
    }
    else
    {
        std::cerr << "More than 20 connected components - attempting to procedurally generate a color" << std::endl;
        std_msgs::ColorRGBA generated_color;
        generated_color.a = 1.0;
        generated_color.r = 0.0;
        generated_color.g = 0.0;
        generated_color.b = 0.0;
        return generated_color;
    }
}

visualization_msgs::Marker CollisionMapGrid::ExportConnectedComponentsForDisplay(bool color_unknown_components)
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
                collision_cell current_cell = collision_field_.Get(x_index, y_index, z_index).first;
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

void CollisionMapGrid::UpdateConnectedComponents()
{
    int64_t total_cells = collision_field_.GetNumXCells() * collision_field_.GetNumYCells() * collision_field_.GetNumZCells();
    int64_t marked_cells = 0;
    u_int32_t connected_components = 0;
    // Sweep through the grid
    for (int64_t x_index = 0; x_index < collision_field_.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < collision_field_.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < collision_field_.GetNumZCells(); z_index++)
            {
                // Check if the cell has already been marked, if so, ignore
                if (collision_field_.Get(x_index, y_index, z_index).first.component > 0)
                {
                    continue;
                }
                // Start marking a new connected component
                connected_components++;
                int64_t cells_marked = MarkConnectedComponent(x_index, y_index, z_index, connected_components);
                marked_cells += cells_marked;
                // Short-circuit if we've marked everything
                if (marked_cells == total_cells)
                {
                    return;
                }
            }
        }
    }
}

int64_t CollisionMapGrid::MarkConnectedComponent(int64_t x_index, int64_t y_index, int64_t z_index, u_int32_t connected_component)
{
    // Make the working queue
    std::list<grid_index> working_queue;
    // Make a hash table to store queued indices (so we don't repeat work)
    std::unordered_map<std::string, int8_t> queued_hashtable;
    // Add the starting index
    grid_index start_index;
    start_index.x = x_index;
    start_index.y = y_index;
    start_index.z = z_index;
    // Enqueue it
    working_queue.push_back(start_index);
    queued_hashtable[GenerateGridIndexKey(start_index)] = 1;
    // Work
    int64_t marked_cells = 0;
    while (working_queue.size() > 0)
    {
        // Get an item off the queue to work with
        grid_index current_index = working_queue.front();
        working_queue.pop_front();
        // Get the current value
        collision_cell current_value = collision_field_.Get(current_index.x, current_index.y, current_index.z).first;
        // Mark the connected component
        current_value.component = connected_component;
        // Update the grid
        collision_field_.Set(current_index.x, current_index.y, current_index.z, current_value);
        // Go through the possible neighbors and enqueue as needed
        // Since there are only six cases (voxels must share a face to be considered connected), we handle each explicitly
        // Case 1
        std::pair<collision_cell, bool> xm1_neighbor = collision_field_.Get(current_index.x - 1, current_index.y, current_index.z);
        if (xm1_neighbor.second && (current_value.occupancy == xm1_neighbor.first.occupancy))
        {
            grid_index neighbor_index;
            neighbor_index.x = current_index.x - 1;
            neighbor_index.y = current_index.y;
            neighbor_index.z = current_index.z;
            std::string key = GenerateGridIndexKey(neighbor_index);
            if (queued_hashtable[key] <= 0)
            {
                queued_hashtable[key] = 1;
                working_queue.push_back(neighbor_index);
            }
        }
        // Case 2
        std::pair<collision_cell, bool> ym1_neighbor = collision_field_.Get(current_index.x, current_index.y - 1, current_index.z);
        if (ym1_neighbor.second && (current_value.occupancy == ym1_neighbor.first.occupancy))
        {
            grid_index neighbor_index;
            neighbor_index.x = current_index.x;
            neighbor_index.y = current_index.y - 1;
            neighbor_index.z = current_index.z;
            std::string key = GenerateGridIndexKey(neighbor_index);
            if (queued_hashtable[key] <= 0)
            {
                queued_hashtable[key] = 1;
                working_queue.push_back(neighbor_index);
            }
        }
        // Case 3
        std::pair<collision_cell, bool> zm1_neighbor = collision_field_.Get(current_index.x, current_index.y, current_index.z - 1);
        if (zm1_neighbor.second && (current_value.occupancy == zm1_neighbor.first.occupancy))
        {
            grid_index neighbor_index;
            neighbor_index.x = current_index.x;
            neighbor_index.y = current_index.y;
            neighbor_index.z = current_index.z - 1;
            std::string key = GenerateGridIndexKey(neighbor_index);
            if (queued_hashtable[key] <= 0)
            {
                queued_hashtable[key] = 1;
                working_queue.push_back(neighbor_index);
            }
        }
        // Case 4
        std::pair<collision_cell, bool> xp1_neighbor = collision_field_.Get(current_index.x + 1, current_index.y, current_index.z);
        if (xp1_neighbor.second && (current_value.occupancy == xp1_neighbor.first.occupancy))
        {
            grid_index neighbor_index;
            neighbor_index.x = current_index.x + 1;
            neighbor_index.y = current_index.y;
            neighbor_index.z = current_index.z;
            std::string key = GenerateGridIndexKey(neighbor_index);
            if (queued_hashtable[key] <= 0)
            {
                queued_hashtable[key] = 1;
                working_queue.push_back(neighbor_index);
            }
        }
        // Case 5
        std::pair<collision_cell, bool> yp1_neighbor = collision_field_.Get(current_index.x, current_index.y + 1, current_index.z);
        if (yp1_neighbor.second && (current_value.occupancy == yp1_neighbor.first.occupancy))
        {
            grid_index neighbor_index;
            neighbor_index.x = current_index.x;
            neighbor_index.y = current_index.y + 1;
            neighbor_index.z = current_index.z;
            std::string key = GenerateGridIndexKey(neighbor_index);
            if (queued_hashtable[key] <= 0)
            {
                queued_hashtable[key] = 1;
                working_queue.push_back(neighbor_index);
            }
        }
        // Case 6
        std::pair<collision_cell, bool> zp1_neighbor = collision_field_.Get(current_index.x, current_index.y, current_index.z + 1);
        if (zp1_neighbor.second && (current_value.occupancy == zp1_neighbor.first.occupancy))
        {
            grid_index neighbor_index;
            neighbor_index.x = current_index.x;
            neighbor_index.y = current_index.y;
            neighbor_index.z = current_index.z + 1;
            std::string key = GenerateGridIndexKey(neighbor_index);
            if (queued_hashtable[key] <= 0)
            {
                queued_hashtable[key] = 1;
                working_queue.push_back(neighbor_index);
            }
        }
    }
    return marked_cells;
}
