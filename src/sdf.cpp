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
#include "sdf_tools/sdf.hpp"
#include "sdf_tools/SDF.h"

using namespace sdf_tools;

std::vector<u_int8_t> SignedDistanceField::decompress_bytes(std::vector<u_int8_t>& compressed)
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

std::vector<u_int8_t> SignedDistanceField::compress_bytes(std::vector<u_int8_t>& uncompressed)
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

SignedDistanceField::SignedDistanceField(std::string frame, double resolution, double x_size, double y_size, double z_size, float OOB_value)
{
    frame_ = frame;
    VOXEL_GRID::VoxelGrid<float> new_field(resolution, x_size, y_size, z_size, OOB_value);
    distance_field_ = new_field;
}

SignedDistanceField::SignedDistanceField(Transformation origin_transform, std::string frame, double resolution, double x_size, double y_size, double z_size, float OOB_value)
{
    frame_ = frame;
    VOXEL_GRID::VoxelGrid<float> new_field(origin_transform, resolution, x_size, y_size, z_size, OOB_value);
    distance_field_ = new_field;
}

std::vector<u_int8_t> SignedDistanceField::GetInternalBinaryRepresentation(const std::vector<float>& field_data)
{
    std::vector<u_int8_t> raw_binary_data(field_data.size() * 4);
    for (size_t field_index = 0, binary_index = 0; field_index < field_data.size(); field_index++, binary_index+=4)
    {
        // Convert the float at the current index into 4 bytes and store them
        float field_value = field_data[field_index];
        std::vector<u_int8_t> binary_value = FloatToBinary(field_value);
        raw_binary_data[binary_index] = binary_value[0];
        raw_binary_data[binary_index + 1] = binary_value[1];
        raw_binary_data[binary_index + 2] = binary_value[2];
        raw_binary_data[binary_index + 3] = binary_value[3];
    }
    return raw_binary_data;
}

std::vector<float> SignedDistanceField::UnpackFieldFromBinaryRepresentation(std::vector<u_int8_t>& binary)
{
    if ((binary.size() % 4) != 0)
    {
        std::cerr << "Invalid binary representation - length is not a multiple of 4" << std::endl;
        return std::vector<float>();
    }
    u_int64_t data_size = binary.size() / 4;
    std::vector<float> field_data(data_size);
    for (size_t field_index = 0, binary_index = 0; field_index < field_data.size(); field_index++, binary_index+=4)
    {
        std::vector<u_int8_t> binary_block{binary[binary_index], binary[binary_index + 1], binary[binary_index + 2], binary[binary_index + 3]};
        field_data[field_index] = FloatFromBinary(binary_block);
    }
    return field_data;
}

bool SignedDistanceField::SaveToFile(std::string& filepath)
{
    // Convert to message representation
    sdf_tools::SDF message_rep = GetMessageRepresentation();
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

bool SignedDistanceField::LoadFromFile(std::string& filepath)
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
        sdf_tools::SDF new_message;
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

sdf_tools::SDF SignedDistanceField::GetMessageRepresentation()
{
    sdf_tools::SDF message_rep;
    // Populate message
    message_rep.header.frame_id = frame_;
    Transformation origin_transform = distance_field_.GetOriginTransform();
    message_rep.origin_transform.translation.x = origin_transform.translation().x();
    message_rep.origin_transform.translation.y = origin_transform.translation().y();
    message_rep.origin_transform.translation.z = origin_transform.translation().z();
    Eigen::Quaterniond origin_transform_rotation(origin_transform.rotation());
    message_rep.origin_transform.rotation.x = origin_transform_rotation.x();
    message_rep.origin_transform.rotation.y = origin_transform_rotation.y();
    message_rep.origin_transform.rotation.z = origin_transform_rotation.z();
    message_rep.origin_transform.rotation.w = origin_transform_rotation.w();
    message_rep.dimensions.x = distance_field_.GetXSize();
    message_rep.dimensions.y = distance_field_.GetYSize();
    message_rep.dimensions.z = distance_field_.GetZSize();
    message_rep.sdf_cell_size = distance_field_.GetCellSize();
    message_rep.OOB_value = distance_field_.GetDefaultValue();
    const std::vector<float>& raw_data = distance_field_.GetRawData();
    std::vector<u_int8_t> binary_data = GetInternalBinaryRepresentation(raw_data);
    message_rep.data = compress_bytes(binary_data);
    return message_rep;
}

bool SignedDistanceField::LoadFromMessageRepresentation(sdf_tools::SDF& message)
{
    // Make a new voxel grid inside
    Eigen::Translation3d origin_translation(message.origin_transform.translation.x, message.origin_transform.translation.y, message.origin_transform.translation.z);
    Eigen::Quaterniond origin_rotation(message.origin_transform.rotation.w, message.origin_transform.rotation.x, message.origin_transform.rotation.y, message.origin_transform.rotation.z);
    Transformation origin_transform = origin_translation * origin_rotation;
    VOXEL_GRID::VoxelGrid<float> new_field(origin_transform, message.sdf_cell_size, message.dimensions.x, message.dimensions.y, message.dimensions.z, message.OOB_value);
    // Unpack the binary data
    std::vector<u_int8_t> binary_data = decompress_bytes(message.data);
    std::vector<float> unpacked = UnpackFieldFromBinaryRepresentation(binary_data);
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
    distance_field_ = new_field;
    frame_ = message.header.frame_id;
    return true;
}

visualization_msgs::Marker SignedDistanceField::ExportForDisplay(float alpha)
{
    // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
    visualization_msgs::Marker display_rep;
    // Populate the header
    display_rep.header.frame_id = frame_;
    // Populate the options
    display_rep.ns = "sdf_display";
    display_rep.id = 1;
    display_rep.type = visualization_msgs::Marker::CUBE_LIST;
    display_rep.action = visualization_msgs::Marker::ADD;
    display_rep.lifetime = ros::Duration(0.0);
    display_rep.frame_locked = false;
    display_rep.scale.x = distance_field_.GetCellSize();
    display_rep.scale.y = distance_field_.GetCellSize();
    display_rep.scale.z = distance_field_.GetCellSize();
    // Add all the cells of the SDF to the message
    double min_distance = 0.0;
    double max_distance = 0.0;
    for (int64_t x_index = 0; x_index < distance_field_.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < distance_field_.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < distance_field_.GetNumZCells(); z_index++)
            {
                // Update minimum/maximum distance variables
                float distance = distance_field_.Get(x_index, y_index, z_index).first;
                if (distance < min_distance)
                {
                    min_distance = distance;
                }
                if (distance > max_distance)
                {
                    max_distance = distance;
                }
                // Convert SDF indices into a real-world location
                std::vector<double> location = distance_field_.GridIndexToLocation(x_index, y_index, z_index);
                geometry_msgs::Point new_point;
                new_point.x = location[0];
                new_point.y = location[1];
                new_point.z = location[2];
                display_rep.points.push_back(new_point);
            }
        }
    }
    // Add colors for all the cells of the SDF to the message
    for (int64_t x_index = 0; x_index < distance_field_.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < distance_field_.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < distance_field_.GetNumZCells(); z_index++)
            {
                // Update minimum/maximum distance variables
                float distance = distance_field_.Get(x_index, y_index, z_index).first;
                std_msgs::ColorRGBA new_color;
                new_color.a = alpha;
                if (distance > 0.0)
                {
                    new_color.b = 0.0;
                    new_color.g = fabs(distance / max_distance);
                    new_color.r = 0.0;
                }
                else if (distance < 0.0)
                {
                    new_color.b = 0.0;
                    new_color.r = fabs(distance / min_distance);
                    new_color.g = 0.0;
                }
                else
                {
                    new_color.b = 1.0;
                    new_color.g = 0.0;
                    new_color.r = 0.0;
                }
                display_rep.colors.push_back(new_color);
            }
        }
    }
    return display_rep;
}

visualization_msgs::Marker SignedDistanceField::ExportForDisplayCollisionOnly(float alpha)
{
    // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
    visualization_msgs::Marker display_rep;
    // Populate the header
    display_rep.header.frame_id = frame_;
    // Populate the options
    display_rep.ns = "sdf_display";
    display_rep.id = 1;
    display_rep.type = visualization_msgs::Marker::CUBE_LIST;
    display_rep.action = visualization_msgs::Marker::ADD;
    display_rep.lifetime = ros::Duration(0.0);
    display_rep.frame_locked = false;
    display_rep.scale.x = distance_field_.GetCellSize();
    display_rep.scale.y = distance_field_.GetCellSize();
    display_rep.scale.z = distance_field_.GetCellSize();
    // Add all the cells of the SDF to the message
    for (int64_t x_index = 0; x_index < distance_field_.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < distance_field_.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < distance_field_.GetNumZCells(); z_index++)
            {
                // Update minimum/maximum distance variables
                float distance = distance_field_.Get(x_index, y_index, z_index).first;
                if (distance <= 0.0)
                {
                    // Convert SDF indices into a real-world location
                    std::vector<double> location = distance_field_.GridIndexToLocation(x_index, y_index, z_index);
                    geometry_msgs::Point new_point;
                    new_point.x = location[0];
                    new_point.y = location[1];
                    new_point.z = location[2];
                    display_rep.points.push_back(new_point);
                    // Color it
                    std_msgs::ColorRGBA new_color;
                    new_color.a = alpha;
                    new_color.b = 0.0;
                    new_color.g = 0.0;
                    new_color.r = 1.0;
                    display_rep.colors.push_back(new_color);
                }
            }
        }
    }
    return display_rep;
}

visualization_msgs::Marker SignedDistanceField::ExportForDebug(float alpha)
{
    // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
    visualization_msgs::Marker display_rep;
    // Populate the header
    display_rep.header.frame_id = frame_;
    // Populate the options
    display_rep.ns = "sdf_display";
    display_rep.id = 1;
    display_rep.type = visualization_msgs::Marker::CUBE_LIST;
    display_rep.action = visualization_msgs::Marker::ADD;
    display_rep.lifetime = ros::Duration(0.0);
    display_rep.frame_locked = false;
    display_rep.scale.x = distance_field_.GetCellSize();
    display_rep.scale.y = distance_field_.GetCellSize();
    display_rep.scale.z = distance_field_.GetCellSize();
    // Add all the cells of the SDF to the message
    for (int64_t x_index = 0; x_index < distance_field_.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < distance_field_.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < distance_field_.GetNumZCells(); z_index++)
            {
                // Convert SDF indices into a real-world location
                std::vector<double> location = distance_field_.GridIndexToLocation(x_index, y_index, z_index);
                geometry_msgs::Point new_point;
                new_point.x = location[0];
                new_point.y = location[1];
                new_point.z = location[2];
                display_rep.points.push_back(new_point);
                // Color it
                std_msgs::ColorRGBA new_color;
                new_color.a = alpha;
                new_color.b = 0.0;
                new_color.g = 1.0;
                new_color.r = 1.0;
                display_rep.colors.push_back(new_color);
            }
        }
    }
    return display_rep;
}
