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

uint64_t CollisionMapGrid::SerializeSelf(
    std::vector<uint8_t>& buffer,
    const std::function<uint64_t(
      const COLLISION_CELL&, std::vector<uint8_t>&)>& value_serializer) const
{
  UNUSED(value_serializer);
  const uint64_t start_buffer_size = buffer.size();
  // Serialize the initialized
  arc_utilities::SerializeFixedSizePOD<uint8_t>((uint8_t)initialized_, buffer);
  // Serialize the transforms
  arc_utilities::SerializeEigen<Eigen::Isometry3d>(origin_transform_, buffer);
  arc_utilities::SerializeEigen<Eigen::Isometry3d>(inverse_origin_transform_,
                                             buffer);
  // Serialize the data
  arc_utilities::SerializeVector<COLLISION_CELL>(
        data_, buffer, arc_utilities::SerializeFixedSizePOD<COLLISION_CELL>);
  // Serialize the cell sizes
  arc_utilities::SerializeFixedSizePOD<double>(cell_x_size_, buffer);
  arc_utilities::SerializeFixedSizePOD<double>(cell_y_size_, buffer);
  arc_utilities::SerializeFixedSizePOD<double>(cell_z_size_, buffer);
  arc_utilities::SerializeFixedSizePOD<double>(inv_cell_x_size_, buffer);
  arc_utilities::SerializeFixedSizePOD<double>(inv_cell_y_size_, buffer);
  arc_utilities::SerializeFixedSizePOD<double>(inv_cell_z_size_, buffer);
  // Serialize the grid sizes
  arc_utilities::SerializeFixedSizePOD<double>(x_size_, buffer);
  arc_utilities::SerializeFixedSizePOD<double>(y_size_, buffer);
  arc_utilities::SerializeFixedSizePOD<double>(z_size_, buffer);
  // Serialize the control/bounds values
  arc_utilities::SerializeFixedSizePOD<int64_t>(stride1_, buffer);
  arc_utilities::SerializeFixedSizePOD<int64_t>(stride2_, buffer);
  arc_utilities::SerializeFixedSizePOD<int64_t>(num_x_cells_, buffer);
  arc_utilities::SerializeFixedSizePOD<int64_t>(num_y_cells_, buffer);
  arc_utilities::SerializeFixedSizePOD<int64_t>(num_z_cells_, buffer);
  // Serialize the default value
  arc_utilities::SerializeFixedSizePOD<COLLISION_CELL>(default_value_, buffer);
  // Serialize the OOB value
  arc_utilities::SerializeFixedSizePOD<COLLISION_CELL>(oob_value_, buffer);
  // Serialize CollisionMapGrid stuff
  arc_utilities::SerializeFixedSizePOD<uint32_t>(number_of_components_, buffer);
  arc_utilities::SerializeString(frame_, buffer);
  arc_utilities::SerializeFixedSizePOD<uint8_t>((uint8_t)components_valid_,
                                              buffer);
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

uint64_t CollisionMapGrid::DeserializeSelf(
    const std::vector<uint8_t>& buffer, const uint64_t current,
    const std::function<std::pair<COLLISION_CELL, uint64_t>(
      const std::vector<uint8_t>&, const uint64_t)>& value_deserializer)
{
  UNUSED(value_deserializer);
  uint64_t current_position = current;
  // Deserialize the initialized
  const std::pair<uint8_t, uint64_t> initialized_deserialized
      = arc_utilities::DeserializeFixedSizePOD<uint8_t>(buffer,
                                                        current_position);
  initialized_ = (bool)initialized_deserialized.first;
  current_position += initialized_deserialized.second;
  // Deserialize the transforms
  const std::pair<Eigen::Isometry3d, uint64_t> origin_transform_deserialized
      = arc_utilities::DeserializeEigen<Eigen::Isometry3d>(buffer,
                                                           current_position);
  origin_transform_ = origin_transform_deserialized.first;
  current_position += origin_transform_deserialized.second;
  const std::pair<Eigen::Isometry3d, uint64_t>
      inverse_origin_transform_deserialized
      = arc_utilities::DeserializeEigen<Eigen::Isometry3d>(buffer,
                                                           current_position);
  inverse_origin_transform_ = inverse_origin_transform_deserialized.first;
  current_position += inverse_origin_transform_deserialized.second;
  // Deserialize the data
  const std::pair<std::vector<COLLISION_CELL>, uint64_t> data_deserialized
      = arc_utilities::DeserializeVector<COLLISION_CELL>(
        buffer, current_position,
        arc_utilities::DeserializeFixedSizePOD<COLLISION_CELL>);
  data_ = data_deserialized.first;
  current_position += data_deserialized.second;
  // Deserialize the cell sizes
  const std::pair<double, uint64_t> cell_x_size_deserialized
      = arc_utilities::DeserializeFixedSizePOD<double>(buffer,
                                                       current_position);
  cell_x_size_ = cell_x_size_deserialized.first;
  current_position += cell_x_size_deserialized.second;
  const std::pair<double, uint64_t> cell_y_size_deserialized
      = arc_utilities::DeserializeFixedSizePOD<double>(buffer,
                                                       current_position);
  cell_y_size_ = cell_y_size_deserialized.first;
  current_position += cell_y_size_deserialized.second;
  const std::pair<double, uint64_t> cell_z_size_deserialized
      = arc_utilities::DeserializeFixedSizePOD<double>(buffer,
                                                       current_position);
  cell_z_size_ = cell_z_size_deserialized.first;
  current_position += cell_z_size_deserialized.second;
  const std::pair<double, uint64_t> inv_cell_x_size_deserialized
      = arc_utilities::DeserializeFixedSizePOD<double>(buffer,
                                                       current_position);
  inv_cell_x_size_ = inv_cell_x_size_deserialized.first;
  current_position += inv_cell_x_size_deserialized.second;
  const std::pair<double, uint64_t> inv_cell_y_size_deserialized
      = arc_utilities::DeserializeFixedSizePOD<double>(buffer,
                                                       current_position);
  inv_cell_y_size_ = inv_cell_y_size_deserialized.first;
  current_position += inv_cell_y_size_deserialized.second;
  const std::pair<double, uint64_t> inv_cell_z_size_deserialized
      = arc_utilities::DeserializeFixedSizePOD<double>(buffer,
                                                       current_position);
  inv_cell_z_size_ = inv_cell_z_size_deserialized.first;
  current_position += inv_cell_z_size_deserialized.second;
  // Deserialize the grid sizes
  const std::pair<double, uint64_t> x_size_deserialized
      = arc_utilities::DeserializeFixedSizePOD<double>(buffer,
                                                       current_position);
  x_size_ = x_size_deserialized.first;
  current_position += x_size_deserialized.second;
  const std::pair<double, uint64_t> y_size_deserialized
      = arc_utilities::DeserializeFixedSizePOD<double>(buffer,
                                                       current_position);
  y_size_ = y_size_deserialized.first;
  current_position += y_size_deserialized.second;
  const std::pair<double, uint64_t> z_size_deserialized
      = arc_utilities::DeserializeFixedSizePOD<double>(buffer,
                                                       current_position);
  z_size_ = z_size_deserialized.first;
  current_position += z_size_deserialized.second;
  // Deserialize the control/bounds values
  const std::pair<int64_t, uint64_t> stride1_deserialized
      = arc_utilities::DeserializeFixedSizePOD<int64_t>(buffer,
                                                        current_position);
  stride1_ = stride1_deserialized.first;
  current_position += stride1_deserialized.second;
  const std::pair<int64_t, uint64_t> stride2_deserialized
      = arc_utilities::DeserializeFixedSizePOD<int64_t>(buffer,
                                                        current_position);
  stride2_ = stride2_deserialized.first;
  current_position += stride2_deserialized.second;
  const std::pair<int64_t, uint64_t> num_x_cells_deserialized
      = arc_utilities::DeserializeFixedSizePOD<int64_t>(buffer,
                                                      current_position);
  num_x_cells_ = num_x_cells_deserialized.first;
  current_position += num_x_cells_deserialized.second;
  const std::pair<int64_t, uint64_t> num_y_cells_deserialized
      = arc_utilities::DeserializeFixedSizePOD<int64_t>(buffer,
                                                        current_position);
  num_y_cells_ = num_y_cells_deserialized.first;
  current_position += num_y_cells_deserialized.second;
  const std::pair<int64_t, uint64_t> num_z_cells_deserialized
      = arc_utilities::DeserializeFixedSizePOD<int64_t>(buffer,
                                                      current_position);
  num_z_cells_ = num_z_cells_deserialized.first;
  current_position += num_z_cells_deserialized.second;
  // Deserialize the default value
  const std::pair<COLLISION_CELL, uint64_t> default_value_deserialized
      = arc_utilities::DeserializeFixedSizePOD<COLLISION_CELL>(buffer,
                                                               current_position);
  default_value_ = default_value_deserialized.first;
  current_position += default_value_deserialized.second;
  // Deserialize the OOB value
  const std::pair<COLLISION_CELL, uint64_t> oob_value_deserialized
      = arc_utilities::DeserializeFixedSizePOD<COLLISION_CELL>(buffer,
                                                               current_position);
  oob_value_ = oob_value_deserialized.first;
  current_position += oob_value_deserialized.second;
  // Deserialize CollisionMapGrid stuff
  const std::pair<uint32_t, uint64_t> number_of_components_deserialized
      = arc_utilities::DeserializeFixedSizePOD<uint32_t>(buffer,
                                                       current_position);
  number_of_components_ = number_of_components_deserialized.first;
  current_position += number_of_components_deserialized.second;
  const std::pair<std::string, uint64_t> frame_deserialized
      = arc_utilities::DeserializeString<char>(buffer, current_position);
  frame_ = frame_deserialized.first;
  current_position += frame_deserialized.second;
  const std::pair<uint8_t, uint64_t> components_valid_deserialized
      = arc_utilities::DeserializeFixedSizePOD<uint8_t>(buffer, current_position);
  components_valid_ = (bool)components_valid_deserialized.first;
  current_position += components_valid_deserialized.second;
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - current;
  return bytes_read;
}

void CollisionMapGrid::SaveToFile(
    const CollisionMapGrid& map,
    const std::string& filepath,
    const bool compress)
{
  std::vector<uint8_t> buffer;
  map.SerializeSelf(buffer);
  std::ofstream output_file(filepath, std::ios::out|std::ios::binary);
  if (compress)
  {
    output_file.write("CMGZ", 4);
    const std::vector<uint8_t> compressed = ZlibHelpers::CompressBytes(buffer);
    const size_t serialized_size = compressed.size();
    output_file.write(reinterpret_cast<const char*>(
                        compressed.data()), (std::streamsize)serialized_size);
  }
  else
  {
    output_file.write("CMGR", 4);
    const size_t serialized_size = buffer.size();
    output_file.write(reinterpret_cast<const char*>(
                        buffer.data()), (std::streamsize)serialized_size);
  }
  output_file.close();
}

CollisionMapGrid CollisionMapGrid::LoadFromFile(
    const std::string& filepath)
{
  std::ifstream input_file(filepath, std::ios::in|std::ios::binary);
  if (input_file.good() == false)
  {
    throw std::invalid_argument("File does not exist");
  }
  input_file.seekg(0, std::ios::end);
  std::streampos end = input_file.tellg();
  input_file.seekg(0, std::ios::beg);
  std::streampos begin = input_file.tellg();
  const std::streamsize serialized_size = end - begin;
  const std::streamsize header_size = 4;
  if (serialized_size >= header_size)
  {
    // Load the header
    std::vector<uint8_t> file_header(header_size + 1, 0x00);
    input_file.read(reinterpret_cast<char*>(file_header.data()),
                    header_size);
    const std::string header_string(
          reinterpret_cast<const char*>(file_header.data()));
    // Load the rest of the file
    std::vector<uint8_t> file_buffer(
          (size_t)serialized_size - header_size, 0x00);
    input_file.read(reinterpret_cast<char*>(file_buffer.data()),
                    serialized_size - header_size);
    // Deserialize
    if (header_string == "CMGZ")
    {
      const std::vector<uint8_t> decompressed
          = ZlibHelpers::DecompressBytes(file_buffer);
      CollisionMapGrid map;
      map.DeserializeSelf(decompressed, 0);
      return map;
    }
    else if (header_string == "CMGR")
    {
      CollisionMapGrid map;
      map.DeserializeSelf(file_buffer, 0);
      return map;
    }
    else
    {
      throw std::invalid_argument(
            "File has invalid header [" + header_string + "]");
    }
  }
  else
  {
    throw std::invalid_argument("File is too small");
  }
}

sdf_tools::CollisionMap CollisionMapGrid::GetMessageRepresentation(
    const CollisionMapGrid& map)
{
  sdf_tools::CollisionMap map_message;
  map_message.header.stamp = ros::Time::now();
  map_message.header.frame_id = map.GetFrame();
  std::vector<uint8_t> buffer;
  map.SerializeSelf(buffer);
  map_message.serialized_map = ZlibHelpers::CompressBytes(buffer);
  map_message.is_compressed = true;
  return map_message;
}

CollisionMapGrid CollisionMapGrid::LoadFromMessageRepresentation(
    const sdf_tools::CollisionMap& message)
{
  if (message.is_compressed)
  {
    const std::vector<uint8_t> uncompressed_map
        = ZlibHelpers::DecompressBytes(message.serialized_map);
    CollisionMapGrid map;
    map.DeserializeSelf(uncompressed_map, 0);
    return map;
  }
  else
  {
    CollisionMapGrid map;
    map.DeserializeSelf(message.serialized_map, 0);
    return map;
  }
}

visualization_msgs::Marker CollisionMapGrid::ExportForDisplay(
    const std_msgs::ColorRGBA& collision_color,
    const std_msgs::ColorRGBA& free_color,
    const std_msgs::ColorRGBA& unknown_color) const
{
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
  display_rep.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(
                       GetOriginTransform());
  display_rep.scale.x = GetResolution();
  display_rep.scale.y = GetResolution();
  display_rep.scale.z = GetResolution();
  // Add all the cells of the SDF to the message
  for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < GetNumZCells(); z_index++)
      {
        // Convert grid indices into a real-world location
        const Eigen::Vector4d location
            = GridIndexToLocationGridFrame(x_index, y_index, z_index);
        geometry_msgs::Point new_point;
        new_point.x = location(0);
        new_point.y = location(1);
        new_point.z = location(2);
        if (GetImmutable(x_index, y_index, z_index).first.occupancy > 0.5)
        {
          if (collision_color.a > 0.0)
          {
            display_rep.points.push_back(new_point);
            display_rep.colors.push_back(collision_color);
          }
        }
        else if (GetImmutable(x_index, y_index, z_index).first.occupancy < 0.5)
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

visualization_msgs::MarkerArray CollisionMapGrid::ExportForSeparateDisplay(
    const std_msgs::ColorRGBA& collision_color,
    const std_msgs::ColorRGBA& free_color,
    const std_msgs::ColorRGBA& unknown_color) const
{
  const std_msgs::ColorRGBA no_color
      = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>
        ::MakeFromFloatColors(0.0, 0.0, 0.0, 0.0);
  visualization_msgs::Marker collision_only_marker
      = ExportForDisplay(collision_color, no_color, no_color);
  collision_only_marker.ns = "collision_only";
  visualization_msgs::Marker free_only_marker
      = ExportForDisplay(no_color, free_color, no_color);
  free_only_marker.ns = "free_only";
  visualization_msgs::Marker unknown_only_marker
      = ExportForDisplay(no_color, no_color, unknown_color);
  unknown_only_marker.ns = "unknown_only";
  visualization_msgs::MarkerArray display_messages;
  display_messages.markers = {collision_only_marker,
                              free_only_marker,
                              unknown_only_marker};
  return display_messages;
}

visualization_msgs::Marker CollisionMapGrid::ExportSurfacesForDisplay(
    const std_msgs::ColorRGBA& collision_color,
    const std_msgs::ColorRGBA& free_color,
    const std_msgs::ColorRGBA& unknown_color) const
{
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
  display_rep.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(
                       GetOriginTransform());
  display_rep.scale.x = GetResolution();
  display_rep.scale.y = GetResolution();
  display_rep.scale.z = GetResolution();
  // Add all the cells of the SDF to the message
  for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < GetNumZCells(); z_index++)
      {
        if (IsSurfaceIndex(x_index, y_index, z_index))
        {
          // Convert grid indices into a real-world location
          const Eigen::Vector4d location
              = GridIndexToLocationGridFrame(x_index, y_index, z_index);
          geometry_msgs::Point new_point;
          new_point.x = location(0);
          new_point.y = location(1);
          new_point.z = location(2);
          if (GetImmutable(x_index, y_index, z_index).first.occupancy > 0.5)
          {
            if (collision_color.a > 0.0)
            {
              display_rep.points.push_back(new_point);
              display_rep.colors.push_back(collision_color);
            }
          }
          else if (GetImmutable(x_index, y_index, z_index).first.occupancy
                   < 0.5)
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

visualization_msgs::MarkerArray
CollisionMapGrid::ExportSurfacesForSeparateDisplay(
    const std_msgs::ColorRGBA& collision_color,
    const std_msgs::ColorRGBA& free_color,
    const std_msgs::ColorRGBA& unknown_color) const
{
  const std_msgs::ColorRGBA no_color
      = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>
        ::MakeFromFloatColors(0.0, 0.0, 0.0, 0.0);
  visualization_msgs::Marker collision_only_marker
      = ExportSurfacesForDisplay(collision_color, no_color, no_color);
  collision_only_marker.ns = "collision_surfaces_only";
  visualization_msgs::Marker free_only_marker
      = ExportSurfacesForDisplay(no_color, free_color, no_color);
  free_only_marker.ns = "free_surfaces_only";
  visualization_msgs::Marker unknown_only_marker
      = ExportSurfacesForDisplay(no_color, no_color, unknown_color);
  unknown_only_marker.ns = "unknown_surfaces_only";
  visualization_msgs::MarkerArray display_messages;
  display_messages.markers = {collision_only_marker,
                              free_only_marker,
                              unknown_only_marker};
  return display_messages;
}

visualization_msgs::Marker
CollisionMapGrid::ExportConnectedComponentsForDisplay(
    const bool color_unknown_components) const
{
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
  display_rep.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(
                       GetOriginTransform());
  display_rep.scale.x = GetResolution();
  display_rep.scale.y = GetResolution();
  display_rep.scale.z = GetResolution();
  // Add all the cells of the SDF to the message
  for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < GetNumZCells(); z_index++)
      {
        // Convert grid indices into a real-world location
        const Eigen::Vector4d location
            = GridIndexToLocationGridFrame(x_index, y_index, z_index);
        geometry_msgs::Point new_point;
        new_point.x = location(0);
        new_point.y = location(1);
        new_point.z = location(2);
        display_rep.points.push_back(new_point);
        const COLLISION_CELL current_cell
            = GetImmutable(x_index, y_index, z_index).first;
        if (current_cell.occupancy != 0.5)
        {
          std_msgs::ColorRGBA color
              = GenerateComponentColor(current_cell.component);
          display_rep.colors.push_back(color);
        }
        else
        {
          if (color_unknown_components)
          {
            std_msgs::ColorRGBA color
                = GenerateComponentColor(current_cell.component);
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
  const std::function<bool(const GRID_INDEX&, const GRID_INDEX&)>
    are_connected_fn = [&] (const GRID_INDEX& index1, const GRID_INDEX& index2)
  {
    auto query1 = GetImmutable(index1);
    auto query2 = GetImmutable(index2);
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
  const std::function<int64_t(const GRID_INDEX&)> get_component_fn
      = [&] (const GRID_INDEX& index)
  {
    auto query = GetImmutable(index);
    if (query.second)
    {
      return (int64_t)query.first.component;
    }
    else
    {
      return (int64_t)-1;
    }
  };
  const std::function<void(const GRID_INDEX&, const uint32_t)> mark_component_fn
      = [&] (const GRID_INDEX& index, const uint32_t component)
  {
    auto query = GetMutable(index);
    if (query.second)
    {
      SetValue(index, COLLISION_CELL(query.first.occupancy, component));
    }
  };
  number_of_components_
      = topology_computation::ComputeConnectedComponents(*this,
                                                         are_connected_fn,
                                                         get_component_fn,
                                                         mark_component_fn);
  components_valid_ = true;
  return number_of_components_;
}

std::map<uint32_t, std::pair<int32_t, int32_t>>
CollisionMapGrid::ComputeComponentTopology(
    const bool ignore_empty_components,
    const bool recompute_connected_components,
    const bool verbose)
{
  // Recompute the connected components if need be
  if (recompute_connected_components)
  {
    UpdateConnectedComponents();
  }
  const std::function<int64_t(const GRID_INDEX&)> get_component_fn
      = [&] (const GRID_INDEX& index)
  {
    auto query = GetImmutable(index);
    if (query.second)
    {
      return (int64_t)query.first.component;
    }
    else
    {
      return (int64_t)-1;
    }
  };
  const std::function<bool(const GRID_INDEX&)> is_surface_index_fn
      = [&] (const GRID_INDEX& index)
  {
    if (ignore_empty_components)
    {
      const COLLISION_CELL& current_cell = GetImmutable(index).first;
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
  return topology_computation::ComputeComponentTopology(*this,
                                                        get_component_fn,
                                                        is_surface_index_fn,
                                                        verbose);
}

CollisionMapGrid CollisionMapGrid::Resample(const double new_resolution) const
{
  CollisionMapGrid resampled(GetOriginTransform(),
                             GetFrame(),
                             new_resolution,
                             GetXSize(), GetYSize(), GetZSize(),
                             GetOOBValue());
  for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < GetNumZCells(); z_index++)
      {
        const COLLISION_CELL& current_cell
            = GetImmutable(x_index, y_index, z_index).first;
        const Eigen::Vector4d current_cell_location
            = GridIndexToLocation(x_index, y_index, z_index);
        resampled.SetValue4d(current_cell_location, current_cell);
      }
    }
  }
  return resampled;
}

std::map<uint32_t, std::unordered_map<GRID_INDEX, uint8_t>>
CollisionMapGrid::ExtractComponentSurfaces(
    const COMPONENT_TYPES component_types_to_extract) const
{
  // Make the helper functions
  const std::function<int64_t(const GRID_INDEX&)> get_component_fn
      = [&] (const GRID_INDEX& index)
  {
    auto query = GetImmutable(index);
    if (query.second)
    {
      return (int64_t)query.first.component;
    }
    else
    {
      return (int64_t)-1;
    }
  };
  const std::function<bool(const GRID_INDEX&)> is_surface_index_fn
      = [&] (const GRID_INDEX& index)
  {
    const COLLISION_CELL& current_cell = GetImmutable(index).first;
    if (current_cell.occupancy > 0.5)
    {
      if ((component_types_to_extract & FILLED_COMPONENTS) > 0x00)
      {
        if (IsConnectedComponentSurfaceIndex(index.x, index.y, index.y))
        {
          return true;
        }
      }
    }
    else if (current_cell.occupancy < 0.5)
    {
      if ((component_types_to_extract & EMPTY_COMPONENTS) > 0x00)
      {
        if (IsConnectedComponentSurfaceIndex(index.x, index.y, index.z))
        {
          return true;
        }
      }
    }
    else
    {
      if ((component_types_to_extract & UNKNOWN_COMPONENTS) > 0x00)
      {
        if (IsConnectedComponentSurfaceIndex(index.x, index.z, index.z))
        {
          return true;
        }
      }
    }
    return false;
  };
  return topology_computation::ExtractComponentSurfaces(*this,
                                                        get_component_fn,
                                                        is_surface_index_fn);
}

std::vector<std::vector<VoxelGrid::GRID_INDEX>>
CollisionMapGrid::ExtractConnectedComponents()
{
  if (!components_valid_)
  {
    UpdateConnectedComponents();
  }
  std::vector<std::vector<GRID_INDEX>> components(number_of_components_);
  for (int64_t x_idx = 0; x_idx < num_x_cells_; ++x_idx)
  {
    for (int64_t y_idx = 0; y_idx < num_y_cells_; ++y_idx)
    {
      for (int64_t z_idx = 0; z_idx < num_z_cells_; ++z_idx)
      {
        const GRID_INDEX index(x_idx, y_idx, z_idx);
        const auto component = GetImmutable(index).first.component;
        // Note that componnet 0 means "not marked at all"
        components.at(component - 1).push_back(index);
      }
    }
  }
  return components;
}

