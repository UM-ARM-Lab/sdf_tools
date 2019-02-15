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
#include <sdf_tools/tagged_object_collision_map.hpp>
#include <arc_utilities/zlib_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/simple_kmeans_clustering.hpp>
#include <sdf_tools/TaggedObjectCollisionMap.h>

using namespace sdf_tools;

uint64_t TaggedObjectCollisionMapGrid::SerializeSelf(
    std::vector<uint8_t>& buffer,
    const std::function<uint64_t(
      const TAGGED_OBJECT_COLLISION_CELL&,
      std::vector<uint8_t>&)>& value_serializer) const
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
  arc_utilities::SerializeVector<TAGGED_OBJECT_COLLISION_CELL>(
        data_, buffer,
        arc_utilities::SerializeFixedSizePOD<TAGGED_OBJECT_COLLISION_CELL>);
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
  arc_utilities::SerializeFixedSizePOD<TAGGED_OBJECT_COLLISION_CELL>(
        default_value_, buffer);
  // Serialize the OOB value
  arc_utilities::SerializeFixedSizePOD<TAGGED_OBJECT_COLLISION_CELL>(
        oob_value_, buffer);
  // Serialize TaggedObjectCollisionMapGrid stuff
  arc_utilities::SerializeFixedSizePOD<uint32_t>(number_of_components_, buffer);
  arc_utilities::SerializeFixedSizePOD<uint32_t>(number_of_convex_segments_, buffer);
  arc_utilities::SerializeString(frame_, buffer);
  arc_utilities::SerializeFixedSizePOD<uint8_t>((uint8_t)components_valid_,
                                              buffer);
  arc_utilities::SerializeFixedSizePOD<uint8_t>((uint8_t)convex_segments_valid_,
                                              buffer);
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

uint64_t TaggedObjectCollisionMapGrid::DeserializeSelf(
    const std::vector<uint8_t>& buffer, const uint64_t current,
    const std::function<std::pair<TAGGED_OBJECT_COLLISION_CELL, uint64_t>(
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
  const std::pair<std::vector<TAGGED_OBJECT_COLLISION_CELL>, uint64_t>
      data_deserialized
      = arc_utilities::DeserializeVector<TAGGED_OBJECT_COLLISION_CELL>(
        buffer, current_position,
        arc_utilities::DeserializeFixedSizePOD<TAGGED_OBJECT_COLLISION_CELL>);
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
  const std::pair<TAGGED_OBJECT_COLLISION_CELL, uint64_t>
      default_value_deserialized
      = arc_utilities::DeserializeFixedSizePOD<TAGGED_OBJECT_COLLISION_CELL>(
          buffer, current_position);
  default_value_ = default_value_deserialized.first;
  current_position += default_value_deserialized.second;
  // Deserialize the OOB value
  const std::pair<TAGGED_OBJECT_COLLISION_CELL, uint64_t>
      oob_value_deserialized
      = arc_utilities::DeserializeFixedSizePOD<TAGGED_OBJECT_COLLISION_CELL>(
          buffer, current_position);
  oob_value_ = oob_value_deserialized.first;
  current_position += oob_value_deserialized.second;
  // Deserialize CollisionMapGrid stuff
  const std::pair<uint32_t, uint64_t> number_of_components_deserialized
      = arc_utilities::DeserializeFixedSizePOD<uint32_t>(buffer,
                                                         current_position);
  number_of_components_ = number_of_components_deserialized.first;
  current_position += number_of_components_deserialized.second;
  const std::pair<uint32_t, uint64_t> number_of_convex_segments_deserialized
      = arc_utilities::DeserializeFixedSizePOD<uint32_t>(buffer,
                                                         current_position);
  number_of_convex_segments_ = number_of_convex_segments_deserialized.first;
  current_position += number_of_convex_segments_deserialized.second;
  const std::pair<std::string, uint64_t> frame_deserialized
      = arc_utilities::DeserializeString<char>(buffer, current_position);
  frame_ = frame_deserialized.first;
  current_position += frame_deserialized.second;
  const std::pair<uint8_t, uint64_t> components_valid_deserialized
      = arc_utilities::DeserializeFixedSizePOD<uint8_t>(buffer, current_position);
  components_valid_ = (bool)components_valid_deserialized.first;
  current_position += components_valid_deserialized.second;
  const std::pair<uint8_t, uint64_t> convex_segments_valid_deserialized
      = arc_utilities::DeserializeFixedSizePOD<uint8_t>(buffer, current_position);
  convex_segments_valid_ = (bool)convex_segments_valid_deserialized.first;
  current_position += convex_segments_valid_deserialized.second;
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - current;
  return bytes_read;
}

void TaggedObjectCollisionMapGrid::SaveToFile(
    const TaggedObjectCollisionMapGrid& map,
    const std::string& filepath,
    const bool compress)
{
  std::vector<uint8_t> buffer;
  map.SerializeSelf(buffer);
  std::ofstream output_file(filepath, std::ios::out|std::ios::binary);
  if (compress)
  {
    output_file.write("TCMZ", 4);
    const std::vector<uint8_t> compressed = ZlibHelpers::CompressBytes(buffer);
    const size_t serialized_size = compressed.size();
    output_file.write(reinterpret_cast<const char*>(
                        compressed.data()), (std::streamsize)serialized_size);
  }
  else
  {
    output_file.write("TCMR", 4);
    const size_t serialized_size = buffer.size();
    output_file.write(reinterpret_cast<const char*>(
                        buffer.data()), (std::streamsize)serialized_size);
  }
  output_file.close();
}

TaggedObjectCollisionMapGrid TaggedObjectCollisionMapGrid::LoadFromFile(
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
    if (header_string == "TCMZ")
    {
      const std::vector<uint8_t> decompressed
          = ZlibHelpers::DecompressBytes(file_buffer);
      TaggedObjectCollisionMapGrid map;
      map.DeserializeSelf(decompressed, 0);
      return map;
    }
    else if (header_string == "TCMR")
    {
      TaggedObjectCollisionMapGrid map;
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

sdf_tools::TaggedObjectCollisionMap
TaggedObjectCollisionMapGrid::GetMessageRepresentation(
    const TaggedObjectCollisionMapGrid& map)
{
  sdf_tools::TaggedObjectCollisionMap map_message;
  map_message.header.stamp = ros::Time::now();
  map_message.header.frame_id = map.GetFrame();
  std::vector<uint8_t> buffer;
  map.SerializeSelf(buffer);
  map_message.serialized_map = ZlibHelpers::CompressBytes(buffer);
  map_message.is_compressed = true;
  return map_message;
}

TaggedObjectCollisionMapGrid
TaggedObjectCollisionMapGrid::LoadFromMessageRepresentation(
    const sdf_tools::TaggedObjectCollisionMap& message)
{
  if (message.is_compressed)
  {
    const std::vector<uint8_t> uncompressed_map
        = ZlibHelpers::DecompressBytes(message.serialized_map);
    TaggedObjectCollisionMapGrid map;
    map.DeserializeSelf(uncompressed_map, 0);
    return map;
  }
  else
  {
    TaggedObjectCollisionMapGrid map;
    map.DeserializeSelf(message.serialized_map, 0);
    return map;
  }
}

uint32_t TaggedObjectCollisionMapGrid::UpdateConnectedComponents()
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
      SetValue(index, TAGGED_OBJECT_COLLISION_CELL(query.first.occupancy,
                                                   query.first.object_id,
                                                   component,
                                                   query.first.convex_segment));
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

TaggedObjectCollisionMapGrid
TaggedObjectCollisionMapGrid::Resample(const double new_resolution) const
{
  TaggedObjectCollisionMapGrid resampled(GetOriginTransform(),
                                         GetFrame(),
                                         new_resolution,
                                         GetXSize(), GetYSize(), GetZSize(),
                                         GetOOBValue());
  for (int64_t x_idx = 0; x_idx < GetNumXCells(); x_idx++)
  {
    for (int64_t y_idx = 0; y_idx < GetNumYCells(); y_idx++)
    {
      for (int64_t z_idx = 0; z_idx < GetNumZCells(); z_idx++)
      {
        const TAGGED_OBJECT_COLLISION_CELL& current_cell
            = GetImmutable(x_idx, y_idx, z_idx).first;
        const Eigen::Vector4d current_cell_location
            = GridIndexToLocation(x_idx, y_idx, z_idx);
        resampled.SetValue4d(current_cell_location, current_cell);
      }
    }
  }
  return resampled;
}

std::map<uint32_t, std::pair<int32_t, int32_t>>
TaggedObjectCollisionMapGrid::ComputeComponentTopology(
    const COMPONENT_TYPES component_types_to_use,
    const bool recompute_connected_components,
    const bool verbose)
{
  // Recompute the connected components if need be
  if (recompute_connected_components)
  {
    UpdateConnectedComponents();
  }
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
    const TAGGED_OBJECT_COLLISION_CELL& current_cell
        = GetImmutable(index).first;
    if (current_cell.occupancy > 0.5)
    {
      if ((component_types_to_use & FILLED_COMPONENTS) > 0x00)
      {
        if (IsConnectedComponentSurfaceIndex(index.x, index.y, index.y))
        {
          return true;
        }
      }
    }
    else if (current_cell.occupancy < 0.5)
    {
      if ((component_types_to_use & EMPTY_COMPONENTS) > 0x00)
      {
        if (IsConnectedComponentSurfaceIndex(index.x, index.y, index.z))
        {
          return true;
        }
      }
    }
    else
    {
      if ((component_types_to_use & UNKNOWN_COMPONENTS) > 0x00)
      {
        if (IsConnectedComponentSurfaceIndex(index.x, index.z, index.z))
        {
          return true;
        }
      }
    }
    return false;
  };
  return topology_computation::ComputeComponentTopology(*this,
                                                        get_component_fn,
                                                        is_surface_index_fn,
                                                        verbose);
}

std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>>
TaggedObjectCollisionMapGrid::ExtractComponentSurfaces(
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
    const TAGGED_OBJECT_COLLISION_CELL& current_cell
        = GetImmutable(index).first;
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

uint32_t TaggedObjectCollisionMapGrid::UpdateConvexSegments(
    const double connected_threshold,
    const bool add_virtual_border)
{
  const auto sdf_result
      = (add_virtual_border) ?
          ExtractSignedDistanceField(
            std::numeric_limits<float>::infinity(),
            std::vector<uint32_t>(),
            true, true) :
          ExtractFreeAndNamedObjectsSignedDistanceField(
            std::numeric_limits<float>::infinity(), true);
  const SignedDistanceField& sdf = sdf_result.first;
  const VoxelGrid<Eigen::Vector3d> extrema_map = sdf.ComputeLocalExtremaMap();
  // Make the helper functions
  // This is not enough, we also need to limit the curvature of the
  // segment/local extrema cluster! Otherwise thin objects will always have
  // their local extrema dominated by their surroundings rather than their
  // own structure!
  const std::function<bool(const GRID_INDEX&, const GRID_INDEX&)>
    are_connected_fn
      = [&] (const GRID_INDEX& index1, const GRID_INDEX& index2)
  {
    auto query1 = GetImmutable(index1);
    auto query2 = GetImmutable(index2);
    assert(query1.second);
    assert(query2.second);
    if (query1.first.object_id == query2.first.object_id)
    {
      auto exmap_query1 = extrema_map.GetImmutable(index1);
      auto examp_query2 = extrema_map.GetImmutable(index2);
      assert(exmap_query1.second);
      assert(examp_query2.second);
      const double maxima_distance
          = (exmap_query1.first - examp_query2.first).norm();
      if (maxima_distance < connected_threshold)
      {
        return true;
      }
      else
      {
        return false;
      }
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
    auto extrema_query = extrema_map.GetImmutable(index);
    if (query.second)
    {
      if ((query.first.occupancy < 0.5f) || (query.first.object_id > 0u))
      {
        const Eigen::Vector3d& extrema = extrema_query.first;
        if (!std::isinf(extrema.x())
            && !std::isinf(extrema.y())
            && !std::isinf(extrema.z()))
        {
          return (int64_t)query.first.convex_segment;
        }
        else
        {
          // Ignore cells with infinite extrema
          return (int64_t)-1;
        }
      }
      else
      {
        // Ignore filled cells that don't belong to an object
        return (int64_t)-1;
      }
    }
    else
    {
      return (int64_t)-1;
    }
  };
  const std::function<void(const GRID_INDEX&, const uint32_t)>
    mark_component_fn
      = [&] (const GRID_INDEX& index, const uint32_t component)
  {
    auto query = GetMutable(index);
    if (query.second)
    {
      SetValue(index, TAGGED_OBJECT_COLLISION_CELL(query.first.occupancy,
                                                   query.first.object_id,
                                                   query.first.component,
                                                   component));
    }
  };
  number_of_convex_segments_
      = topology_computation::ComputeConnectedComponents(*this,
                                                         are_connected_fn,
                                                         get_component_fn,
                                                         mark_component_fn);
  convex_segments_valid_ = true;
  return number_of_convex_segments_;
}

////////////////////////////////////////////////////////////////////////////////
// Visualization
////////////////////////////////////////////////////////////////////////////////

// Note that this does not fill out the namespace field
visualization_msgs::Marker TaggedObjectCollisionMapGrid::DefaultMarker() const
{
    visualization_msgs::Marker m;
    // Populate the header
    m.header.frame_id = frame_;
    // Populate the options
    m.id = 1;
    m.type = visualization_msgs::Marker::CUBE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(0.0);
    m.frame_locked = false;
    m.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(
            GetOriginTransform());
    m.scale.x = GetResolution();
    m.scale.y = GetResolution();
    m.scale.z = GetResolution();
    return m;
}

visualization_msgs::Marker TaggedObjectCollisionMapGrid::ExportForDisplay(
    const float alpha,
    const std::vector<uint32_t>& objects_to_draw) const
{
  const bool draw_all = objects_to_draw.empty();
  std::map<uint32_t, uint32_t> objects_to_draw_map;
  for (size_t idx = 0; idx < objects_to_draw.size(); idx++)
  {
    objects_to_draw_map[objects_to_draw[idx]] = 1u;
  }
  auto display_rep = DefaultMarker();
  display_rep.ns = "tagged_object_collision_map_display";
  // Add all the cells of the SDF to the message
  for (int64_t x_idx = 0; x_idx < GetNumXCells(); x_idx++)
  {
    for (int64_t y_idx = 0; y_idx < GetNumYCells(); y_idx++)
    {
      for (int64_t z_idx = 0; z_idx < GetNumZCells(); z_idx++)
      {
        const auto& cell = GetImmutable(x_idx, y_idx, z_idx).first;
        const auto draw_found_itr = objects_to_draw_map.find(cell.object_id);
        if (draw_all || draw_found_itr != objects_to_draw_map.end())
        {
          const auto object_color =
              GenerateComponentColor(cell.object_id, alpha);
          // This check removes the background as GenerateComponentColor(0)
          // returns an alpha value of 0
          if (object_color.a > 0.0)
          {
            // Convert grid indices into a real-world location
            const auto location =
                GridIndexToLocationGridFrame(x_idx, y_idx, z_idx);
            geometry_msgs::Point new_point;
            new_point.x = location(0);
            new_point.y = location(1);
            new_point.z = location(2);
            display_rep.points.push_back(new_point);
            display_rep.colors.push_back(object_color);
          }
        }
      }
    }
  }
  return display_rep;
}

visualization_msgs::MarkerArray
TaggedObjectCollisionMapGrid::ExportForDisplayUniqueNs(
    const float alpha,
    const std::vector<uint32_t>& objects_to_draw) const
{
  static const std::string ns_base = "tagged_object_collision_map_display_";
  const bool draw_all = objects_to_draw.empty();
  // Track the position of each object in display_rep.markers
  std::map<uint32_t, uint32_t> objects_to_draw_map;
  visualization_msgs::MarkerArray display_rep;
  for (size_t idx = 0; idx < objects_to_draw.size(); idx++)
  {
    const auto object_id = objects_to_draw[idx];
    objects_to_draw_map[object_id] = (uint32_t)idx;
    display_rep.markers.push_back(DefaultMarker());
    display_rep.markers.back().ns = ns_base + std::to_string(object_id);
  }
  // Add all the cells of the SDF to the message
  for (int64_t x_idx = 0; x_idx < GetNumXCells(); x_idx++)
  {
    for (int64_t y_idx = 0; y_idx < GetNumYCells(); y_idx++)
    {
      for (int64_t z_idx = 0; z_idx < GetNumZCells(); z_idx++)
      {
        const auto& cell = GetImmutable(x_idx, y_idx, z_idx).first;
        auto draw_found_itr = objects_to_draw_map.find(cell.object_id);
        // If we should be drawing all objects, then generate a new marker
        if (draw_all && cell.object_id != 0
            && draw_found_itr == objects_to_draw_map.end())
        {
            const auto markers_idx = (uint32_t)objects_to_draw_map.size();
            objects_to_draw_map[cell.object_id] = markers_idx;
            draw_found_itr = objects_to_draw_map.find(cell.object_id);
            display_rep.markers.push_back(DefaultMarker());
            display_rep.markers.back().ns =
                ns_base + std::to_string(cell.object_id);
        }
        // Add the current point to the appropriate marker
        if (draw_found_itr != objects_to_draw_map.end())
        {
          const auto object_color =
              GenerateComponentColor(cell.object_id, alpha);
          // This check removes the background as GenerateComponentColor(0)
          // returns an alpha value of 0
          if (object_color.a > 0.0)
          {
            // Convert grid indices into a real-world location
            const auto location =
                GridIndexToLocationGridFrame(x_idx, y_idx, z_idx);
            geometry_msgs::Point new_point;
            new_point.x = location(0);
            new_point.y = location(1);
            new_point.z = location(2);
            auto& marker = display_rep.markers.at(draw_found_itr->second);
            marker.points.push_back(new_point);
            marker.colors.push_back(object_color);
          }
        }
      }
    }
  }
  // Remove all markers with no points in them
  visualization_msgs::MarkerArray display_rep_pruned;
  for (const auto& marker : display_rep.markers)
  {
      if (!marker.points.empty())
      {
          display_rep_pruned.markers.push_back(marker);
      }
  }
  return display_rep_pruned;
}

visualization_msgs::Marker TaggedObjectCollisionMapGrid::ExportForDisplay(
    std::map<uint32_t, std_msgs::ColorRGBA> color_map) const
{
  auto display_rep = DefaultMarker();
  display_rep.ns = "tagged_object_collision_map_display";
  // Add all the cells of the SDF to the message
  for (int64_t x_idx = 0; x_idx < GetNumXCells(); x_idx++)
  {
    for (int64_t y_idx = 0; y_idx < GetNumYCells(); y_idx++)
    {
      for (int64_t z_idx = 0; z_idx < GetNumZCells(); z_idx++)
      {
        const auto& current_cell = GetImmutable(x_idx, y_idx, z_idx).first;
        // Check if we've been given a color to work with
        auto found_itr = color_map.find(current_cell.object_id);
        // If not, generate one
        if (found_itr == color_map.end())
        {
          color_map[current_cell.object_id] =
              GenerateComponentColor(current_cell.object_id);
          found_itr = color_map.find(current_cell.object_id);
        }
        const std_msgs::ColorRGBA object_color = found_itr->second;
        if (object_color.a > 0.0)
        {
          // Convert grid indices into a real-world location
          const auto location =
              GridIndexToLocationGridFrame(x_idx, y_idx, z_idx);
          geometry_msgs::Point new_point;
          new_point.x = location(0);
          new_point.y = location(1);
          new_point.z = location(2);
          display_rep.points.push_back(new_point);
          display_rep.colors.push_back(object_color);
        }
      }
    }
  }
  return display_rep;
}

visualization_msgs::MarkerArray
TaggedObjectCollisionMapGrid::ExportForDisplayUniqueNs(
    std::map<uint32_t, std_msgs::ColorRGBA> color_map) const
{
  static const std::string ns_base = "tagged_object_collision_map_display_";
  // Track the position of each object in display_rep.markers
  std::map<uint32_t, uint32_t> objects_to_draw_map;
  visualization_msgs::MarkerArray display_rep;
  for (const auto& kv_pair : color_map)
  {
    const auto object_id = kv_pair.first;
    objects_to_draw_map[object_id] = display_rep.markers.size();
    display_rep.markers.push_back(DefaultMarker());
    display_rep.markers.back().ns = ns_base + std::to_string(object_id);
  }
  // Add all the cells of the SDF to the message
  for (int64_t x_idx = 0; x_idx < GetNumXCells(); x_idx++)
  {
    for (int64_t y_idx = 0; y_idx < GetNumYCells(); y_idx++)
    {
      for (int64_t z_idx = 0; z_idx < GetNumZCells(); z_idx++)
      {
        const auto& cell = GetImmutable(x_idx, y_idx, z_idx).first;
        auto draw_found_itr = objects_to_draw_map.find(cell.object_id);
        // If we have not added this marker yet, do so now
        if (cell.object_id != 0
            && draw_found_itr == objects_to_draw_map.end())
        {
            const auto markers_idx = (uint32_t)objects_to_draw_map.size();
            objects_to_draw_map[cell.object_id] = markers_idx;
            draw_found_itr = objects_to_draw_map.find(cell.object_id);
            display_rep.markers.push_back(DefaultMarker());
            display_rep.markers.back().ns =
                ns_base + std::to_string(cell.object_id);
        }
        // Add the current point to the appropriate marker
        if (draw_found_itr != objects_to_draw_map.end())
        {
          // Check if we've been given a color to work with
          auto color_found_itr = color_map.find(cell.object_id);
          // If not, generate one
          if (color_found_itr == color_map.end())
          {
            color_map[cell.object_id] =
                GenerateComponentColor(cell.object_id);
            color_found_itr = color_map.find(cell.object_id);
          }
          const auto object_color = color_found_itr->second;
          if (object_color.a > 0.0)
          {
            // Convert grid indices into a real-world location
            const auto location =
                GridIndexToLocationGridFrame(x_idx, y_idx, z_idx);
            geometry_msgs::Point new_point;
            new_point.x = location(0);
            new_point.y = location(1);
            new_point.z = location(2);
            auto& marker = display_rep.markers.at(draw_found_itr->second);
            marker.points.push_back(new_point);
            marker.colors.push_back(object_color);
          }
        }
      }
    }
  }
  // Remove all markers with no points in them
  visualization_msgs::MarkerArray display_rep_pruned;
  for (const auto& marker : display_rep.markers)
  {
      if (!marker.points.empty())
      {
          display_rep_pruned.markers.push_back(marker);
      }
  }
  return display_rep_pruned;
}

visualization_msgs::Marker
TaggedObjectCollisionMapGrid::ExportContourOnlyForDisplay(
    const float alpha,
    const std::vector<uint32_t>& objects_to_draw) const
{
  const bool draw_all = objects_to_draw.empty();
  // Make SDFs
  const auto per_object_sdfs = draw_all ?
          MakeAllObjectSDFs(true, false)
        : MakeObjectSDFs(objects_to_draw, true, false);
  auto display_rep = DefaultMarker();
  display_rep.ns = "tagged_object_collision_map_display";
  // Add all the cells of the SDF to the message. 1.9 is to ensure that we
  // get all corners, without also getting extra interior cells
  const float bdy_cell_min_dist = -1.9 * GetResolution();
  // Add all the cells of the SDF to the message
  for (int64_t x_idx = 0; x_idx < GetNumXCells(); x_idx++)
  {
    for (int64_t y_idx = 0; y_idx < GetNumYCells(); y_idx++)
    {
      for (int64_t z_idx = 0; z_idx < GetNumZCells(); z_idx++)
      {
        const auto& cell = GetImmutable(x_idx, y_idx, z_idx).first;
        // Get the SDF for the current object
        const auto sdf_found_itr = per_object_sdfs.find(cell.object_id);
        if (sdf_found_itr != per_object_sdfs.end())
        {
          // Extract the distance in the sdf
          const auto& object_sdf = sdf_found_itr->second;
          const float dist = object_sdf.GetImmutable(x_idx, y_idx, z_idx).first;
          // Check if we're on the surface of the object
          if (dist < 0.0 && dist > bdy_cell_min_dist)
          {
            const auto object_color =
                GenerateComponentColor(cell.object_id, alpha);
            if (object_color.a > 0.0)
            {
              // Convert grid indices into a real-world location
              const auto location =
                  GridIndexToLocationGridFrame(x_idx, y_idx, z_idx);
              geometry_msgs::Point new_point;
              new_point.x = location(0);
              new_point.y = location(1);
              new_point.z = location(2);
              display_rep.points.push_back(new_point);
              display_rep.colors.push_back(object_color);
            }
          }
        }
      }
    }
  }
  return display_rep;
}

visualization_msgs::MarkerArray
TaggedObjectCollisionMapGrid::ExportContourOnlyForDisplayUniqueNs(
    const float alpha,
    const std::vector<uint32_t>& objects_to_draw) const
{
  static const std::string ns_base = "tagged_object_collision_map_display_";
  const bool draw_all = objects_to_draw.empty();
  // Make SDFs
  const auto per_object_sdfs = draw_all ?
          MakeAllObjectSDFs(true, false)
        : MakeObjectSDFs(objects_to_draw, true, false);
  // Track the position of each object in display_rep.markers
  visualization_msgs::MarkerArray display_rep;
  std::map<uint32_t, uint32_t> objects_to_draw_map;
  for (const auto& kv_pair : per_object_sdfs)
  {
    const auto object_id = kv_pair.first;
    objects_to_draw_map[object_id] = display_rep.markers.size();
    display_rep.markers.push_back(DefaultMarker());
    display_rep.markers.back().ns = ns_base + std::to_string(object_id);
  }
  // Add all the cells of the SDF to the message. 1.9 is to ensure that we
  // get all corners, without also getting extra interior cells
  const float bdy_cell_min_dist = -1.9 * GetResolution();
  // Add all the cells of the SDF to the message
  for (int64_t x_idx = 0; x_idx < GetNumXCells(); x_idx++)
  {
    for (int64_t y_idx = 0; y_idx < GetNumYCells(); y_idx++)
    {
      for (int64_t z_idx = 0; z_idx < GetNumZCells(); z_idx++)
      {
        const auto& cell = GetImmutable(x_idx, y_idx, z_idx).first;
        // Get the SDF for the current object
        const auto sdf_found_itr = per_object_sdfs.find(cell.object_id);
        if (sdf_found_itr != per_object_sdfs.end())
        {
          // Extract the distance in the sdf
          const auto& object_sdf = sdf_found_itr->second;
          const float dist = object_sdf.GetImmutable(x_idx, y_idx, z_idx).first;
          // Check if we're on the surface of the object
          if (dist < 0.0 && dist > bdy_cell_min_dist)
          {
            const auto object_color =
                GenerateComponentColor(cell.object_id, alpha);
            if (object_color.a > 0.0)
            {
              // Convert grid indices into a real-world location
              const auto location =
                  GridIndexToLocationGridFrame(x_idx, y_idx, z_idx);
              geometry_msgs::Point new_point;
              new_point.x = location(0);
              new_point.y = location(1);
              new_point.z = location(2);
              const auto marker_idx = objects_to_draw_map.at(cell.object_id);
              auto& marker = display_rep.markers.at(marker_idx);
              marker.points.push_back(new_point);
              marker.colors.push_back(object_color);
            }
          }
        }
      }
    }
  }
  // Remove all markers with no points in them
  visualization_msgs::MarkerArray display_rep_pruned;
  for (const auto& marker : display_rep.markers)
  {
      if (!marker.points.empty())
      {
          display_rep_pruned.markers.push_back(marker);
      }
  }
  return display_rep_pruned;
}

// Note that this function will use a default color for objects that a color
// is not provided for (other than object ID 0, which is ignored by
// MakeAllObjectSDFs)
visualization_msgs::Marker
TaggedObjectCollisionMapGrid::ExportContourOnlyForDisplay(
    std::map<uint32_t, std_msgs::ColorRGBA> color_map) const
{
  // Make SDFs for the objects that we will be displaying - filters out id 0
  const auto per_object_sdfs = MakeAllObjectSDFs(true, false);
  // Create a color if needed
  for (const auto& kv_pair : per_object_sdfs)
  {
    const auto object_id = kv_pair.first;
    const auto color_found_itr = color_map.find(object_id);
    if (color_found_itr == color_map.end())
    {
        color_map[object_id] = GenerateComponentColor(object_id);
    }
  }
  auto display_rep = DefaultMarker();
  display_rep.ns = "tagged_object_collision_map_display";
  // Add all the cells of the SDF to the message. 1.9 is to ensure that we
  // get all corners, without also getting extra interior cells
  const float bdy_cell_min_dist = -1.9 * GetResolution();
  for (int64_t x_idx = 0; x_idx < GetNumXCells(); x_idx++)
  {
    for (int64_t y_idx = 0; y_idx < GetNumYCells(); y_idx++)
    {
      for (int64_t z_idx = 0; z_idx < GetNumZCells(); z_idx++)
      {
        const auto& cell = GetImmutable(x_idx, y_idx, z_idx).first;
        // Get the SDF for the current object
        auto sdf_found_itr = per_object_sdfs.find(cell.object_id);
        if (sdf_found_itr != per_object_sdfs.end())
        {
          const auto& object_sdf = sdf_found_itr->second;
          const float dist = object_sdf.GetImmutable(x_idx, y_idx, z_idx).first;
          // Check if we're on the surface of the object
          if (dist < 0.0 && dist > bdy_cell_min_dist)
          {
            const auto object_color = color_map.at(cell.object_id);
            if (object_color.a > 0.0)
            {
              // Convert grid indices into a real-world location
              const auto location =
                  GridIndexToLocationGridFrame(x_idx, y_idx, z_idx);
              geometry_msgs::Point new_point;
              new_point.x = location(0);
              new_point.y = location(1);
              new_point.z = location(2);
              display_rep.points.push_back(new_point);
              display_rep.colors.push_back(object_color);
            }
          }
        }
      }
    }
  }
  return display_rep;
}

visualization_msgs::MarkerArray
TaggedObjectCollisionMapGrid::ExportContourOnlyForDisplayUniqueNs(
    std::map<uint32_t, std_msgs::ColorRGBA> color_map) const
{
  static const std::string ns_base = "tagged_object_collision_map_display_";
  // Make SDFs for the objects that we will be displaying - filters out id 0
  const auto per_object_sdfs = MakeAllObjectSDFs(true, false);
  // Track the position of each object in display_rep.markers
  visualization_msgs::MarkerArray display_rep;
  std::map<uint32_t, uint32_t> objects_to_draw_map;
  for (const auto& kv_pair : per_object_sdfs)
  {
    const auto object_id = kv_pair.first;
    objects_to_draw_map[object_id] = display_rep.markers.size();
    display_rep.markers.push_back(DefaultMarker());
    display_rep.markers.back().ns = ns_base + std::to_string(object_id);
    // Create a color if needed
    const auto color_found_itr = color_map.find(object_id);
    if (color_found_itr == color_map.end())
    {
        color_map[object_id] = GenerateComponentColor(object_id);
    }
  }
  // Add all the cells of the SDF to the message. 1.9 is to ensure that we
  // get all corners, without also getting extra interior cells
  const float bdy_cell_min_dist = -1.9 * GetResolution();
  // Add all the cells of the SDF to the message
  for (int64_t x_idx = 0; x_idx < GetNumXCells(); x_idx++)
  {
    for (int64_t y_idx = 0; y_idx < GetNumYCells(); y_idx++)
    {
      for (int64_t z_idx = 0; z_idx < GetNumZCells(); z_idx++)
      {
        const auto& cell = GetImmutable(x_idx, y_idx, z_idx).first;
        // Get the SDF for the current object
        const auto sdf_found_itr = per_object_sdfs.find(cell.object_id);
        if (sdf_found_itr != per_object_sdfs.end())
        {
          // Extract the distance in the sdf
          const auto& object_sdf = sdf_found_itr->second;
          const float dist = object_sdf.GetImmutable(x_idx, y_idx, z_idx).first;
          // Check if we're on the surface of the object
          if (dist < 0.0 && dist > bdy_cell_min_dist)
          {
            const auto object_color = color_map.at(cell.object_id);
            if (object_color.a > 0.0)
            {
              // Convert grid indices into a real-world location
              const auto location =
                  GridIndexToLocationGridFrame(x_idx, y_idx, z_idx);
              geometry_msgs::Point new_point;
              new_point.x = location(0);
              new_point.y = location(1);
              new_point.z = location(2);
              const auto marker_idx = objects_to_draw_map.at(cell.object_id);
              auto& marker = display_rep.markers.at(marker_idx);
              marker.points.push_back(new_point);
              marker.colors.push_back(object_color);
            }
          }
        }
      }
    }
  }
  // Remove all markers with no points in them
  visualization_msgs::MarkerArray display_rep_pruned;
  for (const auto& marker : display_rep.markers)
  {
      if (!marker.points.empty())
      {
          display_rep_pruned.markers.push_back(marker);
      }
  }
  return display_rep_pruned;
}





visualization_msgs::Marker
TaggedObjectCollisionMapGrid::ExportForDisplayOccupancyOnly(
    const std_msgs::ColorRGBA& collision_color,
    const std_msgs::ColorRGBA& free_color,
    const std_msgs::ColorRGBA& unknown_color) const
{
  visualization_msgs::Marker display_rep = DefaultMarker();
  display_rep.ns = "tagged_object_collision_map_occupancy_display";
  // Add all the cells of the SDF to the message
  for (int64_t x_idx = 0; x_idx < GetNumXCells(); x_idx++)
  {
    for (int64_t y_idx = 0; y_idx < GetNumYCells(); y_idx++)
    {
      for (int64_t z_idx = 0; z_idx < GetNumZCells(); z_idx++)
      {
        const auto& cell = GetImmutable(x_idx, y_idx, z_idx).first;
        std_msgs::ColorRGBA color;
        if (cell.occupancy > 0.5)
        {
          color = collision_color;
        }
        else if (cell.occupancy < 0.5)
        {
          color = free_color;
        }
        else
        {
          color = unknown_color;
        }
        if (color.a > 0.0)
        {
          // Convert grid indices into a real-world location
          const auto location =
              GridIndexToLocationGridFrame(x_idx, y_idx, z_idx);
          geometry_msgs::Point new_point;
          new_point.x = location(0);
          new_point.y = location(1);
          new_point.z = location(2);
          display_rep.points.push_back(new_point);
          display_rep.colors.push_back(color);
        }
      }
    }
  }
  return display_rep;
}

visualization_msgs::Marker
TaggedObjectCollisionMapGrid::ExportConnectedComponentsForDisplay(
    const bool color_unknown_components) const
{
  visualization_msgs::Marker display_rep = DefaultMarker();
  display_rep.ns = "tagged_object_connected_components_display";
  // Add all the cells of the SDF to the message
  for (int64_t x_idx = 0; x_idx < GetNumXCells(); x_idx++)
  {
    for (int64_t y_idx = 0; y_idx < GetNumYCells(); y_idx++)
    {
      for (int64_t z_idx = 0; z_idx < GetNumZCells(); z_idx++)
      {
        // Convert grid indices into a real-world location
        const auto location =
            GridIndexToLocationGridFrame(x_idx, y_idx, z_idx);
        geometry_msgs::Point new_point;
        new_point.x = location(0);
        new_point.y = location(1);
        new_point.z = location(2);
        display_rep.points.push_back(new_point);
        const auto& cell = GetImmutable(x_idx, y_idx, z_idx).first;
        if (cell.occupancy != 0.5)
        {
          const auto color = GenerateComponentColor(cell.component);
          display_rep.colors.push_back(color);
        }
        else
        {
          if (color_unknown_components)
          {
            const auto color = GenerateComponentColor(cell.component);
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

visualization_msgs::Marker
TaggedObjectCollisionMapGrid::ExportConvexSegmentForDisplay(
    const uint32_t object_id,
    const uint32_t convex_segment) const
{
  visualization_msgs::Marker display_rep = DefaultMarker();
  display_rep.ns = "tagged_object_"
                   + std::to_string(object_id)
                   + "_convex_segment_"
                   + std::to_string(convex_segment)
                   + "_display";
  // Add all the cells of the SDF to the message
  for (int64_t x_idx = 0; x_idx < GetNumXCells(); x_idx++)
  {
    for (int64_t y_idx = 0; y_idx < GetNumYCells(); y_idx++)
    {
      for (int64_t z_idx = 0; z_idx < GetNumZCells(); z_idx++)
      {
        const auto& cell = GetImmutable(x_idx, y_idx, z_idx).first;
        if ((cell.object_id == object_id)
            && (cell.convex_segment == convex_segment))
        {
          // Convert grid indices into a real-world location
          const auto location
              = GridIndexToLocationGridFrame(x_idx, y_idx, z_idx);
          geometry_msgs::Point new_point;
          new_point.x = location(0);
          new_point.y = location(1);
          new_point.z = location(2);
          display_rep.points.push_back(new_point);
          // Generate a color
          if (number_of_convex_segments_ < 22)
          {
            const auto color = GenerateComponentColor(convex_segment);
            display_rep.colors.push_back(color);
          }
          else
          {
            const auto color =
                arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>
                  ::InterpolateHotToCold(convex_segment, 1.0,
                                         (double)number_of_convex_segments_);
            display_rep.colors.push_back(color);
          }
        }
      }
    }
  }
  return display_rep;
}

visualization_msgs::Marker
TaggedObjectCollisionMapGrid::ExportSurfaceForDisplay(
    const std::unordered_map<GRID_INDEX, uint8_t>& surface,
    const std_msgs::ColorRGBA& surface_color) const
{
  visualization_msgs::Marker display_rep = DefaultMarker();
  display_rep.ns = "tagged_object_collision_map_surface";
  // Add all the cells of the surface
  std::unordered_map<GRID_INDEX, uint8_t>::const_iterator surface_itr;
  for (surface_itr = surface.begin();
       surface_itr != surface.end();
       ++surface_itr)
  {
    const GRID_INDEX index = surface_itr->first;
    const int8_t validity = surface_itr->second;
    if (validity == 1)
    {
      // Convert grid indices into a real-world location
      const auto location = GridIndexToLocationGridFrame(index);
      geometry_msgs::Point new_point;
      new_point.x = location(0);
      new_point.y = location(1);
      new_point.z = location(2);
      display_rep.points.push_back(new_point);
      display_rep.colors.push_back(surface_color);
    }
  }
  return display_rep;
}
