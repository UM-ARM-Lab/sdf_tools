#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <unordered_map>
#include <ros/ros.h>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/serialization_eigen.hpp>
#include <arc_utilities/zlib_helpers.hpp>
#include <sdf_tools/sdf.hpp>
#include <sdf_tools/SDF.h>

using namespace sdf_tools;

/////////////////////////////////////////////////////////////////////
// Local extrema map computation
/////////////////////////////////////////////////////////////////////

void SignedDistanceField::FollowGradientsToLocalExtremaUnsafe(
    VoxelGrid<Eigen::Vector3d>& watershed_map,
    const int64_t x_index, const int64_t y_index, const int64_t z_index) const
{
  // First, check if we've already found the local extrema for the current cell
  const Eigen::Vector3d& stored
      = watershed_map.GetImmutable(x_index, y_index, z_index).first;
  if (stored.x() != -std::numeric_limits<double>::infinity()
      && stored.y() != -std::numeric_limits<double>::infinity()
      && stored.z() != -std::numeric_limits<double>::infinity())
  {
    // We've already found it for this cell, so we can skip it
    return;
  }
  // Find the local extrema
  std::vector<double> raw_gradient
      = GetGradient(x_index, y_index, z_index, true);
  Eigen::Vector3d gradient_vector(raw_gradient[0],
                                  raw_gradient[1],
                                  raw_gradient[2]);
  if (GradientIsEffectiveFlat(gradient_vector))
  {
    const Eigen::Vector4d location
        = GridIndexToLocationGridFrame(x_index, y_index, z_index);
    const Eigen::Vector3d local_extrema(location(0), location(1), location(2));
    watershed_map.SetValue(x_index, y_index, z_index, local_extrema);
  }
  else
  {
    // Follow the gradient, one cell at a time, until we reach a local maxima
    std::unordered_map<GRID_INDEX, int8_t> path;
    GRID_INDEX current_index(x_index, y_index, z_index);
    path[current_index] = 1;
    Eigen::Vector3d local_extrema(-std::numeric_limits<double>::infinity(),
                                  -std::numeric_limits<double>::infinity(),
                                  -std::numeric_limits<double>::infinity());
    while (true)
    {
      if (path.size() == 10000)
      {
        std::cerr << "Warning, gradient path is long (i.e >= 10000 steps)"
                  << std::endl;
      }
      current_index = GetNextFromGradient(current_index, gradient_vector);
      if (path[current_index] != 0)
      {
        // If we've already been here, then we are done
        const Eigen::Vector4d location
            = GridIndexToLocationGridFrame(current_index);
        local_extrema = Eigen::Vector3d(location(0), location(1), location(2));
        break;
      }
      // Check if we've been pushed past the edge
      if (current_index.x < 0 || current_index.y < 0 || current_index.z < 0
          || current_index.x >= watershed_map.GetNumXCells()
          || current_index.y >= watershed_map.GetNumYCells()
          || current_index.z >= watershed_map.GetNumZCells())
      {
        // We have the "off the grid" local maxima
        local_extrema
            = Eigen::Vector3d(std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::infinity());
        break;
      }
      path[current_index] = 1;
      // Check if the new index has already been checked
      const Eigen::Vector3d& new_stored
          = watershed_map.GetImmutable(current_index).first;
      if (new_stored.x() != -std::numeric_limits<double>::infinity()
          && new_stored.y() != -std::numeric_limits<double>::infinity()
          && new_stored.z() != -std::numeric_limits<double>::infinity())
      {
        // We have the local maxima
        local_extrema = new_stored;
        break;
      }
      else
      {
        raw_gradient = GetGradient(current_index, true);
        gradient_vector = Eigen::Vector3d(raw_gradient[0],
                                          raw_gradient[1],
                                          raw_gradient[2]);
        if (GradientIsEffectiveFlat(gradient_vector))
        {
          // We have the local maxima
          const Eigen::Vector4d location
              = GridIndexToLocationGridFrame(current_index);
          local_extrema
              = Eigen::Vector3d(location(0), location(1), location(2));
          break;
        }
      }
    }
    // Now, go back and mark the entire explored path with the local maxima
    for (auto path_itr = path.begin(); path_itr != path.end(); ++path_itr)
    {
      const GRID_INDEX& index = path_itr->first;
      watershed_map.SetValue(index, local_extrema);
    }
  }
}

bool SignedDistanceField::GradientIsEffectiveFlat(
    const Eigen::Vector3d& gradient) const
{
  // A gradient is at a local maxima if the absolute value of all components
  // (x,y,z) are less than 1/2 SDF resolution
  const double step_resolution = GetResolution() * 0.06125;
  if (std::abs(gradient.x()) <= step_resolution
      && std::abs(gradient.y()) <= step_resolution
      && std::abs(gradient.z()) <= step_resolution)
  {
      return true;
  }
  else
  {
      return false;
  }
}

GRID_INDEX SignedDistanceField::GetNextFromGradient(
    const GRID_INDEX& index,
    const Eigen::Vector3d& gradient) const
{
  // Check if it's inside an obstacle
  const float stored_distance = GetImmutable(index).first;
  Eigen::Vector3d working_gradient = gradient;
  if (stored_distance < 0.0)
  {
    working_gradient = gradient * -1.0;
  }
  // Given the gradient, pick the "best fit" of the 26 neighboring points
  GRID_INDEX next_index = index;
  const double step_resolution = GetResolution() * 0.06125;
  if (working_gradient.x() > step_resolution)
  {
      next_index.x++;
  }
  else if (working_gradient.x() < -step_resolution)
  {
      next_index.x--;
  }
  if (working_gradient.y() > step_resolution)
  {
      next_index.y++;
  }
  else if (working_gradient.y() < -step_resolution)
  {
      next_index.y--;
  }
  if (working_gradient.z() > step_resolution)
  {
      next_index.z++;
  }
  else if (working_gradient.z() < -step_resolution)
  {
      next_index.z--;
  }
  return next_index;
}

VoxelGrid::VoxelGrid<Eigen::Vector3d>
SignedDistanceField::ComputeLocalExtremaMap() const
{
  VoxelGrid<Eigen::Vector3d> watershed_map(
        GetOriginTransform(), GetResolution(),
        GetXSize(), GetYSize(), GetZSize(),
        Eigen::Vector3d(-std::numeric_limits<double>::infinity(),
                        -std::numeric_limits<double>::infinity(),
                        -std::numeric_limits<double>::infinity()));
  for (int64_t x_idx = 0; x_idx < watershed_map.GetNumXCells(); x_idx++)
  {
    for (int64_t y_idx = 0; y_idx < watershed_map.GetNumYCells(); y_idx++)
    {
      for (int64_t z_idx = 0; z_idx < watershed_map.GetNumZCells(); z_idx++)
      {
        // We use an "unsafe" function here because we know all the indices
        // we provide it will be safe
        FollowGradientsToLocalExtremaUnsafe(watershed_map, x_idx, y_idx, z_idx);
      }
    }
  }
  return watershed_map;
}

///////////////////////////////////////////////////////////////////////
// Serialization, saving, loading, etc.
///////////////////////////////////////////////////////////////////////

uint64_t SignedDistanceField::SerializeSelf(
    std::vector<uint8_t>& buffer,
    const std::function<uint64_t(
      const float&, std::vector<uint8_t>&)>& value_serializer) const
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
  arc_utilities::SerializeVector<float>(
        data_, buffer, arc_utilities::SerializeFixedSizePOD<float>);
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
  arc_utilities::SerializeFixedSizePOD<float>(default_value_, buffer);
  // Serialize the OOB value
  arc_utilities::SerializeFixedSizePOD<float>(oob_value_, buffer);
  // Serialize SDF stuff
  arc_utilities::SerializeString(frame_, buffer);
  arc_utilities::SerializeFixedSizePOD<uint8_t>((uint8_t)locked_,
                                              buffer);
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

uint64_t SignedDistanceField::DeserializeSelf(
    const std::vector<uint8_t>& buffer, const uint64_t current,
    const std::function<std::pair<float, uint64_t>(
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
  const std::pair<std::vector<float>, uint64_t> data_deserialized
      = arc_utilities::DeserializeVector<float>(
        buffer, current_position,
        arc_utilities::DeserializeFixedSizePOD<float>);
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
  const std::pair<float, uint64_t> default_value_deserialized
      = arc_utilities::DeserializeFixedSizePOD<float>(buffer,
                                                      current_position);
  default_value_ = default_value_deserialized.first;
  current_position += default_value_deserialized.second;
  // Deserialize the OOB value
  const std::pair<float, uint64_t> oob_value_deserialized
      = arc_utilities::DeserializeFixedSizePOD<float>(buffer,
                                                      current_position);
  oob_value_ = oob_value_deserialized.first;
  current_position += oob_value_deserialized.second;
  // Deserialize SDF stuff
  const std::pair<std::string, uint64_t> frame_deserialized
      = arc_utilities::DeserializeString<char>(buffer, current_position);
  frame_ = frame_deserialized.first;
  current_position += frame_deserialized.second;
  const std::pair<uint8_t, uint64_t> locked_deserialized
      = arc_utilities::DeserializeFixedSizePOD<uint8_t>(buffer,
                                                        current_position);
  locked_ = (bool)locked_deserialized.first;
  current_position += locked_deserialized.second;
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - current;
  return bytes_read;
}

void SignedDistanceField::SaveToFile(
    const SignedDistanceField& sdf,
    const std::string& filepath,
    const bool compress)
{
  std::vector<uint8_t> buffer;
  sdf.SerializeSelf(buffer);
  std::ofstream output_file(filepath, std::ios::out|std::ios::binary);
  if (compress)
  {
    output_file.write("SDFZ", 4);
    const std::vector<uint8_t> compressed = ZlibHelpers::CompressBytes(buffer);
    const size_t serialized_size = compressed.size();
    output_file.write(reinterpret_cast<const char*>(
                        compressed.data()), (std::streamsize)serialized_size);
  }
  else
  {
    output_file.write("SDFR", 4);
    const size_t serialized_size = buffer.size();
    output_file.write(reinterpret_cast<const char*>(
                        buffer.data()), (std::streamsize)serialized_size);
  }
  output_file.close();
}

SignedDistanceField SignedDistanceField::LoadFromFile(
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
    if (header_string == "SDFZ")
    {
      const std::vector<uint8_t> decompressed
          = ZlibHelpers::DecompressBytes(file_buffer);
      SignedDistanceField sdf;
      sdf.DeserializeSelf(decompressed, 0);
      return sdf;
    }
    else if (header_string == "SDFR")
    {
      SignedDistanceField sdf;
      sdf.DeserializeSelf(file_buffer, 0);
      return sdf;
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

sdf_tools::SDF SignedDistanceField::GetMessageRepresentation(
    const SignedDistanceField& sdf)
{
  sdf_tools::SDF sdf_message;
  sdf_message.header.stamp = ros::Time::now();
  sdf_message.header.frame_id = sdf.GetFrame();
  std::vector<uint8_t> buffer;
  sdf.SerializeSelf(buffer);
  sdf_message.serialized_sdf = ZlibHelpers::CompressBytes(buffer);
  sdf_message.is_compressed = true;
  return sdf_message;
}

SignedDistanceField SignedDistanceField::LoadFromMessageRepresentation(
    const sdf_tools::SDF& message)
{
  if (message.is_compressed)
  {
    const std::vector<uint8_t> uncompressed_sdf
        = ZlibHelpers::DecompressBytes(message.serialized_sdf);
    SignedDistanceField sdf;
    sdf.DeserializeSelf(uncompressed_sdf, 0);
    return sdf;
  }
  else
  {
    SignedDistanceField sdf;
    sdf.DeserializeSelf(message.serialized_sdf, 0);
    return sdf;
  }
}

visualization_msgs::Marker SignedDistanceField::ExportForDisplay(
    const float alpha) const
{
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
  display_rep.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(
                       GetOriginTransform());
  display_rep.scale.x = GetResolution();
  display_rep.scale.y = GetResolution();
  display_rep.scale.z = GetResolution();
  // Add all the cells of the SDF to the message
  double min_distance = 0.0;
  double max_distance = 0.0;
  for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < GetNumZCells(); z_index++)
      {
        // Update minimum/maximum distance variables
        const float distance
            = GetImmutable(x_index, y_index, z_index).first;
        if (distance < min_distance)
        {
          min_distance = distance;
        }
        if (distance > max_distance)
        {
          max_distance = distance;
        }
        // Convert SDF indices into a real-world location
        const Eigen::Vector4d location
            = GridIndexToLocationGridFrame(x_index, y_index, z_index);
        geometry_msgs::Point new_point;
        new_point.x = location(0);
        new_point.y = location(1);
        new_point.z = location(2);
        display_rep.points.push_back(new_point);
      }
    }
  }
  // Add colors for all the cells of the SDF to the message
  for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < GetNumZCells(); z_index++)
      {
        // Update minimum/maximum distance variables
        const float distance = GetImmutable(x_index, y_index, z_index).first;
        std_msgs::ColorRGBA new_color;
        new_color.a = arc_helpers::ClampValue(alpha, 0.0f, 1.0f);
        if (distance > 0.0)
        {
          new_color.b = 0.0;
          new_color.g = (std::abs(distance / max_distance) * 0.8) + 0.2;
          new_color.r = 0.0;
        }
        else if (distance < 0.0)
        {
          new_color.b = 0.0;
          new_color.g = 0.0;
          new_color.r = (std::abs(distance / min_distance) * 0.8) + 0.2;
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

visualization_msgs::Marker SignedDistanceField::ExportForDisplayCollisionOnly(
    const float alpha) const
{
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
  display_rep.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(
                       GetOriginTransform());
  display_rep.scale.x = GetResolution();
  display_rep.scale.y = GetResolution();
  display_rep.scale.z = GetResolution();
  // Color it
  std_msgs::ColorRGBA collision_color;
  collision_color.a = alpha;
  collision_color.b = 0.0;
  collision_color.g = 0.0;
  collision_color.r = 1.0;
  display_rep.color = collision_color;
  // Add all the cells of the SDF to the message
  for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
  {
      for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
      {
          for (int64_t z_index = 0; z_index < GetNumZCells(); z_index++)
          {
              // Update minimum/maximum distance variables
              const float distance
                  = GetImmutable(x_index, y_index, z_index).first;
              if (distance <= 0.0)
              {
                  // Convert SDF indices into a real-world location
                  const Eigen::Vector4d location
                      = GridIndexToLocationGridFrame(x_index, y_index, z_index);
                  geometry_msgs::Point new_point;
                  new_point.x = location(0);
                  new_point.y = location(1);
                  new_point.z = location(2);
                  display_rep.points.push_back(new_point);
              }
          }
      }
  }
  return display_rep;
}
