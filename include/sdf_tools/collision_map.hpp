#include <stdlib.h>
#include <vector>
#include <string>
#include <Eigen/Geometry>
#include <visualization_msgs/MarkerArray.h>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/serialization.hpp>
#include <arc_utilities/voxel_grid.hpp>
#include <sdf_tools/sdf.hpp>
#include <sdf_tools/sdf_generation.hpp>
#include <sdf_tools/CollisionMap.h>

#ifndef COLLISION_MAP_HPP
#define COLLISION_MAP_HPP

#define ENABLE_UNORDERED_MAP_SIZE_HINTS

namespace sdf_tools
{
struct COLLISION_CELL
{
  float occupancy;
  uint32_t component;

  COLLISION_CELL() : occupancy(0.0), component(0) {}

  explicit COLLISION_CELL(const float in_occupancy)
    : occupancy(in_occupancy), component(0) {}

  COLLISION_CELL(const float in_occupancy, const uint32_t in_component)
    : occupancy(in_occupancy), component(in_component) {}
};

class CollisionMapGrid : public VoxelGrid::VoxelGrid<COLLISION_CELL>
{
protected:

  inline static std_msgs::ColorRGBA GenerateComponentColor(
      const uint32_t component, const float alpha=1.0f)
  {
      return arc_helpers::GenerateUniqueColor<std_msgs::ColorRGBA>(
            component, alpha);
  }

  inline bool IsSurfaceIndex(const int64_t x_index,
                             const int64_t y_index,
                             const int64_t z_index) const
  {
    // First, we make sure that indices are within bounds
    // Out of bounds indices are NOT surface cells
    if (IndexInBounds(x_index, y_index, z_index) == false)
    {
      return false;
    }
    // Check all 26 possible neighbors
    const int64_t min_x_check = std::max((int64_t)0, x_index - 1);
    const int64_t max_x_check = std::min(GetNumXCells() - 1, x_index + 1);
    const int64_t min_y_check = std::max((int64_t)0, y_index - 1);
    const int64_t max_y_check = std::min(GetNumYCells() - 1, y_index + 1);
    const int64_t min_z_check = std::max((int64_t)0, z_index - 1);
    const int64_t max_z_check = std::min(GetNumZCells() - 1, z_index + 1);
    const float our_occupancy
      = GetImmutable(x_index, y_index, z_index).first.occupancy;
    for (int64_t x_idx = min_x_check; x_idx <= max_x_check; x_idx++)
    {
      for (int64_t y_idx = min_y_check; y_idx <= max_y_check; y_idx++)
      {
        for (int64_t z_idx = min_z_check; z_idx <= max_z_check; z_idx++)
        {
          // Skip ourselves
          if ((x_idx != x_index) || (y_idx != y_index) || (z_idx != z_index))
          {
            const float other_occupancy
              = GetImmutable(x_idx, y_idx, z_idx).first.occupancy;
            if ((our_occupancy < 0.5) && (other_occupancy >= 0.5))
            {
              return true;
            }
            else if ((our_occupancy > 0.5) && (other_occupancy <= 0.5))
            {
              return true;
            }
            else if ((our_occupancy == 0.5) && (other_occupancy != 0.5))
            {
              return true;
            }
          }
        }
      }
    }
    return false;
  }

  inline bool IsConnectedComponentSurfaceIndex(const int64_t x_index,
                                               const int64_t y_index,
                                               const int64_t z_index) const
  {
    // First, we make sure that indices are within bounds
    // Out of bounds indices are NOT surface cells
    if (x_index < 0 || y_index < 0 || z_index < 0 || x_index >= GetNumXCells()
        || y_index >= GetNumYCells() || z_index >= GetNumZCells())
    {
      return false;
    }
    // Edge indices are automatically surface cells
    if (x_index == 0 || y_index == 0 || z_index == 0
        || x_index == (GetNumXCells() - 1) || y_index == (GetNumYCells() - 1)
        || z_index == (GetNumZCells()))
    {
      return true;
    }
    // If the cell is inside the grid, we check the neighbors
    // Note that we must check all 26 neighbors
    const uint32_t our_component
        = GetImmutable(x_index, y_index, z_index).first.component;
    // Check neighbor 1
    if (our_component
        != GetImmutable(x_index, y_index, z_index - 1).first.component)
    {
      return true;
    }
    // Check neighbor 2
    else if (our_component
             != GetImmutable(x_index, y_index, z_index + 1).first.component)
    {
      return true;
    }
    // Check neighbor 3
    else if (our_component
             != GetImmutable(x_index, y_index - 1, z_index).first.component)
    {
      return true;
    }
    // Check neighbor 4
    else if (our_component
             != GetImmutable(x_index, y_index + 1, z_index).first.component)
    {
      return true;
    }
    // Check neighbor 5
    else if (our_component
             != GetImmutable(x_index - 1, y_index, z_index).first.component)
    {
      return true;
    }
    // Check neighbor 6
    else if (our_component
             != GetImmutable(x_index + 1, y_index, z_index).first.component)
    {
      return true;
    }
    // If none of the faces are exposed, it's not a surface voxel
    return false;
  }

  uint32_t number_of_components_;
  std::string frame_;
  bool components_valid_;

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  inline CollisionMapGrid(const Eigen::Isometry3d& origin_transform,
                          const std::string& frame,
                          const double resolution,
                          const double x_size,
                          const double y_size,
                          const double z_size,
                          const COLLISION_CELL& default_value,
                          const COLLISION_CELL& OOB_value)
    : VoxelGrid::VoxelGrid<COLLISION_CELL>(origin_transform, resolution,
                                           x_size, y_size, z_size,
                                           default_value, OOB_value),
      number_of_components_(0u),
      frame_(frame),
      components_valid_(false) {}

  inline CollisionMapGrid(const std::string& frame,
                          const double resolution,
                          const double x_size,
                          const double y_size,
                          const double z_size,
                          const COLLISION_CELL& default_value,
                          const COLLISION_CELL& OOB_value)
    : VoxelGrid::VoxelGrid<COLLISION_CELL>(resolution, x_size, y_size, z_size,
                                           default_value, OOB_value),
      number_of_components_(0u),
      frame_(frame),
      components_valid_(false) {}

  inline CollisionMapGrid(const Eigen::Isometry3d& origin_transform,
                          const std::string& frame,
                          const double resolution,
                          const double x_size,
                          const double y_size,
                          const double z_size,
                          const COLLISION_CELL& oob_default_value)
    : VoxelGrid::VoxelGrid<COLLISION_CELL>(origin_transform, resolution,
                                           x_size, y_size, z_size,
                                           oob_default_value),
      number_of_components_(0u),
      frame_(frame),
      components_valid_(false) {}

  inline CollisionMapGrid(const std::string& frame,
                          const double resolution,
                          const double x_size,
                          const double y_size,
                          const double z_size,
                          const COLLISION_CELL& oob_default_value)
    : VoxelGrid::VoxelGrid<COLLISION_CELL>(resolution, x_size, y_size, z_size,
                                           oob_default_value),
      number_of_components_(0u),
      frame_(frame),
      components_valid_(false) {}

  inline CollisionMapGrid(const Eigen::Isometry3d& origin_transform,
                          const std::string& frame,
                          const double resolution,
                          const int64_t x_cells,
                          const int64_t y_cells,
                          const int64_t z_cells,
                          const COLLISION_CELL& default_value,
                          const COLLISION_CELL& OOB_value)
    : VoxelGrid::VoxelGrid<COLLISION_CELL>(origin_transform, resolution,
                                           x_cells, y_cells, z_cells,
                                           default_value, OOB_value),
      number_of_components_(0u),
      frame_(frame),
      components_valid_(false) {}

  inline CollisionMapGrid(const std::string& frame,
                          const double resolution,
                          const int64_t x_cells,
                          const int64_t y_cells,
                          const int64_t z_cells,
                          const COLLISION_CELL& default_value,
                          const COLLISION_CELL& OOB_value)
    : VoxelGrid::VoxelGrid<COLLISION_CELL>(resolution, x_cells, y_cells,
                                           z_cells, default_value, OOB_value),
      number_of_components_(0u),
      frame_(frame),
      components_valid_(false) {}

  inline CollisionMapGrid(const Eigen::Isometry3d& origin_transform,
                          const std::string& frame,
                          const double resolution,
                          const int64_t x_cells,
                          const int64_t y_cells,
                          const int64_t z_cells,
                          const COLLISION_CELL& oob_default_value)
    : VoxelGrid::VoxelGrid<COLLISION_CELL>(origin_transform, resolution,
                                           x_cells, y_cells, z_cells,
                                           oob_default_value),
      number_of_components_(0u),
      frame_(frame),
      components_valid_(false) {}

  inline CollisionMapGrid(const std::string& frame,
                          const double resolution,
                          const int64_t x_cells,
                          const int64_t y_cells,
                          const int64_t z_cells,
                          const COLLISION_CELL& oob_default_value)
    : VoxelGrid::VoxelGrid<COLLISION_CELL>(resolution, x_cells, y_cells,
                                           z_cells, oob_default_value),
      number_of_components_(0u),
      frame_(frame),
      components_valid_(false) {}

  inline CollisionMapGrid()
    : VoxelGrid::VoxelGrid<COLLISION_CELL>(),
      number_of_components_(0), frame_(""), components_valid_(false) {}

  virtual VoxelGrid<COLLISION_CELL>* Clone() const
  {
    return new CollisionMapGrid(
          static_cast<const CollisionMapGrid&>(*this));
  }

  inline bool AreComponentsValid() const
  {
    return components_valid_;
  }

  virtual std::pair<COLLISION_CELL&, bool> GetMutable3d(
      const Eigen::Vector3d& location)
  {
    const GRID_INDEX index = LocationToGridIndex3d(location);
    if (IndexInBounds(index))
    {
      return GetMutable(index);
    }
    else
    {
      return std::pair<COLLISION_CELL&, bool>(oob_value_, false);
    }
  }

  virtual std::pair<COLLISION_CELL&, bool> GetMutable4d(
      const Eigen::Vector4d& location)
  {
    const GRID_INDEX index = LocationToGridIndex4d(location);
    if (IndexInBounds(index))
    {
      return GetMutable(index);
    }
    else
    {
      return std::pair<COLLISION_CELL&, bool>(oob_value_, false);
    }
  }

  virtual std::pair<COLLISION_CELL&, bool> GetMutable(const double x,
                                                      const double y,
                                                      const double z)
  {
    const Eigen::Vector4d location(x, y, z, 1.0);
    return GetMutable4d(location);
  }

  virtual std::pair<COLLISION_CELL&, bool> GetMutable(const GRID_INDEX& index)
  {
    if (IndexInBounds(index))
    {
      components_valid_ = false;
      return std::pair<COLLISION_CELL&, bool>(
            AccessIndex(GetDataIndex(index)), true);
    }
    else
    {
      return std::pair<COLLISION_CELL&, bool>(oob_value_, false);
    }
  }

  virtual std::pair<COLLISION_CELL&, bool> GetMutable(const int64_t x_index,
                                                      const int64_t y_index,
                                                      const int64_t z_index)
  {
    if (IndexInBounds(x_index, y_index, z_index))
    {
      components_valid_ = false;
      return std::pair<COLLISION_CELL&, bool>(
            AccessIndex(GetDataIndex(x_index, y_index, z_index)), true);
    }
    else
    {
      return std::pair<COLLISION_CELL&, bool>(oob_value_, false);
    }
  }

  virtual bool SetValue3d(const Eigen::Vector3d& location,
                          const COLLISION_CELL& value)
  {
    const GRID_INDEX index = LocationToGridIndex3d(location);
    if (IndexInBounds(index))
    {
      return SetValue(index, value);
    }
    else
    {
      return false;
    }
  }

  virtual bool SetValue4d(const Eigen::Vector4d& location,
                          const COLLISION_CELL& value)
  {
    const GRID_INDEX index = LocationToGridIndex4d(location);
    if (IndexInBounds(index))
    {
      return SetValue(index, value);
    }
    else
    {
      return false;
    }
  }

  virtual bool SetValue(const double x,
                        const double y,
                        const double z,
                        const COLLISION_CELL& value)
  {
    const Eigen::Vector4d location(x, y, z, 1.0);
    return SetValue4d(location, value);
  }

  virtual bool SetValue(const GRID_INDEX& index,
                        const COLLISION_CELL& value)
  {
    if (IndexInBounds(index))
    {
      components_valid_ = false;
      AccessIndex(GetDataIndex(index)) = value;
      return true;
    }
    else
    {
      return false;
    }
  }

  virtual bool SetValue(const int64_t x_index,
                        const int64_t y_index,
                        const int64_t z_index,
                        const COLLISION_CELL& value)
  {
    if (IndexInBounds(x_index, y_index, z_index))
    {
      components_valid_ = false;
      AccessIndex(GetDataIndex(x_index, y_index, z_index)) = value;
      return true;
    }
    else
    {
      return false;
    }
  }

  virtual bool SetValue3d(const Eigen::Vector3d& location,
                          COLLISION_CELL&& value)
  {
    const GRID_INDEX index = LocationToGridIndex3d(location);
    if (IndexInBounds(index))
    {
      return SetValue(index, value);
    }
    else
    {
      return false;
    }
  }

  virtual bool SetValue4d(const Eigen::Vector4d& location,
                          COLLISION_CELL&& value)
  {
    const GRID_INDEX index = LocationToGridIndex4d(location);
    if (IndexInBounds(index))
    {
      return SetValue(index, value);
    }
    else
    {
      return false;
    }
  }

  virtual bool SetValue(const double x,
                        const double y,
                        const double z,
                        COLLISION_CELL&& value)
  {
    const Eigen::Vector4d location(x, y, z, 1.0);
    return SetValue4d(location, value);
  }

  virtual bool SetValue(const GRID_INDEX& index,
                        COLLISION_CELL&& value)
  {
    if (IndexInBounds(index))
    {
      components_valid_ = false;
      AccessIndex(GetDataIndex(index)) = value;
      return true;
    }
    else
    {
      return false;
    }
  }

  virtual bool SetValue(const int64_t x_index,
                        const int64_t y_index,
                        const int64_t z_index,
                        COLLISION_CELL&& value)
  {
    if (IndexInBounds(x_index, y_index, z_index))
    {
      components_valid_ = false;
      AccessIndex(GetDataIndex(x_index, y_index, z_index)) = value;
      return true;
    }
    else
    {
      return false;
    }
  }

  inline double GetResolution() const { return GetCellSizes().x(); }

  inline std::string GetFrame() const
  {
    return frame_;
  }

  inline void SetFrame(const std::string& new_frame)
  {
    frame_ = new_frame;
  }

  inline std::pair<uint32_t, bool> GetNumConnectedComponents() const
  {
    return std::pair<uint32_t, bool>(number_of_components_, components_valid_);
  }

  inline std::pair<bool, bool> CheckIfCandidateCorner3d(
      const Eigen::Vector3d& location) const
  {
    const GRID_INDEX index = LocationToGridIndex3d(location);
    if (IndexInBounds(index))
    {
      return CheckIfCandidateCorner(index);
    }
    else
    {
      return std::pair<bool, bool>(false, false);
    }
  }

  inline std::pair<bool, bool> CheckIfCandidateCorner4d(
      const Eigen::Vector4d& location) const
  {
    const GRID_INDEX index = LocationToGridIndex4d(location);
    if (IndexInBounds(index))
    {
      return CheckIfCandidateCorner(index);
    }
    else
    {
      return std::pair<bool, bool>(false, false);
    }
  }

  inline std::pair<bool, bool> CheckIfCandidateCorner(
      const double x, const double y, const double z) const
  {
    const Eigen::Vector4d location(x, y, z, 1.0);
    return CheckIfCandidateCorner4d(location);
  }

  inline std::pair<bool, bool> CheckIfCandidateCorner(
      const GRID_INDEX& index) const
  {
    return CheckIfCandidateCorner(index.x, index.y, index.z);
  }

  inline std::pair<bool, bool> CheckIfCandidateCorner(
      const int64_t x_index, const int64_t y_index, const int64_t z_index) const
  {
    const std::pair<const COLLISION_CELL&, bool> current_cell
        = GetImmutable(x_index, y_index, z_index);
    if (current_cell.second)
    {
      // Grab the six neighbors & check if they belong to a different component
      uint32_t different_neighbors = 0u;
      const std::pair<const COLLISION_CELL&, bool> xm1yz_cell
          = GetImmutable(x_index - 1, y_index, z_index);
      if (xm1yz_cell.second
          && (xm1yz_cell.first.component != current_cell.first.component))
      {
        different_neighbors++;
      }
      const std::pair<const COLLISION_CELL&, bool> xp1yz_cell
          = GetImmutable(x_index + 1, y_index, z_index);
      if (xp1yz_cell.second
          && (xp1yz_cell.first.component != current_cell.first.component))
      {
        different_neighbors++;
      }
      const std::pair<const COLLISION_CELL&, bool> xym1z_cell
          = GetImmutable(x_index, y_index - 1, z_index);
      if (xym1z_cell.second
          && (xym1z_cell.first.component != current_cell.first.component))
      {
        different_neighbors++;
      }
      const std::pair<const COLLISION_CELL&, bool> xyp1z_cell
          = GetImmutable(x_index, y_index + 1, z_index);
      if (xyp1z_cell.second
          && (xyp1z_cell.first.component != current_cell.first.component))
      {
        different_neighbors++;
      }
      const std::pair<const COLLISION_CELL&, bool> xyzm1_cell
          = GetImmutable(x_index, y_index, z_index - 1);
      if (xyzm1_cell.second
          && (xyzm1_cell.first.component != current_cell.first.component))
      {
        different_neighbors++;
      }
      const std::pair<const COLLISION_CELL&, bool> xyzp1_cell
          = GetImmutable(x_index, y_index, z_index + 1);
      if (xyzp1_cell.second
          && (xyzp1_cell.first.component != current_cell.first.component))
      {
        different_neighbors++;
      }
      // We now have between zero and six neighbors to work with
      if (different_neighbors <= 1u)
      {
        // If there is one or fewer neighbors to work with,
        // we are clearly not a corner
        return std::pair<bool, bool>(false, true);
      }
      else
      {
        // If there are 2 or more neighbors to work with,
        // we are a candidate corner
        return std::pair<bool, bool>(true, true);
      }
    }
    else
    {
      // Not in the grid
      return std::pair<bool, bool>(false, false);
    }
  }

  virtual uint64_t SerializeSelf(
      std::vector<uint8_t>& buffer,
      const std::function<uint64_t(
        const COLLISION_CELL&, std::vector<uint8_t>&)>& value_serializer
      = arc_utilities::SerializeFixedSizePOD<COLLISION_CELL>) const;

  virtual uint64_t DeserializeSelf(
      const std::vector<uint8_t>& buffer, const uint64_t current,
      const std::function<std::pair<COLLISION_CELL, uint64_t>(
        const std::vector<uint8_t>&, const uint64_t)>& value_deserializer
      = arc_utilities::DeserializeFixedSizePOD<COLLISION_CELL>);

  static void SaveToFile(const CollisionMapGrid& map,
                         const std::string& filepath,
                         const bool compress);

  static CollisionMapGrid LoadFromFile(const std::string& filepath);

  static sdf_tools::CollisionMap GetMessageRepresentation(
      const CollisionMapGrid& map);

  static CollisionMapGrid LoadFromMessageRepresentation(
      const sdf_tools::CollisionMap& message);

  uint32_t UpdateConnectedComponents();

  enum COMPONENT_TYPES : uint8_t { FILLED_COMPONENTS=0x01,
                                   EMPTY_COMPONENTS=0x02,
                                   UNKNOWN_COMPONENTS=0x04 };

  std::map<uint32_t, std::unordered_map<GRID_INDEX, uint8_t>>
  ExtractComponentSurfaces(
      const COMPONENT_TYPES component_types_to_extract) const;

  std::map<uint32_t, std::unordered_map<GRID_INDEX, uint8_t>>
  ExtractFilledComponentSurfaces() const
  {
    return ExtractComponentSurfaces(FILLED_COMPONENTS);
  }

  std::map<uint32_t, std::unordered_map<GRID_INDEX, uint8_t>>
  ExtractUnknownComponentSurfaces() const
  {
    return ExtractComponentSurfaces(UNKNOWN_COMPONENTS);
  }

  std::map<uint32_t, std::unordered_map<GRID_INDEX, uint8_t>>
  ExtractEmptyComponentSurfaces() const
  {
    return ExtractComponentSurfaces(EMPTY_COMPONENTS);
  }

  CollisionMapGrid Resample(const double new_resolution) const;

  std::map<uint32_t, std::pair<int32_t, int32_t>>
  ComputeComponentTopology(const bool ignore_empty_components,
                           const bool recompute_connected_components,
                           const bool verbose);

  std::pair<sdf_tools::SignedDistanceField, std::pair<double, double>>
  ExtractSignedDistanceField(const float oob_value,
                             const bool unknown_is_filled,
                             const bool add_virtual_border) const
  {
    // Make the helper function
    const std::function<bool(const GRID_INDEX&)>
        is_filled_fn = [&] (const GRID_INDEX& index)
    {
      const auto query = GetImmutable(index);
      if (query.second)
      {
        if (query.first.occupancy > 0.5)
        {
          // Mark as filled
          return true;
        }
        else if (unknown_is_filled && (query.first.occupancy == 0.5))
        {
          // Mark as filled
          return true;
        }
        // Mark as free
        return false;
      }
      else
      {
        throw std::runtime_error("index out of grid bounds");
      }
    };
    return sdf_generation::ExtractSignedDistanceField(
          *this, is_filled_fn, oob_value, GetFrame(), add_virtual_border);
  }

  visualization_msgs::Marker ExportForDisplay(
      const std_msgs::ColorRGBA& collision_color,
      const std_msgs::ColorRGBA& free_color,
      const std_msgs::ColorRGBA& unknown_color) const;

  visualization_msgs::MarkerArray ExportForSeparateDisplay(
      const std_msgs::ColorRGBA& collision_color,
      const std_msgs::ColorRGBA& free_color,
      const std_msgs::ColorRGBA& unknown_color) const;

  visualization_msgs::Marker ExportSurfacesForDisplay(
      const std_msgs::ColorRGBA& collision_color,
      const std_msgs::ColorRGBA& free_color,
      const std_msgs::ColorRGBA& unknown_color) const;

  visualization_msgs::MarkerArray ExportSurfacesForSeparateDisplay(
      const std_msgs::ColorRGBA& collision_color,
      const std_msgs::ColorRGBA& free_color,
      const std_msgs::ColorRGBA& unknown_color) const;

  visualization_msgs::Marker ExportConnectedComponentsForDisplay(
      const bool color_unknown_components) const;

  std::vector<std::vector<GRID_INDEX>> ExtractConnectedComponents();

};
}

#endif // COLLISION_MAP_HPP
