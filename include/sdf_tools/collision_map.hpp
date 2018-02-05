#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <functional>
#include <unordered_map>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <Eigen/Geometry>
#include <visualization_msgs/MarkerArray.h>
#include <arc_utilities/arc_helpers.hpp>
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

        COLLISION_CELL(const float in_occupancy) : occupancy(in_occupancy), component(0) {}

        COLLISION_CELL(const float in_occupancy, const uint32_t in_component) : occupancy(in_occupancy), component(in_component) {}
    };

    inline std::vector<uint8_t> CollisionCellToBinary(const COLLISION_CELL& value)
    {
        std::vector<uint8_t> binary(sizeof(COLLISION_CELL));
        memcpy(&binary.front(), &value, sizeof(COLLISION_CELL));
        return binary;
    }

    inline COLLISION_CELL CollisionCellFromBinary(const std::vector<uint8_t>& binary)
    {
        if (binary.size() != sizeof(COLLISION_CELL))
        {
            std::cerr << "Binary value is not " << sizeof(COLLISION_CELL) << " bytes" << std::endl;
            return COLLISION_CELL(NAN, 0u);
        }
        else
        {
            COLLISION_CELL loaded;
            memcpy(&loaded, &binary.front(), sizeof(COLLISION_CELL));
            return loaded;
        }
    }

    class CollisionMapGrid
    {
    protected:

        inline static std_msgs::ColorRGBA GenerateComponentColor(const uint32_t component, const float alpha=1.0f)
        {
            return arc_helpers::GenerateUniqueColor<std_msgs::ColorRGBA>(component, alpha);
        }

        inline bool IsSurfaceIndex(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            // First, we make sure that indices are within bounds
            // Out of bounds indices are NOT surface cells
            if (collision_field_.IndexInBounds(x_index, y_index, z_index) == false)
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
            const float our_occupancy = collision_field_.GetImmutable(x_index, y_index, z_index).first.occupancy;
            for (int64_t x_idx = min_x_check; x_idx <= max_x_check; x_idx++)
            {
                for (int64_t y_idx = min_y_check; y_idx <= max_y_check; y_idx++)
                {
                    for (int64_t z_idx = min_z_check; z_idx <= max_z_check; z_idx++)
                    {
                        // Skip ourselves
                        if ((x_idx != x_index) || (y_idx != y_index) || (z_idx != z_index))
                        {
                            const float other_occupancy = collision_field_.GetImmutable(x_idx, y_idx, z_idx).first.occupancy;
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

        inline bool IsConnectedComponentSurfaceIndex(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            // First, we make sure that indices are within bounds
            // Out of bounds indices are NOT surface cells
            if (x_index < 0 || y_index < 0 || z_index < 0 || x_index >= GetNumXCells() || y_index >= GetNumYCells() || z_index >= GetNumZCells())
            {
                return false;
            }
            // Edge indices are automatically surface cells
            if (x_index == 0 || y_index == 0 || z_index == 0 || x_index == (GetNumXCells() - 1) || y_index == (GetNumYCells() - 1) || z_index == (GetNumZCells()))
            {
                return true;
            }
            // If the cell is inside the grid, we check the neighbors
            // Note that we must check all 26 neighbors
            uint32_t our_component = collision_field_.GetImmutable(x_index, y_index, z_index).first.component;
            // Check neighbor 1
            if (our_component != collision_field_.GetImmutable(x_index, y_index, z_index - 1).first.component)
            {
                return true;
            }
            // Check neighbor 2
            else if (our_component != collision_field_.GetImmutable(x_index, y_index, z_index + 1).first.component)
            {
                return true;
            }
            // Check neighbor 3
            else if (our_component != collision_field_.GetImmutable(x_index, y_index - 1, z_index).first.component)
            {
                return true;
            }
            // Check neighbor 4
            else if (our_component != collision_field_.GetImmutable(x_index, y_index + 1, z_index).first.component)
            {
                return true;
            }
            // Check neighbor 5
            else if (our_component != collision_field_.GetImmutable(x_index - 1, y_index, z_index).first.component)
            {
                return true;
            }
            // Check neighbor 6
            else if (our_component != collision_field_.GetImmutable(x_index + 1, y_index, z_index).first.component)
            {
                return true;
            }
            // If none of the faces are exposed, it's not a surface voxel
            return false;
        }

        VoxelGrid::VoxelGrid<COLLISION_CELL> collision_field_;
        uint32_t number_of_components_;
        std::string frame_;
        bool initialized_;
        bool components_valid_;

        std::vector<uint8_t> PackBinaryRepresentation(const std::vector<COLLISION_CELL>& raw) const;

        std::vector<COLLISION_CELL> UnpackBinaryRepresentation(const std::vector<uint8_t>& packed) const;

    public:

        inline CollisionMapGrid(const std::string& frame, const double resolution, const double x_size, const double y_size, const double z_size, const COLLISION_CELL& default_value, const COLLISION_CELL& OOB_value) : initialized_(true)
        {
            frame_ = frame;
            VoxelGrid::VoxelGrid<COLLISION_CELL> new_field(resolution, x_size, y_size, z_size, default_value, OOB_value);
            collision_field_ = new_field;
            number_of_components_ = 0;
            components_valid_ = false;
        }

        inline CollisionMapGrid(const Eigen::Isometry3d& origin_transform, const std::string& frame, const double resolution, const double x_size, double y_size, const double z_size, const COLLISION_CELL& default_value, const COLLISION_CELL& OOB_value) : initialized_(true)
        {
            frame_ = frame;
            VoxelGrid::VoxelGrid<COLLISION_CELL> new_field(origin_transform, resolution, x_size, y_size, z_size, default_value, OOB_value);
            collision_field_ = new_field;
            number_of_components_ = 0;
            components_valid_ = false;
        }

        inline CollisionMapGrid(const std::string& frame, const double resolution, const double x_size, const double y_size, const double z_size, const COLLISION_CELL& OOB_default_value) : initialized_(true)
        {
            frame_ = frame;
            VoxelGrid::VoxelGrid<COLLISION_CELL> new_field(resolution, x_size, y_size, z_size, OOB_default_value);
            collision_field_ = new_field;
            number_of_components_ = 0;
            components_valid_ = false;
        }

        inline CollisionMapGrid(const Eigen::Isometry3d& origin_transform, const std::string& frame, const double resolution, const double x_size, double y_size, const double z_size, const COLLISION_CELL& OOB_default_value) : initialized_(true)
        {
            frame_ = frame;
            VoxelGrid::VoxelGrid<COLLISION_CELL> new_field(origin_transform, resolution, x_size, y_size, z_size, OOB_default_value);
            collision_field_ = new_field;
            number_of_components_ = 0;
            components_valid_ = false;
        }

        inline CollisionMapGrid() : number_of_components_(0), initialized_(false), components_valid_(false) {}

        inline bool IsInitialized() const
        {
            return initialized_;
        }

        inline bool AreComponentsValid() const
        {
            return components_valid_;
        }

        inline std::pair<COLLISION_CELL, bool> Get3d(const Eigen::Vector3d& location) const
        {
            return collision_field_.GetImmutable3d(location);
        }

        inline std::pair<COLLISION_CELL, bool> Get4d(const Eigen::Vector4d& location) const
        {
            return collision_field_.GetImmutable4d(location);
        }

        inline std::pair<COLLISION_CELL, bool> Get(const double x, const double y, const double z) const
        {
            return collision_field_.GetImmutable(x, y, z);
        }

        inline std::pair<COLLISION_CELL, bool> Get(const VoxelGrid::GRID_INDEX& index) const
        {
            return collision_field_.GetImmutable(index);
        }

        inline std::pair<COLLISION_CELL, bool> Get(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            return collision_field_.GetImmutable(x_index, y_index, z_index);
        }

        inline bool Set(const double x, const double y, const double z, COLLISION_CELL value)
        {
            components_valid_ = false;
            return collision_field_.SetValue(x, y, z, value);
        }

        inline bool Set3d(const Eigen::Vector3d& location, COLLISION_CELL value)
        {
            components_valid_ = false;
            return collision_field_.SetValue3d(location, value);
        }

        inline bool Set4d(const Eigen::Vector4d& location, COLLISION_CELL value)
        {
            components_valid_ = false;
            return collision_field_.SetValue4d(location, value);
        }

        inline bool Set(const int64_t x_index, const int64_t y_index, const int64_t z_index, COLLISION_CELL value)
        {
            components_valid_ = false;
            return collision_field_.SetValue(x_index, y_index, z_index, value);
        }

        inline bool Set(const VoxelGrid::GRID_INDEX& index, COLLISION_CELL value)
        {
            components_valid_ = false;
            return collision_field_.SetValue(index, value);
        }

        inline double GetXSize() const
        {
            return collision_field_.GetXSize();
        }

        inline double GetYSize() const
        {
            return collision_field_.GetYSize();
        }

        inline double GetZSize() const
        {
            return collision_field_.GetZSize();
        }

        inline double GetResolution() const
        {
            return collision_field_.GetCellSizes().x();
        }

        inline COLLISION_CELL GetDefaultValue() const
        {
            return collision_field_.GetDefaultValue();
        }

        inline COLLISION_CELL GetOOBValue() const
        {
            return collision_field_.GetOOBValue();
        }

        inline int64_t GetNumXCells() const
        {
            return collision_field_.GetNumXCells();
        }

        inline int64_t GetNumYCells() const
        {
            return collision_field_.GetNumYCells();
        }

        inline int64_t GetNumZCells() const
        {
            return collision_field_.GetNumZCells();
        }

        inline const Eigen::Isometry3d& GetOriginTransform() const
        {
            return collision_field_.GetOriginTransform();
        }

        inline const Eigen::Isometry3d& GetInverseOriginTransform() const
        {
            return collision_field_.GetInverseOriginTransform();
        }

        inline std::string GetFrame() const
        {
            return frame_;
        }

        inline std::pair<uint32_t, bool> GetNumConnectedComponents() const
        {
            return std::pair<uint32_t, bool>(number_of_components_, components_valid_);
        }

        inline VoxelGrid::GRID_INDEX LocationToGridIndex3d(const Eigen::Vector3d& location) const
        {
            return collision_field_.LocationToGridIndex3d(location);
        }

        inline VoxelGrid::GRID_INDEX LocationToGridIndex4d(const Eigen::Vector4d& location) const
        {
            return collision_field_.LocationToGridIndex4d(location);
        }

        inline VoxelGrid::GRID_INDEX LocationToGridIndex(double x, double y, double z) const
        {
            return collision_field_.LocationToGridIndex(x, y, z);
        }

        inline Eigen::Vector4d GridIndexToLocation(const VoxelGrid::GRID_INDEX& index) const
        {
            return collision_field_.GridIndexToLocation(index);
        }

        inline Eigen::Vector4d GridIndexToLocation(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            return collision_field_.GridIndexToLocation(x_index, y_index, z_index);
        }

        inline std::vector<COLLISION_CELL>& GetMutableRawData()
        {
            return collision_field_.GetMutableRawData();
        }

        inline const std::vector<COLLISION_CELL>& GetImmutableRawData() const
        {
            return collision_field_.GetImmutableRawData();
        }

        bool SaveToFile(const std::string& filepath);

        bool LoadFromFile(const std::string &filepath);

        sdf_tools::CollisionMap GetMessageRepresentation();

        bool LoadFromMessageRepresentation(sdf_tools::CollisionMap& message);

        uint32_t UpdateConnectedComponents();

        std::map<uint32_t, std::pair<int32_t, int32_t>> ComputeComponentTopology(bool ignore_empty_components, bool recompute_connected_components, bool verbose);

        std::pair<sdf_tools::SignedDistanceField, std::pair<double, double>> ExtractSignedDistanceField(const float oob_value) const
        {
            // Make the helper function
            const std::function<bool(const COLLISION_CELL& cell)> is_filled_fn = [&] (const COLLISION_CELL& stored)
            {
                if (stored.occupancy > 0.5)
                {
                    // Mark as filled
                    return true;
                }
                else
                {
                    // Mark as free
                    return false;
                }
            };
            return sdf_generation::ExtractSignedDistanceField(collision_field_, is_filled_fn, oob_value, GetFrame());
        }

        visualization_msgs::Marker ExportForDisplay(const std_msgs::ColorRGBA& collision_color, const std_msgs::ColorRGBA& free_color, const std_msgs::ColorRGBA& unknown_color) const;

        visualization_msgs::MarkerArray ExportForSeparateDisplay(const std_msgs::ColorRGBA& collision_color, const std_msgs::ColorRGBA& free_color, const std_msgs::ColorRGBA& unknown_color) const;

        visualization_msgs::Marker ExportSurfacesForDisplay(const std_msgs::ColorRGBA& collision_color, const std_msgs::ColorRGBA& free_color, const std_msgs::ColorRGBA& unknown_color) const;

        visualization_msgs::MarkerArray ExportSurfacesForSeparateDisplay(const std_msgs::ColorRGBA& collision_color, const std_msgs::ColorRGBA& free_color, const std_msgs::ColorRGBA& unknown_color) const;

        visualization_msgs::Marker ExportConnectedComponentsForDisplay(bool color_unknown_components) const;
    };
}

#endif // COLLISION_MAP_HPP
