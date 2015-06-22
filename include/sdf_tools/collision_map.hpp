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
#include <visualization_msgs/Marker.h>
#include "arc_utilities//voxel_grid.hpp"
#include "sdf_tools/sdf.hpp"
#include "sdf_tools/CollisionMap.h"

#ifndef COLLISION_MAP_HPP
#define COLLISION_MAP_HPP

namespace sdf_tools
{
    struct COLLISION_CELL
    {
        float occupancy;
        u_int32_t component;

        COLLISION_CELL() : occupancy(0.0), component(0) {}

        COLLISION_CELL(const float in_occupancy) : occupancy(in_occupancy), component(0) {}

        COLLISION_CELL(const float in_occupancy, const u_int32_t in_component) : occupancy(in_occupancy), component(in_component) {}
    };

    inline std::vector<u_int8_t> CollisionCellToBinary(COLLISION_CELL value)
    {
        std::vector<u_int8_t> binary(8);
        u_int32_t occupancy_binary_value = 0;
        memcpy(&occupancy_binary_value, &value.occupancy, sizeof(u_int32_t));
        // Copy byte 1, least-significant byte
        binary[3] = occupancy_binary_value & 0x000000ff;
        // Copy byte 2
        occupancy_binary_value = occupancy_binary_value >> 8;
        binary[2] = occupancy_binary_value & 0x000000ff;
        // Copy byte 3
        occupancy_binary_value = occupancy_binary_value >> 8;
        binary[1] = occupancy_binary_value & 0x000000ff;
        // Copy byte 4, most-significant byte
        occupancy_binary_value = occupancy_binary_value >> 8;
        binary[0] = occupancy_binary_value & 0x000000ff;
        u_int32_t component_binary_value = value.component;
        // Copy byte 1, least-significant byte
        binary[7] = component_binary_value & 0x000000ff;
        // Copy byte 2
        component_binary_value = component_binary_value >> 8;
        binary[6] = component_binary_value & 0x000000ff;
        // Copy byte 3
        component_binary_value = component_binary_value >> 8;
        binary[5] = component_binary_value & 0x000000ff;
        // Copy byte 4, most-significant byte
        component_binary_value = component_binary_value >> 8;
        binary[4] = component_binary_value & 0x000000ff;
        return binary;
    }

    inline COLLISION_CELL CollisionCellFromBinary(std::vector<u_int8_t>& binary)
    {
        if (binary.size() != 8)
        {
            std::cerr << "Binary value is not 8 bytes" << std::endl;
            COLLISION_CELL error_cell;
            error_cell.component = 0;
            error_cell.occupancy = NAN;
            return error_cell;
        }
        else
        {
            COLLISION_CELL loaded;
            u_int32_t occupancy_binary_value = 0;
            // Copy in byte 4, most-significant byte
            occupancy_binary_value = occupancy_binary_value | binary[0];
            occupancy_binary_value = occupancy_binary_value << 8;
            // Copy in byte 3
            occupancy_binary_value = occupancy_binary_value | binary[1];
            occupancy_binary_value = occupancy_binary_value << 8;
            // Copy in byte 2
            occupancy_binary_value = occupancy_binary_value | binary[2];
            occupancy_binary_value = occupancy_binary_value << 8;
            // Copy in byte 1, least-significant byte
            occupancy_binary_value = occupancy_binary_value | binary[3];
            // Convert binary to float and store
            memcpy(&loaded.occupancy, &occupancy_binary_value, sizeof(float));
            u_int32_t component_binary_value = 0;
            // Copy in byte 4, most-significant byte
            component_binary_value = component_binary_value | binary[4];
            component_binary_value = component_binary_value << 8;
            // Copy in byte 3
            component_binary_value = component_binary_value | binary[5];
            component_binary_value = component_binary_value << 8;
            // Copy in byte 2
            component_binary_value = component_binary_value | binary[6];
            component_binary_value = component_binary_value << 8;
            // Copy in byte 1, least-significant byte
            component_binary_value = component_binary_value | binary[7];
            // Convert binary to float and store
            loaded.component = component_binary_value;
            return loaded;
        }
    }

    constexpr float ColorChannelFromHex(u_int8_t hexval)
    {
        return (float)hexval / 255.0;
    }

    class CollisionMapGrid
    {
    protected:

        inline bool IsSurfaceIndex(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
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
            u_int32_t our_component = collision_field_.GetImmutable(x_index, y_index, z_index).first.component;
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

        typedef struct
        {
            u_int32_t location[3];
            u_int32_t closest_point[3];
            double distance_square;
            int32_t update_direction;
        } bucket_cell;

        typedef VoxelGrid::VoxelGrid<bucket_cell> DistanceField;

        DistanceField BuildDistanceField(const std::vector<VoxelGrid::GRID_INDEX>& points) const;

        std::vector<std::vector<std::vector<std::vector<int>>>> MakeNeighborhoods() const;

        inline int GetDirectionNumber(const int dx, const int dy, const int dz) const
        {
            return ((dx + 1) * 9) + ((dy + 1) * 3) + (dz + 1);
        }

        inline double ComputeDistanceSquared(const int32_t x1, const int32_t y1, const int32_t z1, const int32_t x2, const int32_t y2, const int32_t z2) const
        {
            int32_t dx = x1 - x2;
            int32_t dy = y1 - y2;
            int32_t dz = z1 - z2;
            return double((dx * dx) + (dy * dy) + (dz * dz));
        }

        bool initialized_;
        std::string frame_;
        VoxelGrid::VoxelGrid<COLLISION_CELL> collision_field_;
        u_int32_t number_of_components_;
        bool components_valid_;

        std::vector<u_int8_t> PackBinaryRepresentation(std::vector<COLLISION_CELL>& raw);

        std::vector<COLLISION_CELL> UnpackBinaryRepresentation(std::vector<u_int8_t>& packed);

        int64_t MarkConnectedComponent(int64_t x_index, int64_t y_index, int64_t z_index, u_int32_t connected_component);

        std_msgs::ColorRGBA GenerateComponentColor(u_int32_t component) const;

    public:

        CollisionMapGrid(std::string frame, double resolution, double x_size, double y_size, double z_size, COLLISION_CELL OOB_value);

        CollisionMapGrid(Eigen::Affine3d origin_transform, std::string frame, double resolution, double x_size, double y_size, double z_size, COLLISION_CELL OOB_value);

        CollisionMapGrid() : initialized_(false), number_of_components_(0), components_valid_(false) {}

        inline bool IsInitialized() const
        {
            return initialized_;
        }

        inline bool AreComponentsValid() const
        {
            return components_valid_;
        }

        inline std::pair<COLLISION_CELL, bool> Get(const Eigen::Vector3d& location) const
        {
            return collision_field_.GetImmutable(location);
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
            return collision_field_.SetWithValue(x, y, z, value);
        }

        inline bool Set(const Eigen::Vector3d& location, COLLISION_CELL value)
        {
            components_valid_ = false;
            return collision_field_.SetWithValue(location, value);
        }

        inline bool Set(const int64_t x_index, const int64_t y_index, const int64_t z_index, COLLISION_CELL value)
        {
            components_valid_ = false;
            return collision_field_.SetWithValue(x_index, y_index, z_index, value);
        }

        inline bool Set(const VoxelGrid::GRID_INDEX& index, COLLISION_CELL value)
        {
            components_valid_ = false;
            return collision_field_.SetWithValue(index, value);
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
            return collision_field_.GetCellSize();
        }

        inline COLLISION_CELL GetOOBValue() const
        {
            return collision_field_.GetDefaultValue();
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

        inline Eigen::Affine3d GetOriginTransform() const
        {
            return collision_field_.GetOriginTransform();
        }

        inline std::pair<u_int32_t, bool> GetNumConnectedComponents() const
        {
            return std::pair<u_int32_t, bool>(number_of_components_, components_valid_);
        }

        inline std::vector<int64_t> LocationToGridIndex(double x, double y, double z) const
        {
            return collision_field_.LocationToGridIndex(x, y, z);
        }

        inline std::vector<double> GridIndexToLocation(int64_t x_index, int64_t y_index, int64_t z_index) const
        {
            return collision_field_.GridIndexToLocation(x_index, y_index, z_index);
        }

        bool SaveToFile(const std::string& filepath);

        bool LoadFromFile(const std::string &filepath);

        sdf_tools::CollisionMap GetMessageRepresentation();

        bool LoadFromMessageRepresentation(sdf_tools::CollisionMap& message);

        u_int32_t UpdateConnectedComponents();

        std::map<u_int32_t, std::pair<int32_t, int32_t>> ComputeComponentTopology(bool ignore_empty_components, bool recompute_connected_components, bool verbose);

        std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, u_int8_t>> ExtractComponentSurfaces(const bool ignore_empty_components) const;

        std::pair<int32_t, int32_t> ComputeHolesInSurface(const u_int32_t component, const std::unordered_map<VoxelGrid::GRID_INDEX, u_int8_t>& surface, const bool verbose) const;

        int32_t ComputeConnectivityOfSurfaceVertices(const std::unordered_map<VoxelGrid::GRID_INDEX, u_int8_t>& surface_vertices) const;

        sdf_tools::SignedDistanceField ExtractSignedDistanceField(const float oob_value) const;

        visualization_msgs::Marker ExportForDisplay(std_msgs::ColorRGBA collision_color, std_msgs::ColorRGBA free_color, std_msgs::ColorRGBA unknown_color) const;

        visualization_msgs::Marker ExportConnectedComponentsForDisplay(bool color_unknown_components) const;
    };
}

#endif // COLLISION_MAP_HPP
