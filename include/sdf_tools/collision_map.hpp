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
#include "sdf_tools/CollisionMap.h"

#ifndef COLLISION_MAP_HPP
#define COLLISION_MAP_HPP

namespace sdf_tools
{
    typedef struct
    {
        float occupancy;
        u_int32_t component;
    } collision_cell;

    inline std::vector<u_int8_t> CollisionCellToBinary(collision_cell value)
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

    inline collision_cell CollisionCellFromBinary(std::vector<u_int8_t>& binary)
    {
        if (binary.size() != 8)
        {
            std::cerr << "Binary value is not 8 bytes" << std::endl;
            collision_cell error_cell;
            error_cell.component = 0;
            error_cell.occupancy = NAN;
            return error_cell;
        }
        else
        {
            collision_cell loaded;
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

        std::string frame_;
        VoxelGrid::VoxelGrid<collision_cell> collision_field_;
        u_int32_t number_of_components_;
        bool components_valid_;

        std::vector<u_int8_t> PackBinaryRepresentation(std::vector<collision_cell>& raw);

        std::vector<collision_cell> UnpackBinaryRepresentation(std::vector<u_int8_t>& packed);

        int64_t MarkConnectedComponent(int64_t x_index, int64_t y_index, int64_t z_index, u_int32_t connected_component);

        std_msgs::ColorRGBA GenerateComponentColor(u_int32_t component) const;

    public:

        CollisionMapGrid(std::string frame, double resolution, double x_size, double y_size, double z_size, collision_cell OOB_value);

        CollisionMapGrid(Eigen::Affine3d origin_transform, std::string frame, double resolution, double x_size, double y_size, double z_size, collision_cell OOB_value);

        CollisionMapGrid() : number_of_components_(0), components_valid_(false) {}

        inline std::pair<collision_cell, bool> Get(const double x, const double y, const double z) const
        {
            return collision_field_.GetImmutable(x, y, z);
        }

        inline std::pair<collision_cell, bool> Get(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            return collision_field_.GetImmutable(x_index, y_index, z_index);
        }

        inline bool Set(double x, double y, double z, collision_cell value)
        {
            return collision_field_.SetWithValue(x, y, z, value);
        }

        inline bool Set(int64_t x_index, int64_t y_index, int64_t z_index, collision_cell value)
        {
            return collision_field_.SetWithValue(x_index, y_index, z_index, value);
        }

        inline bool CheckInBounds(double x, double y, double z) const
        {
            return collision_field_.GetImmutable(x, y, z).second;
        }

        inline bool CheckInBounds(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            if (x_index >= 0 && y_index >= 0 && z_index >= 0 && x_index < GetNumXCells() && y_index < GetNumYCells() && z_index < GetNumZCells())
            {
                return true;
            }
            else
            {
                return true;
            }
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

        inline collision_cell GetOOBValue() const
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

        bool SaveToFile(std::string& filepath);

        bool LoadFromFile(std::string& filepath);

        sdf_tools::CollisionMap GetMessageRepresentation();

        bool LoadFromMessageRepresentation(sdf_tools::CollisionMap& message);

        u_int32_t UpdateConnectedComponents();

        std::map<u_int32_t, std::pair<int32_t, int32_t>> ComputeComponentTopology(bool ignore_empty_components, bool recompute_connected_components, bool verbose);

        std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, u_int8_t>> ExtractComponentSurfaces(const bool ignore_empty_components) const;

        std::pair<int32_t, int32_t> ComputeHolesInSurface(const u_int32_t component, const std::unordered_map<VoxelGrid::GRID_INDEX, u_int8_t>& surface, const bool verbose) const;

        int32_t ComputeConnectivityOfSurfaceVertices(const std::unordered_map<VoxelGrid::GRID_INDEX, u_int8_t>& surface_vertices) const;

        visualization_msgs::Marker ExportForDisplay(std_msgs::ColorRGBA collision_color, std_msgs::ColorRGBA free_color, std_msgs::ColorRGBA unknown_color) const;

        visualization_msgs::Marker ExportConnectedComponentsForDisplay(bool color_unknown_components) const;
    };
}

#endif // COLLISION_MAP_HPP
