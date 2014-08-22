#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include "sdf_tools/voxel_grid.hpp"
#include "sdf_tools/CollisionMap.h"

#ifndef COLLISION_MAP_HPP
#define COLLISION_MAP_HPP

typedef Eigen::Transform<double, 3, Eigen::Affine> Transformation;

namespace sdf_tools
{

    typedef struct {
        float occupancy;
        u_int32_t component;
    } collision_cell;

    std::vector<u_int8_t> CollisionCellToBinary(collision_cell value)
    {
        std::vector<u_int8_t> binary(8);
        u_int32_t occupancy_binary_value = *(u_int32_t*) &value.occupancy;
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

    collision_cell CollisionCellFromBinary(std::vector<u_int8_t>& binary)
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
            occupancy_binary_value  = occupancy_binary_value | binary[0];
            occupancy_binary_value = occupancy_binary_value << 8;
            // Copy in byte 3
            occupancy_binary_value  = occupancy_binary_value | binary[1];
            occupancy_binary_value = occupancy_binary_value << 8;
            // Copy in byte 2
            occupancy_binary_value  = occupancy_binary_value | binary[2];
            occupancy_binary_value = occupancy_binary_value << 8;
            // Copy in byte 1, least-significant byte
            occupancy_binary_value  = occupancy_binary_value | binary[3];
            // Convert binary to float and store
            loaded.occupancy = *(float*) &occupancy_binary_value;
            u_int32_t component_binary_value = 0;
            // Copy in byte 4, most-significant byte
            component_binary_value  = component_binary_value | binary[4];
            component_binary_value = component_binary_value << 8;
            // Copy in byte 3
            component_binary_value  = component_binary_value | binary[5];
            component_binary_value = component_binary_value << 8;
            // Copy in byte 2
            component_binary_value  = component_binary_value | binary[6];
            component_binary_value = component_binary_value << 8;
            // Copy in byte 1, least-significant byte
            component_binary_value  = component_binary_value | binary[7];
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

        typedef struct {
            int64_t x;
            int64_t y;
            int64_t z;
        } grid_index;

        inline std::string GenerateGridIndexKey(grid_index& index)
        {
            char raw_key[(16 * 3) + 1];
            // Hex unsigned 64-bit int, pad with zeros, fixed number (16) of characters to print
            // %0 16 lx
            sprintf(raw_key, "%016lx%016lx%016lx", (u_int64_t)index.x, (u_int64_t)index.y, (u_int64_t)index.z);
            return std::string(raw_key);
        }

        std::string frame_;
        VOXEL_GRID::VoxelGrid<collision_cell> collision_field_;

        std::vector<u_int8_t> decompress_bytes(std::vector<u_int8_t>& compressed);

        std::vector<u_int8_t> compress_bytes(std::vector<u_int8_t>& uncompressed);

        std::vector<u_int8_t> PackBinaryRepresentation(std::vector<collision_cell>& raw);

        std::vector<collision_cell> UnpackBinaryRepresentation(std::vector<u_int8_t>& packed);

        int64_t MarkConnectedComponent(int64_t x_index, int64_t y_index, int64_t z_index, u_int32_t connected_component);

        std_msgs::ColorRGBA GenerateComponentColor(u_int32_t component);

    public:

        CollisionMapGrid(std::string frame, double resolution, double x_size, double y_size, double z_size, collision_cell OOB_value);

        CollisionMapGrid(Transformation origin_transform, std::string frame, double resolution, double x_size, double y_size, double z_size, collision_cell OOB_value);

        CollisionMapGrid()
        {
        }

        ~CollisionMapGrid()
        {
        }

        inline collision_cell Get(double x, double y, double z)
        {
            return collision_field_.Get(x, y, z).first;
        }

        inline collision_cell Get(int64_t x_index, int64_t y_index, int64_t z_index)
        {
            return collision_field_.Get(x_index, y_index, z_index).first;
        }

        inline bool Set(double x, double y, double z, collision_cell value)
        {
            return collision_field_.Set(x, y, z, value);
        }

        inline bool Set(int64_t x_index, int64_t y_index, int64_t z_index, collision_cell value)
        {
            return collision_field_.Set(x_index, y_index, z_index, value);
        }

        inline bool CheckInBounds(double x, double y, double z)
        {
            return collision_field_.Get(x, y, z).second;
        }

        inline bool CheckInBounds(int64_t x_index, int64_t y_index, int64_t z_index)
        {
            return collision_field_.Get(x_index, y_index, z_index).second;
        }

        inline double GetXSize()
        {
            return collision_field_.GetXSize();
        }

        inline double GetYSize()
        {
            return collision_field_.GetYSize();
        }

        inline double GetZSize()
        {
            return collision_field_.GetZSize();
        }

        inline double GetResolution()
        {
            return collision_field_.GetCellSize();
        }

        inline collision_cell GetOOBValue()
        {
            return collision_field_.GetDefaultValue();
        }

        inline int64_t GetNumXCells()
        {
            return collision_field_.GetNumXCells();
        }

        inline int64_t GetNumYCells()
        {
            return collision_field_.GetNumYCells();
        }

        inline int64_t GetNumZCells()
        {
            return collision_field_.GetNumZCells();
        }

        inline Transformation GetOriginTransform()
        {
            return collision_field_.GetOriginTransform();
        }

        inline std::vector<int64_t> LocationToGridIndex(double x, double y, double z)
        {
            return collision_field_.LocationToGridIndex(x, y, z);
        }

        inline std::vector<double> GridIndexToLocation(int64_t x_index, int64_t y_index, int64_t z_index)
        {
            return collision_field_.GridIndexToLocation(x_index, y_index, z_index);
        }

        bool SaveToFile(std::string& filepath);

        bool LoadFromFile(std::string& filepath);

        sdf_tools::CollisionMap GetMessageRepresentation();

        bool LoadFromMessageRepresentation(sdf_tools::CollisionMap& message);

        void UpdateConnectedComponents();

        visualization_msgs::Marker ExportForDisplay(std_msgs::ColorRGBA collision_color, std_msgs::ColorRGBA free_color, std_msgs::ColorRGBA unknown_color);

        visualization_msgs::Marker ExportConnectedComponentsForDisplay(bool color_unknown_components);
    };
}

#endif // COLLISION_MAP_HPP
