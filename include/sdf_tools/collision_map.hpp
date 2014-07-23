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

    class CollisionMapGrid
    {
    protected:

        std::string frame_;
        VOXEL_GRID::VoxelGrid<int8_t> collision_field_;

        std::vector<u_int8_t> decompress_bytes(std::vector<u_int8_t>& compressed);

        std::vector<u_int8_t> compress_bytes(std::vector<u_int8_t>& uncompressed);

        std::vector<u_int8_t> PackBinaryRepresentation(std::vector<int8_t>& raw);

        std::vector<int8_t> UnpackBinaryRepresentation(std::vector<u_int8_t>& packed);

    public:

        CollisionMapGrid(std::string frame, double resolution, double x_size, double y_size, double z_size, int8_t OOB_value);

        CollisionMapGrid(Transformation origin_transform, std::string frame, double resolution, double x_size, double y_size, double z_size, int8_t OOB_value);

        CollisionMapGrid()
        {
        }

        ~CollisionMapGrid()
        {
        }

        inline int8_t Get(double x, double y, double z)
        {
            return collision_field_.Get(x, y, z).first;
        }

        inline int8_t Get(int64_t x_index, int64_t y_index, int64_t z_index)
        {
            return collision_field_.Get(x_index, y_index, z_index).first;
        }

        inline bool Set(double x, double y, double z, int8_t value)
        {
            return collision_field_.Set(x, y, z, value);
        }

        inline bool Set(int64_t x_index, int64_t y_index, int64_t z_index, int8_t value)
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

        inline float GetOOBValue()
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

        visualization_msgs::Marker ExportForDisplay(std_msgs::ColorRGBA collision_color, std_msgs::ColorRGBA free_color, std_msgs::ColorRGBA unknown_color);
    };
}

#endif // COLLISION_MAP_HPP
