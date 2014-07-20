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
#include "sdf_tools/SDF.h"

#ifndef SDF_HPP
#define SDF_HPP

typedef Eigen::Transform<double, 3, Eigen::Affine> Transformation;

inline std::vector<u_int8_t> FloatToBinary(float value)
{
    u_int32_t binary_value = *(u_int32_t*) &value;
    std::vector<u_int8_t> binary(4);
    // Copy byte 1, least-significant byte
    binary[3] = binary_value & 0x000000ff;
    // Copy byte 2
    binary_value = binary_value >> 8;
    binary[2] = binary_value & 0x000000ff;
    // Copy byte 3
    binary_value = binary_value >> 8;
    binary[1] = binary_value & 0x000000ff;
    // Copy byte 4, most-significant byte
    binary_value = binary_value >> 8;
    binary[0] = binary_value & 0x000000ff;
    return binary;
}

inline float FloatFromBinary(std::vector<u_int8_t>& binary)
{
    if (binary.size() != 4)
    {
        std::cerr << "Binary value is not 4 bytes" << std::endl;
        return NAN;
    }
    else
    {
        u_int32_t binary_value = 0;
        // Copy in byte 4, most-significant byte
        binary_value  = binary_value | binary[0];
        binary_value = binary_value << 8;
        // Copy in byte 3
        binary_value  = binary_value | binary[1];
        binary_value = binary_value << 8;
        // Copy in byte 2
        binary_value  = binary_value | binary[2];
        binary_value = binary_value << 8;
        // Copy in byte 1, least-significant byte
        binary_value  = binary_value | binary[3];
        // Convert binary to float and store
        float field_value = *(float*) &binary_value;
        return field_value;
    }
}

namespace sdf_tools
{

    class SignedDistanceField
    {
    protected:

        std::string frame_;
        VOXEL_GRID::VoxelGrid<float> distance_field_;

        std::vector<u_int8_t> GetInternalBinaryRepresentation(const std::vector<float> &field_data);

        std::vector<float> UnpackFieldFromBinaryRepresentation(std::vector<u_int8_t>& binary);

        std::vector<u_int8_t> decompress_bytes(std::vector<u_int8_t>& compressed);

        std::vector<u_int8_t> compress_bytes(std::vector<u_int8_t>& uncompressed);

    public:

        SignedDistanceField(std::string frame, double resolution, double x_size, double y_size, double z_size, float OOB_value);

        SignedDistanceField(Transformation origin_transform, std::string frame, double resolution, double x_size, double y_size, double z_size, float OOB_value);

        SignedDistanceField()
        {
        }

        ~SignedDistanceField()
        {
        }

        inline float Get(double x, double y, double z)
        {
            return distance_field_.Get(x, y, z).first;
        }

        inline float Get(int64_t x_index, int64_t y_index, int64_t z_index)
        {
            return distance_field_.Get(x_index, y_index, z_index).first;
        }

        inline bool Set(double x, double y, double z, float value)
        {
            return distance_field_.Set(x, y, z, value);
        }

        inline bool Set(int64_t x_index, int64_t y_index, int64_t z_index, float value)
        {
            return distance_field_.Set(x_index, y_index, z_index, value);
        }

        inline bool CheckInBounds(double x, double y, double z)
        {
            return distance_field_.Get(x, y, z).second;
        }

        inline bool CheckInBounds(int64_t x_index, int64_t y_index, int64_t z_index)
        {
            return distance_field_.Get(x_index, y_index, z_index).second;
        }

        inline double GetXSize()
        {
            return distance_field_.GetXSize();
        }

        inline double GetYSize()
        {
            return distance_field_.GetYSize();
        }

        inline double GetZSize()
        {
            return distance_field_.GetZSize();
        }

        inline double GetResolution()
        {
            return distance_field_.GetCellSize();
        }

        inline float GetOOBValue()
        {
            return distance_field_.GetDefaultValue();
        }

        inline int64_t GetNumXCells()
        {
            return distance_field_.GetNumXCells();
        }

        inline int64_t GetNumYCells()
        {
            return distance_field_.GetNumYCells();
        }

        inline int64_t GetNumZCells()
        {
            return distance_field_.GetNumZCells();
        }

        inline std::vector<double> GetGradient(double x, double y, double z)
        {
            std::vector<int64_t> indices = LocationToGridIndex(x, y, z);
            if (indices.size() != 3)
            {
                return std::vector<double>();
            }
            else
            {
                return GetGradient(indices[0], indices[1], indices[2]);
            }
        }

        inline std::vector<double> GetGradient(int64_t x_index, int64_t y_index, int64_t z_index)
        {
            // Make sure the index is inside bounds
            if ((x_index >= 0) && (y_index >= 0) && (z_index >= 0) && (x_index < GetNumXCells()) && (y_index < GetNumYCells()) && (z_index < GetNumZCells()))
            {
                // Make sure the index we're trying to query is one cell in from the edge
                if ((x_index > 0) && (y_index > 0) && (z_index > 0) && (x_index < (GetNumXCells() - 1)) && (y_index < (GetNumYCells() - 1)) && (z_index < (GetNumZCells() - 1)))
                {
                    double inv_twice_resolution = 1.0 / (2.0 * GetResolution());
                    double gx = (Get(x_index + 1, y_index, z_index) - Get(x_index - 1, y_index, z_index)) * inv_twice_resolution;
                    double gy = (Get(x_index, y_index + 1, z_index) - Get(x_index, y_index - 1, z_index)) * inv_twice_resolution;
                    double gz = (Get(x_index, y_index, z_index + 1) - Get(x_index, y_index, z_index - 1)) * inv_twice_resolution;
                    return std::vector<double>{gx, gy, gz};
                }
                // If we're on the edge, return no gradient
                else
                {
                    return std::vector<double>();
                }
            }
            // If we're out of bounds, return no gradient
            else
            {
                return std::vector<double>();
            }
        }

        inline Transformation GetOriginTransform()
        {
            return distance_field_.GetOriginTransform();
        }

        inline std::vector<int64_t> LocationToGridIndex(double x, double y, double z)
        {
            return distance_field_.LocationToGridIndex(x, y, z);
        }

        inline std::vector<double> GridIndexToLocation(int64_t x_index, int64_t y_index, int64_t z_index)
        {
            return distance_field_.GridIndexToLocation(x_index, y_index, z_index);
        }

        bool SaveToFile(std::string& filepath);

        bool LoadFromFile(std::string& filepath);

        sdf_tools::SDF GetMessageRepresentation();

        bool LoadFromMessageRepresentation(sdf_tools::SDF& message);

        visualization_msgs::Marker ExportForDisplay(float alpha=0.01);

        visualization_msgs::Marker ExportForDisplayCollisionOnly(float alpha=0.01);

        visualization_msgs::Marker ExportForDebug(float alpha=0.5);
    };
}

#endif // SDF_HPP
