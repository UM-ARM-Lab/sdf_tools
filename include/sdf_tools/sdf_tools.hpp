#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <Eigen/Geometry>
#include "sdf_tools/voxel_grid.hpp"
#include "sdf_tools/SDF.h"

#ifndef SDF_TOOLS_HPP
#define SDF_TOOLS_HPP

typedef Eigen::Transform<double, 3, Eigen::Affine> Transformation;

namespace SDF_TOOLS
{
    std::vector<u_int8_t> decompress_bytes(std::vector<u_int8_t>& compressed);

    std::vector<u_int8_t> compress_bytes(std::vector<u_int8_t>& uncompressed);

    class SignedDistanceField
    {
    protected:

        VOXEL_GRID::VoxelGrid<double> distance_field_;

    public:

        SignedDistanceField(double resolution, double x_size, double y_size, double z_size, double OOB_value);

        SignedDistanceField()
        {
        }

        ~SignedDistanceField()
        {
        }

        inline double Lookup(double x, double y, double z)
        {
            ;
        }

        inline bool CheckInBounds(double x, double y, double z)
        {
            ;
        }

        std::vector<u_int8_t> GetBinaryRepresentation();

        bool LoadFromBinaryRepresentation(std::vector<u_int8_t>& compressed);

        bool SaveToFile(std::string& filepath);

        bool LoadFromFile(std::string& filepath);

        sdf_tools::SDF GetMessageRepresentation();

        bool LoadFromMessageRepresentation(sdf_tools::SDF& message);
    };
}

#endif // SDF_TOOLS_HPP
