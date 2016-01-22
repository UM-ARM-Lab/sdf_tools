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
#include "arc_utilities/voxel_grid.hpp"
#include "sdf_tools/ReachabilityMap.h"

#ifndef REACHABILITY_MAP_HPP
#define REACHABILITY_MAP_HPP

#ifndef REACHABILITY_MAP_CELL_TYPE
#define REACHABILITY_MAP_CELL_TYPE u_int64_t
#endif

namespace sdf_tools
{
    inline std::vector<u_int8_t> ReachabilityToBinary(const REACHABILITY_MAP_CELL_TYPE& value)
    {
        std::vector<u_int8_t> binary(sizeof(REACHABILITY_MAP_CELL_TYPE));
        memcpy(&binary.front(), &value, sizeof(REACHABILITY_MAP_CELL_TYPE));
        return binary;
    }

    inline REACHABILITY_MAP_CELL_TYPE ReachabilityFromBinary(const std::vector<u_int8_t>& binary)
    {
        if (binary.size() != sizeof(REACHABILITY_MAP_CELL_TYPE))
        {
            std::cerr << "Binary value is not " << sizeof(REACHABILITY_MAP_CELL_TYPE) << " bytes" << std::endl;
            return REACHABILITY_MAP_CELL_TYPE();
        }
        else
        {
            REACHABILITY_MAP_CELL_TYPE loaded;
            memcpy(&loaded, &binary.front(), sizeof(REACHABILITY_MAP_CELL_TYPE));
            return loaded;
        }
    }

    class ReachabilityMapGrid
    {
    protected:

        bool initialized_;
        std::string frame_;
        VoxelGrid::VoxelGrid<REACHABILITY_MAP_CELL_TYPE> reachability_map_;

        std::vector<u_int8_t> PackBinaryRepresentation(const std::vector<REACHABILITY_MAP_CELL_TYPE>& raw) const;

        std::vector<REACHABILITY_MAP_CELL_TYPE> UnpackBinaryRepresentation(const std::vector<u_int8_t>& packed) const;

    public:

        inline ReachabilityMapGrid(std::string frame, double resolution, double x_size, double y_size, double z_size, const REACHABILITY_MAP_CELL_TYPE& default_value) : initialized_(true)
        {
            frame_ = frame;
            VoxelGrid::VoxelGrid<REACHABILITY_MAP_CELL_TYPE> new_field(resolution, x_size, y_size, z_size, default_value);
            reachability_map_ = new_field;
        }

        inline ReachabilityMapGrid(Eigen::Affine3d origin_transform, std::string frame, double resolution, double x_size, double y_size, double z_size, const REACHABILITY_MAP_CELL_TYPE& default_value) : initialized_(true)
        {
            frame_ = frame;
            VoxelGrid::VoxelGrid<REACHABILITY_MAP_CELL_TYPE> new_field(origin_transform, resolution, x_size, y_size, z_size, default_value);
            reachability_map_ = new_field;
        }

        inline ReachabilityMapGrid() : initialized_(false) {}

        inline bool IsInitialized() const
        {
            return initialized_;
        }

        inline std::pair<const REACHABILITY_MAP_CELL_TYPE&, bool> GetImmutable(const Eigen::Vector3d& location) const
        {
            return reachability_map_.GetImmutable(location);
        }

        inline std::pair<const REACHABILITY_MAP_CELL_TYPE&, bool> GetImmutable(const double x, const double y, const double z) const
        {
            return reachability_map_.GetImmutable(x, y, z);
        }

        inline std::pair<const REACHABILITY_MAP_CELL_TYPE&, bool> GetImmutable(const VoxelGrid::GRID_INDEX& index) const
        {
            return reachability_map_.GetImmutable(index);
        }

        inline std::pair<const REACHABILITY_MAP_CELL_TYPE&, bool> GetImmutable(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            return reachability_map_.GetImmutable(x_index, y_index, z_index);
        }

        inline std::pair<REACHABILITY_MAP_CELL_TYPE&, bool> GetMutable(const Eigen::Vector3d& location)
        {
            return reachability_map_.GetMutable(location);
        }

        inline std::pair<REACHABILITY_MAP_CELL_TYPE&, bool> GetMutable(const double x, const double y, const double z)
        {
            return reachability_map_.GetMutable(x, y, z);
        }

        inline std::pair<REACHABILITY_MAP_CELL_TYPE&, bool> GetMutable(const VoxelGrid::GRID_INDEX& index)
        {
            return reachability_map_.GetMutable(index);
        }

        inline std::pair<REACHABILITY_MAP_CELL_TYPE&, bool> GetMutable(const int64_t x_index, const int64_t y_index, const int64_t z_index)
        {
            return reachability_map_.GetMutable(x_index, y_index, z_index);
        }

        inline bool Set(const double x, const double y, const double z, const REACHABILITY_MAP_CELL_TYPE& value)
        {
            return reachability_map_.SetValue(x, y, z, value);
        }

        inline bool Set(const Eigen::Vector3d& location, const REACHABILITY_MAP_CELL_TYPE& value)
        {
            return reachability_map_.SetValue(location, value);
        }

        inline bool Set(const int64_t x_index, const int64_t y_index, const int64_t z_index, const REACHABILITY_MAP_CELL_TYPE& value)
        {
            return reachability_map_.SetValue(x_index, y_index, z_index, value);
        }

        inline bool Set(const VoxelGrid::GRID_INDEX& index, const REACHABILITY_MAP_CELL_TYPE& value)
        {
            return reachability_map_.SetValue(index, value);
        }

        inline bool Set(const double x, const double y, const double z, REACHABILITY_MAP_CELL_TYPE&& value)
        {
            return reachability_map_.SetValue(x, y, z, value);
        }

        inline bool Set(const Eigen::Vector3d& location, REACHABILITY_MAP_CELL_TYPE&& value)
        {
            return reachability_map_.SetValue(location, value);
        }

        inline bool Set(const int64_t x_index, const int64_t y_index, const int64_t z_index, REACHABILITY_MAP_CELL_TYPE&& value)
        {
            return reachability_map_.SetValue(x_index, y_index, z_index, value);
        }

        inline bool Set(const VoxelGrid::GRID_INDEX& index, REACHABILITY_MAP_CELL_TYPE&& value)
        {
            return reachability_map_.SetValue(index, value);
        }

        inline double GetXSize() const
        {
            return reachability_map_.GetXSize();
        }

        inline double GetYSize() const
        {
            return reachability_map_.GetYSize();
        }

        inline double GetZSize() const
        {
            return reachability_map_.GetZSize();
        }

        inline double GetResolution() const
        {
            return reachability_map_.GetCellSizes()[0];
        }

        inline REACHABILITY_MAP_CELL_TYPE GetOOBValue() const
        {
            return reachability_map_.GetDefaultValue();
        }

        inline int64_t GetNumXCells() const
        {
            return reachability_map_.GetNumXCells();
        }

        inline int64_t GetNumYCells() const
        {
            return reachability_map_.GetNumYCells();
        }

        inline int64_t GetNumZCells() const
        {
            return reachability_map_.GetNumZCells();
        }

        inline Eigen::Affine3d GetOriginTransform() const
        {
            return reachability_map_.GetOriginTransform();
        }

        inline std::string GetFrame() const
        {
            return frame_;
        }

        inline std::vector<int64_t> LocationToGridIndex(const double x, const double y, const double z) const
        {
            return reachability_map_.LocationToGridIndex(x, y, z);
        }

        inline std::vector<double> GridIndexToLocation(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            return reachability_map_.GridIndexToLocation(x_index, y_index, z_index);
        }

        bool SaveToFile(const std::string& filepath) const;

        bool LoadFromFile(const std::string &filepath);

        sdf_tools::ReachabilityMap GetMessageRepresentation() const;

        bool LoadFromMessageRepresentation(const sdf_tools::ReachabilityMap& message);
    };
}

#endif // REACHABILITY_MAP_HPP
