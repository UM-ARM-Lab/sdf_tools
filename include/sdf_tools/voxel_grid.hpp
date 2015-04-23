#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <Eigen/Geometry>

#ifndef VOXEL_GRID_HPP
#define VOXEL_GRID_HPP

namespace VOXEL_GRID
{
    struct GRID_INDEX
    {
        int64_t x;
        int64_t y;
        int64_t z;

        GRID_INDEX() : x(0), y(0), z(0) {}

        GRID_INDEX(const int64_t in_x, const int64_t in_y, const int64_t in_z) : x(in_x), y(in_y), z(in_z) {}

        bool operator==(const GRID_INDEX& other) const
        {
            return (x == other.x && y == other.y && z == other.z);
        }
    };

    template <typename T>
    class VoxelGrid
    {
    protected:

        bool initialized_;
        Eigen::Affine3d origin_transform_;
        Eigen::Affine3d inverse_origin_transform_;
        std::vector<T> data_;
        double cell_size_;
        double x_size_;
        double y_size_;
        double z_size_;
        int64_t stride1_;
        int64_t stride2_;
        int64_t num_x_cells_;
        int64_t num_y_cells_;
        int64_t num_z_cells_;
        T default_value_;
        T oob_value_;

        inline int64_t GetDataIndex(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            return (x_index * stride1_) + (y_index * stride2_) + z_index;
        }

        void SetContents(T value)
        {
            data_.clear();
            data_.resize(num_x_cells_ * num_y_cells_ * num_z_cells_, value);
        }

    public:

        VoxelGrid(Eigen::Affine3d origin_transform, double cell_size, double x_size, double y_size, double z_size, T default_value)
        {
            origin_transform_ = origin_transform;
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_size_ = fabs(cell_size);
            x_size_ = fabs(x_size);
            y_size_ = fabs(y_size);
            z_size_ = fabs(z_size);
            default_value_ = default_value;
            oob_value_ = default_value;
            num_x_cells_ = int64_t(ceil(x_size_ / cell_size_));
            num_y_cells_ = int64_t(ceil(y_size_ / cell_size_));
            num_z_cells_ = int64_t(ceil(z_size_ / cell_size_));
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
            initialized_ = true;
        }

        VoxelGrid(Eigen::Affine3d origin_transform, double cell_size, double x_size, double y_size, double z_size, T default_value, T oob_value)
        {
            origin_transform_ = origin_transform;
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_size_ = fabs(cell_size);
            x_size_ = fabs(x_size);
            y_size_ = fabs(y_size);
            z_size_ = fabs(z_size);
            default_value_ = default_value;
            oob_value_ = oob_value;
            num_x_cells_ = int64_t(ceil(x_size_ / cell_size_));
            num_y_cells_ = int64_t(ceil(y_size_ / cell_size_));
            num_z_cells_ = int64_t(ceil(z_size_ / cell_size_));
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
            initialized_ = true;
        }

        VoxelGrid(double cell_size, double x_size, double y_size, double z_size, T default_value)
        {
            Eigen::Translation3d origin_translation(-x_size * 0.5, -y_size * 0.5, -z_size * 0.5);
            Eigen::Quaterniond origin_rotation;
            origin_rotation.setIdentity();
            origin_transform_ = origin_translation * origin_rotation;
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_size_ = fabs(cell_size);
            x_size_ = fabs(x_size);
            y_size_ = fabs(y_size);
            z_size_ = fabs(z_size);
            default_value_ = default_value;
            oob_value_ = default_value;
            num_x_cells_ = int64_t(ceil(x_size_ / cell_size_));
            num_y_cells_ = int64_t(ceil(y_size_ / cell_size_));
            num_z_cells_ = int64_t(ceil(z_size_ / cell_size_));
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
            initialized_ = true;
        }

        VoxelGrid(double cell_size, double x_size, double y_size, double z_size, T default_value, T oob_value)
        {
            Eigen::Translation3d origin_translation(-x_size * 0.5, -y_size * 0.5, -z_size * 0.5);
            Eigen::Quaterniond origin_rotation;
            origin_rotation.setIdentity();
            origin_transform_ = origin_translation * origin_rotation;
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_size_ = fabs(cell_size);
            x_size_ = fabs(x_size);
            y_size_ = fabs(y_size);
            z_size_ = fabs(z_size);
            default_value_ = default_value;
            oob_value_ = oob_value;
            num_x_cells_ = int64_t(ceil(x_size_ / cell_size_));
            num_y_cells_ = int64_t(ceil(y_size_ / cell_size_));
            num_z_cells_ = int64_t(ceil(z_size_ / cell_size_));
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
            initialized_ = true;
        }

        VoxelGrid()
        {
            origin_transform_ = Eigen::Affine3d::Identity();
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_size_ = 0.0;
            x_size_ = 0.0;
            y_size_ = 0.0;
            z_size_ = 0.0;
            num_x_cells_ = 0;
            num_y_cells_ = 0;
            num_z_cells_ = 0;
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            initialized_ = false;
        }

        inline bool IsInitialized() const
        {
            return initialized_;
        }

        void ResetWithDefault()
        {
            SetContents(default_value_);
        }

        void ResetWithNewValue(T new_value)
        {
            SetContents(new_value);
        }

        void ResetWithNewDefault(T new_default)
        {
            default_value_ = new_default;
            SetContents(default_value_);
        }

        inline std::pair<const T&, bool> GetImmutable(const Eigen::Vector3d& location) const
        {
            return GetImmutable(location.x(), location.y(), location.z());
        }

        inline std::pair<const T&, bool> GetImmutable(const double x, const double y, const double z) const
        {
            std::vector<int64_t> indices = LocationToGridIndex(x, y, z);
            if (indices.size() != 3)
            {
                return std::pair<const T&, bool>(oob_value_, false);
            }
            else
            {
                return GetImmutable(indices[0], indices[1], indices[2]);
            }
        }

        inline std::pair<const T&, bool> GetImmutable(const GRID_INDEX& index) const
        {
            return GetImmutable(index.x, index.y, index.z);
        }

        inline std::pair<const T&, bool> GetImmutable(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            if (x_index < 0 || y_index < 0 || z_index < 0 || x_index >= num_x_cells_ || y_index >= num_y_cells_ || z_index >= num_z_cells_)
            {
                return std::pair<const T&, bool>(oob_value_, false);
            }
            else
            {
                return std::pair<const T&, bool>(data_[GetDataIndex(x_index, y_index, z_index)], true);
            }
        }

        inline std::pair<T, bool> GetCopy(const Eigen::Vector3d& location) const
        {
            return GetCopy(location.x(), location.y(), location.z());
        }

        inline std::pair<T, bool> GetCopy(const double x, const double y, const double z) const
        {
            std::vector<int64_t> indices = LocationToGridIndex(x, y, z);
            if (indices.size() != 3)
            {
                return std::pair<T, bool>(oob_value_, false);
            }
            else
            {
                return GetCopy(indices[0], indices[1], indices[2]);
            }
        }

        inline std::pair<T, bool> GetCopy(const GRID_INDEX& index) const
        {
            return GetCopy(index.x, index.y, index.z);
        }

        inline std::pair<T, bool> GetCopy(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            if (x_index < 0 || y_index < 0 || z_index < 0 || x_index >= num_x_cells_ || y_index >= num_y_cells_ || z_index >= num_z_cells_)
            {
                return std::pair<T, bool>(oob_value_, false);
            }
            else
            {
                return std::pair<T, bool>(data_[GetDataIndex(x_index, y_index, z_index)], true);
            }
        }

        inline std::pair<T&, bool> GetMutable(const Eigen::Vector3d& location) const
        {
            return GetMutable(location.x(), location.y(), location.z());
        }

        inline std::pair<T&, bool> GetMutable(const double x, const double y, const double z)
        {
            std::vector<int64_t> indices = LocationToGridIndex(x, y, z);
            if (indices.size() != 3)
            {
                return std::pair<T&, bool>(oob_value_, false);
            }
            else
            {
                return GetMutable(indices[0], indices[1], indices[2]);
            }
        }

        inline std::pair<T&, bool> GetMutable(const GRID_INDEX& index)
        {
            return GetMutable(index.x, index.y, index.z);
        }

        inline std::pair<T&, bool> GetMutable(const int64_t x_index, const int64_t y_index, const int64_t z_index)
        {
            if (x_index < 0 || y_index < 0 || z_index < 0 || x_index >= num_x_cells_ || y_index >= num_y_cells_ || z_index >= num_z_cells_)
            {
                return std::pair<T&, bool>(oob_value_, false);
            }
            else
            {
                return std::pair<T&, bool>(data_[GetDataIndex(x_index, y_index, z_index)], true);
            }
        }

        inline bool SetWithReference(const Eigen::Vector3d& location, T& value)
        {
            return SetWithReference(location.x(), location.y(), location.z(), value);
        }

        inline bool SetWithReference(const double x, const double y, const double z, T& value)
        {
            std::vector<int64_t> indices = LocationToGridIndex(x, y, z);
            if (indices.size() != 3)
            {
                return false;
            }
            else
            {
                return SetWithReference(indices[0], indices[1], indices[2], value);
            }
        }

        inline bool  SetWithReference(const GRID_INDEX& index, T& value)
        {
            return SetWithReference(index.x, index.y, index.z, value);
        }

        inline bool SetWithReference(const int64_t x_index, const int64_t y_index, const int64_t z_index, T& value)
        {
            if (x_index < 0 || y_index < 0 || z_index < 0 || x_index >= num_x_cells_ || y_index >= num_y_cells_ || z_index >= num_z_cells_)
            {
                return false;
            }
            else
            {
                data_[GetDataIndex(x_index, y_index, z_index)] = value;
                return true;
            }
        }

        inline bool SetWithValue(const Eigen::Vector3d& location, T value)
        {
            return SetWithValue(location.x(), location.y(), location.z(), value);
        }

        inline bool SetWithValue(const double x, const double y, const double z, T value)
        {
            std::vector<int64_t> indices = LocationToGridIndex(x, y, z);
            if (indices.size() != 3)
            {
                return false;
            }
            else
            {
                return SetWithValue(indices[0], indices[1], indices[2], value);
            }
        }

        inline bool  SetWithValue(const GRID_INDEX& index, T value)
        {
            return SetWithValue(index.x, index.y, index.z, value);
        }

        inline bool SetWithValue(const int64_t x_index, const int64_t y_index, const int64_t z_index, T value)
        {
            if (x_index < 0 || y_index < 0 || z_index < 0 || x_index >= num_x_cells_ || y_index >= num_y_cells_ || z_index >= num_z_cells_)
            {
                return false;
            }
            else
            {
                data_[GetDataIndex(x_index, y_index, z_index)] = value;
                return true;
            }
        }

        inline double GetXSize() const
        {
            return x_size_;
        }

        inline double GetYSize() const
        {
            return y_size_;
        }

        inline double GetZSize() const
        {
            return z_size_;
        }

        inline double GetCellSize() const
        {
            return cell_size_;
        }

        inline T GetDefaultValue() const
        {
            return default_value_;
        }

        inline T GetOOBValue() const
        {
            return oob_value_;
        }

        inline void SetDefaultValue(T default_value)
        {
            default_value_ = default_value;
        }

        inline void SetOOBValue(T oob_value)
        {
            oob_value_ = oob_value;
        }

        inline int64_t GetNumXCells() const
        {
            return num_x_cells_;
        }

        inline int64_t GetNumYCells() const
        {
            return num_y_cells_;
        }

        inline int64_t GetNumZCells() const
        {
            return num_z_cells_;
        }

        inline Eigen::Affine3d GetOriginTransform() const
        {
            return origin_transform_;
        }

        inline std::vector<int64_t> LocationToGridIndex(const Eigen::Vector3d& location) const
        {
            return LocationToGridIndex(location.x(), location.y(), location.z());
        }

        inline std::vector<int64_t> LocationToGridIndex(const double x, const double y, const double z) const
        {
            Eigen::Vector3d point(x, y, z);
            Eigen::Vector3d point_in_grid_frame = inverse_origin_transform_ * point;
            int64_t x_cell = int64_t(point_in_grid_frame.x() / cell_size_);
            int64_t y_cell = int64_t(point_in_grid_frame.y() / cell_size_);
            int64_t z_cell = int64_t(point_in_grid_frame.z() / cell_size_);
            if (x_cell < 0 || y_cell < 0 || z_cell < 0 || x_cell >= num_x_cells_ || y_cell >= num_y_cells_ || z_cell >= num_z_cells_)
            {
                return std::vector<int64_t>();
            }
            else
            {
                return std::vector<int64_t>{x_cell, y_cell, z_cell};
            }
        }

        inline std::vector<double> GridIndexToLocation(const VOXEL_GRID::GRID_INDEX& index) const
        {
            return GridIndexToLocation(index.x, index.y, index.z);
        }

        inline std::vector<double> GridIndexToLocation(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            if (x_index < 0 || y_index < 0 || z_index < 0 || x_index >= num_x_cells_ || y_index >= num_y_cells_ || z_index >= num_z_cells_)
            {
                return std::vector<double>();
            }
            else
            {
                Eigen::Vector3d point_in_grid_frame(cell_size_ * (double(x_index) + 0.5), cell_size_ * (double(y_index) + 0.5), cell_size_ * (double(z_index) + 0.5));
                Eigen::Vector3d point = origin_transform_ * point_in_grid_frame;
                return std::vector<double>{point.x(), point.y(), point.z()};
            }
        }

        const std::vector<T>& GetRawData() const
        {
            return data_;
        }

        std::vector<T> CopyRawData() const
        {
            return data_;
        }

        bool SetRawData(std::vector<T>& data)
        {
            int64_t expected_length = num_x_cells_ * num_y_cells_ * num_z_cells_;
            if ((int64_t)data.size() != expected_length)
            {
                std::cerr << "Failed to load internal data - expected " << expected_length << " got " << data.size() << std::endl;
                return false;
            }
            else
            {
                data_ = data;
                return true;
            }
        }

        inline u_int64_t HashDataIndex(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            return (x_index * stride1_) + (y_index * stride2_) + z_index;
        }
    };
}

namespace std
{
    template <>
    struct hash<VOXEL_GRID::GRID_INDEX>
    {
        std::size_t operator()(const VOXEL_GRID::GRID_INDEX& index) const
        {
            using std::size_t;
            using std::hash;
            return ((std::hash<int64_t>()(index.x) ^ (std::hash<int64_t>()(index.y) << 1) >> 1) ^ (std::hash<int64_t>()(index.z) << 1));
        }
    };
}

#endif // VOXEL_GRID_HPP
