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

typedef Eigen::Transform<double, 3, Eigen::Affine> Transformation;

namespace VOXEL_GRID
{
    template <typename T>
    class VoxelGrid
    {
    protected:

        Transformation origin_transform_;
        Transformation inverse_origin_transform_;
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

        inline int64_t GetDataIndex(int64_t x_index, int64_t y_index, int64_t z_index)
        {
            return (x_index * stride1_) + (y_index * stride2_) + z_index;
        }

    public:

        VoxelGrid(Transformation origin_transform, double cell_size, double x_size, double y_size, double z_size, T default_value)
        {
            origin_transform_ = origin_transform;
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_size_ = fabs(cell_size);
            x_size_ = fabs(x_size);
            y_size_ = fabs(y_size);
            z_size_ = fabs(z_size);
            default_value_ = default_value;
            num_x_cells_ = int64_t(x_size_ / cell_size_);
            num_y_cells_ = int64_t(y_size_ / cell_size_);
            num_z_cells_ = int64_t(z_size_ / cell_size_);
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            Reset(default_value_);
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
            num_x_cells_ = int64_t(x_size_ / cell_size_);
            num_y_cells_ = int64_t(y_size_ / cell_size_);
            num_z_cells_ = int64_t(z_size_ / cell_size_);
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            Reset(default_value_);
        }

        VoxelGrid()
        {
        }

        ~VoxelGrid()
        {
        }

        void Reset(T new_value)
        {
            data_.clear();
            data_.resize(num_x_cells_ * num_y_cells_ * num_z_cells_, new_value);
        }

        inline std::pair<T&, bool> operator()(double x, double y, double z)
        {
            return Get(x, y, z);
        }

        inline std::pair<T&, bool> operator()(int64_t x_index, int64_t y_index, int64_t z_index)
        {
            return Get(x_index, y_index, z_index);
        }

        inline std::pair<T&, bool> Get(double x, double y, double z)
        {
            std::vector<int64_t> indices = LocationToGridIndex(x, y, z);
            if (indices.size() != 3)
            {
                T default_value_copy = default_value_;
                return std::pair<T&, bool>(default_value_copy, false);
            }
            else
            {
                return Get(indices[0], indices[1], indices[2]);
            }
        }

        inline std::pair<T&, bool> Get(int64_t x_index, int64_t y_index, int64_t z_index)
        {
            if (x_index < 0 || y_index < 0 || z_index < 0 || x_index >= num_x_cells_ || y_index >= num_y_cells_ || z_index >= num_z_cells_)
            {
                T default_value_copy = default_value_;
                return std::pair<T&, bool>(default_value_copy, false);
            }
            else
            {
                return std::pair<T&, bool>(data_[GetDataIndex(x_index, y_index, z_index)], true);
            }
        }

        inline bool Set(double x, double y, double z, T& value)
        {
            std::vector<int64_t> indices = LocationToGridIndex(x, y, z);
            if (indices.size() != 3)
            {
                return false;
            }
            else
            {
                return Set(indices[0], indices[1], indices[2], value);
            }
        }

        inline bool Set(int64_t x_index, int64_t y_index, int64_t z_index, T& value)
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

        inline double GetXSize()
        {
            return x_size_;
        }

        inline double GetYSize()
        {
            return y_size_;
        }

        inline double GetZSize()
        {
            return z_size_;
        }

        inline double GetCellSize()
        {
            return cell_size_;
        }

        inline T GetDefaultValue()
        {
            return default_value_;
        }

        inline int64_t GetNumXCells()
        {
            return num_x_cells_;
        }

        inline int64_t GetNumYCells()
        {
            return num_y_cells_;
        }

        inline int64_t GetNumZCells()
        {
            return num_z_cells_;
        }

        inline Transformation GetOriginTransform()
        {
            return origin_transform_;
        }

        inline std::vector<int64_t> LocationToGridIndex(double x, double y, double z)
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

        inline std::vector<double> GridIndexToLocation(int64_t x_index, int64_t y_index, int64_t z_index)
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

        const std::vector<T>& GetRawData()
        {
            return data_;
        }

        std::vector<T> CopyRawData()
        {
            return data_;
        }

        bool SetRawData(std::vector<T>& data)
        {
            int64_t expected_length = num_x_cells_ * num_y_cells_ * num_z_cells_;
            if (data.size() != expected_length)
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
    };
}

#endif // VOXEL_GRID_HPP
