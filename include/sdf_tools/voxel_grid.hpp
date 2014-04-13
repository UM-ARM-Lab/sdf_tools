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
        u_int32_t stride1_;
        u_int32_t stride2_;
        u_int32_t num_x_cells_;
        u_int32_t num_y_cells_;
        u_int32_t num_z_cells_;
        T default_value_;

        inline u_int64_t GetDataIndex(u_int32_t x_index, u_int32_t y_index, u_int32_t z_index)
        {
            return (x_index * stride1_) + (y_index * stride2_) + z_index;
        }

    public:

        VoxelGrid(Transformation origin_transform, double cell_size, double x_size, double y_size, double z_size, T default_value)
        {
            origin_transform_ = origin_transform;
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_size_ = cell_size;
            x_size_ = x_size;
            y_size_ = y_size;
            z_size_ = z_size;
            default_value_ = default_value;
            num_x_cells_ = u_int32_t(x_size_ / cell_size_);
            num_y_cells_ = u_int32_t(y_size_ / cell_size_);
            num_z_cells_ = u_int32_t(z_size_ / cell_size_);
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
            cell_size_ = cell_size;
            x_size_ = x_size;
            y_size_ = y_size;
            z_size_ = z_size;
            default_value_ = default_value;
            num_x_cells_ = u_int32_t(x_size_ / cell_size_);
            num_y_cells_ = u_int32_t(y_size_ / cell_size_);
            num_z_cells_ = u_int32_t(z_size_ / cell_size_);
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

        inline std::pair<T&, bool> operator()(u_int32_t x_index, u_int32_t y_index, u_int32_t z_index)
        {
            return Get(x_index, y_index, z_index);
        }

        inline std::pair<T&, bool> Get(double x, double y, double z)
        {
            std::vector<u_int32_t> indices = LocationToGridIndex(x, y, z);
            if (indices.size() != 3)
            {
                return std::pair<T&, bool>(default_value_, false);
            }
            else
            {
                return std::pair<T&, bool>(data_[GetDataIndex(indices[0], indices[1], indices[2])], true);
            }
        }

        inline std::pair<T&, bool> Get(u_int32_t x_index, u_int32_t y_index, u_int32_t z_index)
        {
            if (x_index >= num_x_cells_ || y_index >= num_y_cells_ || z_index >= num_z_cells_)
            {
                return std::pair<T&, bool>(default_value_, false);
            }
            else
            {
                return std::pair<T&, bool>(data_[GetDataIndex(x_index, y_index, z_index)], true);
            }
        }

        inline bool Set(double x, double y, double z, T& value)
        {
            std::vector<u_int32_t> indices = LocationToGridIndex(x, y, z);
            if (indices.size() != 3)
            {
                return false;
            }
            else
            {
                data_[GetDataIndex(indices[0], indices[1], indices[2])] = value;
                return true;
            }
        }

        inline bool Set(u_int32_t x_index, u_int32_t y_index, u_int32_t z_index, T& value)
        {
            if (x_index >= num_x_cells_ || y_index >= num_y_cells_ || z_index >= num_z_cells_)
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

        inline u_int32_t GetNumXCells()
        {
            return num_x_cells_;
        }

        inline u_int32_t GetNumYCells()
        {
            return num_y_cells_;
        }

        inline u_int32_t GetNumZCells()
        {
            return num_z_cells_;
        }

        inline Transformation GetOriginTransform()
        {
            return origin_transform_;
        }

        inline std::vector<u_int32_t> LocationToGridIndex(double x, double y, double z)
        {
            Eigen::Vector3d point(x, y, z);
            Eigen::Vector3d point_in_grid_frame = inverse_origin_transform_ * point;
            long x_cell = long(point_in_grid_frame.x() / cell_size_);
            long y_cell = long(point_in_grid_frame.y() / cell_size_);
            long z_cell = long(point_in_grid_frame.z() / cell_size_);
            if (x_cell < 0 || y_cell < 0 || z_cell < 0)
            {
                return std::vector<u_int32_t>();
            }
            else if (x_cell >= num_x_cells_ || y_cell >= num_y_cells_ || z_cell >= num_z_cells_)
            {
                return std::vector<u_int32_t>();
            }
            else
            {
                std::vector<u_int32_t> indices(3);
                indices[0] = u_int32_t(x_cell);
                indices[1] = u_int32_t(y_cell);
                indices[2] = u_int32_t(z_cell);
                return indices;
            }
        }

        inline std::vector<double> GridIndexToLocation(u_int32_t x_index, u_int32_t y_index, u_int32_t z_index)
        {
            Eigen::Vector3d point_in_grid_frame(cell_size_ * (double(x_index) + 0.5), cell_size_ * (double(y_index) + 0.5), cell_size_ * (double(z_index) + 0.5));
            Eigen::Vector3d point = origin_transform_ * point_in_grid_frame;
            std::vector<double> location(3);
            location[0] = point.x();
            location[1] = point.y();
            location[2] = point.z();
            return location;
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
            u_int64_t expected_length = x_size_ * y_size_ * z_size_;
            if (data.size() != expected_length)
            {
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
