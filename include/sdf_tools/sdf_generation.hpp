#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/voxel_grid.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <sdf_tools/sdf.hpp>

#ifndef SDF_GENERATION_HPP
#define SDF_GENERATION_HPP

namespace sdf_generation
{
    struct bucket_cell
    {
        double distance_square;
        int32_t update_direction;
        uint32_t location[3];
        uint32_t closest_point[3];
    };

    typedef VoxelGrid::VoxelGrid<bucket_cell> DistanceField;

    inline int GetDirectionNumber(const int dx, const int dy, const int dz)
    {
        return ((dx + 1) * 9) + ((dy + 1) * 3) + (dz + 1);
    }

    inline std::vector<std::vector<std::vector<std::vector<int>>>> MakeNeighborhoods()
    {
        std::vector<std::vector<std::vector<std::vector<int>>>> neighborhoods;
        neighborhoods.resize(2);
        for (size_t n = 0; n < neighborhoods.size(); n++)
        {
            neighborhoods[n].resize(27);
            // Loop through the source directions
            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        int direction_number = GetDirectionNumber(dx, dy, dz);
                        // Loop through the target directions
                        for (int tdx = -1; tdx <= 1; tdx++)
                        {
                            for (int tdy = -1; tdy <= 1; tdy++)
                            {
                                for (int tdz = -1; tdz <= 1; tdz++)
                                {
                                    if (tdx == 0 && tdy == 0 && tdz == 0)
                                    {
                                        continue;
                                    }
                                    if (n >= 1)
                                    {
                                        if ((abs(tdx) + abs(tdy) + abs(tdz)) != 1)
                                        {
                                            continue;
                                        }
                                        if ((dx * tdx) < 0 || (dy * tdy) < 0 || (dz * tdz) < 0)
                                        {
                                            continue;
                                        }
                                    }
                                    std::vector<int> new_point;
                                    new_point.resize(3);
                                    new_point[0] = tdx;
                                    new_point[1] = tdy;
                                    new_point[2] = tdz;
                                    neighborhoods[n][direction_number].push_back(new_point);
                                }
                            }
                        }
                    }
                }
            }
        }
        return neighborhoods;
    }

    inline double ComputeDistanceSquared(const int32_t x1, const int32_t y1, const int32_t z1, const int32_t x2, const int32_t y2, const int32_t z2)
    {
        int32_t dx = x1 - x2;
        int32_t dy = y1 - y2;
        int32_t dz = z1 - z2;
        return double((dx * dx) + (dy * dy) + (dz * dz));
    }

    template<typename T, typename Allocator=std::allocator<T>, typename BackingStore=std::vector<T, Allocator>>
    inline DistanceField BuildDistanceField(const VoxelGrid::VoxelGrid<T, Allocator, BackingStore>& grid, const std::vector<VoxelGrid::GRID_INDEX>& points)
    {
        const Eigen::Vector3d cell_sizes = grid.GetCellSizes();
        if ((cell_sizes.x() != cell_sizes.y()) || (cell_sizes.x() != cell_sizes.z()))
        {
            throw std::invalid_argument("Grid must have uniform resolution");
        }
        // Make the DistanceField container
        bucket_cell default_cell;
        default_cell.distance_square = std::numeric_limits<double>::infinity();
        DistanceField distance_field(grid.GetOriginTransform(), cell_sizes.x(), grid.GetXSize(), grid.GetYSize(), grid.GetZSize(), default_cell);
        // Compute maximum distance square
        long max_distance_square = (distance_field.GetNumXCells() * distance_field.GetNumXCells()) + (distance_field.GetNumYCells() * distance_field.GetNumYCells()) + (distance_field.GetNumZCells() * distance_field.GetNumZCells());
        // Make bucket queue
        std::vector<std::vector<bucket_cell>> bucket_queue(max_distance_square + 1);
        bucket_queue[0].reserve(points.size());
        // Set initial update direction
        int initial_update_direction = GetDirectionNumber(0, 0, 0);
        // Mark all points with distance zero and add to the bucket queue
        for (size_t index = 0; index < points.size(); index++)
        {
            const VoxelGrid::GRID_INDEX& current_index = points[index];
            std::pair<bucket_cell&, bool> query = distance_field.GetMutable(current_index);
            if (query.second)
            {
                query.first.location[0] = current_index.x;
                query.first.location[1] = current_index.y;
                query.first.location[2] = current_index.z;
                query.first.closest_point[0] = current_index.x;
                query.first.closest_point[1] = current_index.y;
                query.first.closest_point[2] = current_index.z;
                query.first.distance_square = 0.0;
                query.first.update_direction = initial_update_direction;
                bucket_queue[0].push_back(query.first);
            }
            // If the point is outside the bounds of the SDF, skip
            else
            {
                continue;
            }
        }
        // Process the bucket queue
        std::vector<std::vector<std::vector<std::vector<int>>>> neighborhoods = MakeNeighborhoods();
        for (size_t bq_idx = 0; bq_idx < bucket_queue.size(); bq_idx++)
        {
            std::vector<bucket_cell>::iterator queue_itr = bucket_queue[bq_idx].begin();
            while (queue_itr != bucket_queue[bq_idx].end())
            {
                // Get the current location
                bucket_cell& cur_cell = *queue_itr;
                double x = cur_cell.location[0];
                double y = cur_cell.location[1];
                double z = cur_cell.location[2];
                // Pick the update direction
                int D = bq_idx;
                if (D > 1)
                {
                    D = 1;
                }
                // Make sure the update direction is valid
                if (cur_cell.update_direction < 0 || cur_cell.update_direction > 26)
                {
                    ++queue_itr;
                    continue;
                }
                // Get the current neighborhood list
                std::vector<std::vector<int>>& neighborhood = neighborhoods[D][cur_cell.update_direction];
                // Update the distance from the neighboring cells
                for (size_t nh_idx = 0; nh_idx < neighborhood.size(); nh_idx++)
                {
                    // Get the direction to check
                    int dx = neighborhood[nh_idx][0];
                    int dy = neighborhood[nh_idx][1];
                    int dz = neighborhood[nh_idx][2];
                    int nx = x + dx;
                    int ny = y + dy;
                    int nz = z + dz;
                    std::pair<bucket_cell&, bool> neighbor_query = distance_field.GetMutable((int64_t)nx, (int64_t)ny, (int64_t)nz);
                    if (!neighbor_query.second)
                    {
                        // "Neighbor" is outside the bounds of the SDF
                        continue;
                    }
                    // Update the neighbor's distance based on the current
                    int new_distance_square = ComputeDistanceSquared(nx, ny, nz, cur_cell.closest_point[0], cur_cell.closest_point[1], cur_cell.closest_point[2]);
                    if (new_distance_square > max_distance_square)
                    {
                        // Skip these cases
                        continue;
                    }
                    if (new_distance_square < neighbor_query.first.distance_square)
                    {
                        // If the distance is better, time to update the neighbor
                        neighbor_query.first.distance_square = new_distance_square;
                        neighbor_query.first.closest_point[0] = cur_cell.closest_point[0];
                        neighbor_query.first.closest_point[1] = cur_cell.closest_point[1];
                        neighbor_query.first.closest_point[2] = cur_cell.closest_point[2];
                        neighbor_query.first.location[0] = nx;
                        neighbor_query.first.location[1] = ny;
                        neighbor_query.first.location[2] = nz;
                        neighbor_query.first.update_direction = GetDirectionNumber(dx, dy, dz);
                        // Add the neighbor into the bucket queue
                        bucket_queue[new_distance_square].push_back(neighbor_query.first);
                    }
                }
                // Increment the queue iterator
                ++queue_itr;
            }
            // Clear the current queue now that we're done with it
            bucket_queue[bq_idx].clear();
        }
        return distance_field;
    }

    template<typename T, typename Allocator=std::allocator<T>, typename BackingStore=std::vector<T, Allocator>>
    inline std::pair<sdf_tools::SignedDistanceField, std::pair<double, double>> ExtractSignedDistanceField(const VoxelGrid::VoxelGrid<T, Allocator, BackingStore>& grid, const std::function<bool(const VoxelGrid::GRID_INDEX&)>& is_filled_fn, const float oob_value, const std::string& frame)
    {
        const Eigen::Vector3d cell_sizes = grid.GetCellSizes();
        if ((cell_sizes.x() != cell_sizes.y()) || (cell_sizes.x() != cell_sizes.z()))
        {
            throw std::invalid_argument("Grid must have uniform resolution");
        }
        std::vector<VoxelGrid::GRID_INDEX> filled;
        std::vector<VoxelGrid::GRID_INDEX> free;
        for (int64_t x_index = 0; x_index < grid.GetNumXCells(); x_index++)
        {
            for (int64_t y_index = 0; y_index < grid.GetNumYCells(); y_index++)
            {
                for (int64_t z_index = 0; z_index < grid.GetNumZCells(); z_index++)
                {
                    const VoxelGrid::GRID_INDEX current_index(x_index, y_index, z_index);
                    if (is_filled_fn(current_index))
                    {
                        // Mark as filled
                        filled.push_back(current_index);
                    }
                    else
                    {
                        // Mark as free space
                        free.push_back(current_index);
                    }
                }
            }
        }
        // Make two distance fields (one for distance to filled voxels, one for distance to free voxels
        const DistanceField filled_distance_field = BuildDistanceField(grid, filled);
        const DistanceField free_distance_field = BuildDistanceField(grid, free);
        // Generate the SDF
        sdf_tools::SignedDistanceField new_sdf(grid.GetOriginTransform(), frame, cell_sizes.x(), grid.GetXSize(), grid.GetYSize(), grid.GetZSize(), oob_value);
        double max_distance = -std::numeric_limits<double>::infinity();
        double min_distance = std::numeric_limits<double>::infinity();
        for (int64_t x_index = 0; x_index < filled_distance_field.GetNumXCells(); x_index++)
        {
            for (int64_t y_index = 0; y_index < filled_distance_field.GetNumYCells(); y_index++)
            {
                for (int64_t z_index = 0; z_index < filled_distance_field.GetNumZCells(); z_index++)
                {
                    const double distance1 = std::sqrt(filled_distance_field.GetImmutable(x_index, y_index, z_index).first.distance_square) * new_sdf.GetResolution();
                    const double distance2 = std::sqrt(free_distance_field.GetImmutable(x_index, y_index, z_index).first.distance_square) * new_sdf.GetResolution();
                    const double distance = distance1 - distance2;
                    if (distance > max_distance)
                    {
                        max_distance = distance;
                    }
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                    }
                    new_sdf.SetValue(x_index, y_index, z_index, distance);
                }
            }
        }
        std::pair<double, double> extrema(max_distance, min_distance);
        return std::pair<sdf_tools::SignedDistanceField, std::pair<double, double>>(new_sdf, extrema);
    }

    template<typename T, typename Allocator=std::allocator<T>, typename BackingStore=std::vector<T, Allocator>>
    inline std::pair<sdf_tools::SignedDistanceField, std::pair<double, double>> ExtractSignedDistanceField(const VoxelGrid::VoxelGrid<T, Allocator, BackingStore>& grid, const std::function<bool(const T&)>& is_filled_fn, const float oob_value, const std::string& frame)
    {
        const std::function<bool(const VoxelGrid::GRID_INDEX&)> real_is_filled_fn = [&] (const VoxelGrid::GRID_INDEX& index)
        {
            const T& stored = grid.GetImmutable(index).first;
            // If it matches an object to use OR there are no objects supplied
            if (is_filled_fn(stored))
            {
                // Mark as filled
                return true;
            }
            else
            {
                // Mark as free space
                return false;
            }
        };
        return ExtractSignedDistanceField(grid, real_is_filled_fn, oob_value, frame);
    }
}

#endif // SDF_GENERATION_HPP
