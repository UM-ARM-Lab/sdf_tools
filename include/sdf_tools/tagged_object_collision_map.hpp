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
#include "sdf_tools/sdf.hpp"
#include "sdf_tools/TaggedObjectCollisionMap.h"

#ifndef TAGGED_OBJECT_COLLISION_MAP_HPP
#define TAGGED_OBJECT_COLLISION_MAP_HPP

#define ENABLE_UNORDERED_MAP_SIZE_HINTS

namespace sdf_tools
{
    constexpr float ColorChannelFromHex(u_int8_t hexval)
    {
        return (float)hexval / 255.0;
    }

    u_int8_t ColorChannelToHex(float colorval)
    {
        assert(colorval >= 0.0);
        assert(colorval <= 1.0);
        return (u_int8_t)round(colorval * 255.0);
    }

    struct TAGGED_OBJECT_COLLISION_CELL
    {
        float occupancy;
        u_int32_t component;
        u_int32_t object_id;
        u_int8_t r;
        u_int8_t g;
        u_int8_t b;
        u_int8_t a;

        TAGGED_OBJECT_COLLISION_CELL() : occupancy(0.0), component(0u), object_id(0u), r(0u), g(0u), b(0u), a(0u) {}

        TAGGED_OBJECT_COLLISION_CELL(const float in_occupancy, const u_int32_t in_object_id) : occupancy(in_occupancy), component(0), object_id(in_object_id), r(0u), g(0u), b(0u), a(0u) {}

        TAGGED_OBJECT_COLLISION_CELL(const float in_occupancy, const u_int32_t in_object_id, const u_int8_t in_r, const u_int8_t in_g, const u_int8_t in_b, const u_int8_t in_a) : occupancy(in_occupancy), component(0), object_id(in_object_id), r(in_r), g(in_g), b(in_b), a(in_a) {}

        TAGGED_OBJECT_COLLISION_CELL(const float in_occupancy, const u_int32_t in_object_id, const u_int8_t in_r, const float in_g, const float in_b, const float in_a) : occupancy(in_occupancy), component(0), object_id(in_object_id), r(ColorChannelToHex(in_r)), g(ColorChannelToHex(in_g)), b(ColorChannelToHex(in_b)), a(ColorChannelToHex(in_a)) {}

        TAGGED_OBJECT_COLLISION_CELL(const float in_occupancy, const u_int32_t in_object_id, const u_int32_t in_component) : occupancy(in_occupancy), component(in_component), object_id(in_object_id), r(0u), g(0u), b(0u), a(0u) {}

        TAGGED_OBJECT_COLLISION_CELL(const float in_occupancy, const u_int32_t in_object_id, const u_int32_t in_component, const u_int8_t in_r, const u_int8_t in_g, const u_int8_t in_b, const u_int8_t in_a) : occupancy(in_occupancy), component(in_component), object_id(in_object_id), r(in_r), g(in_g), b(in_b), a(in_a) {}

        TAGGED_OBJECT_COLLISION_CELL(const float in_occupancy, const u_int32_t in_object_id, const u_int32_t in_component, const float in_r, const float in_g, const float in_b, const float in_a) : occupancy(in_occupancy), component(in_component), object_id(in_object_id), r(ColorChannelToHex(in_r)), g(ColorChannelToHex(in_g)), b(ColorChannelToHex(in_b)), a(ColorChannelToHex(in_a)) {}
    };

    inline std::vector<u_int8_t> TaggedObjectCollisionCellToBinary(const TAGGED_OBJECT_COLLISION_CELL& value)
    {
        std::vector<u_int8_t> binary(sizeof(TAGGED_OBJECT_COLLISION_CELL));
        memcpy(&binary.front(), &value, sizeof(TAGGED_OBJECT_COLLISION_CELL));
        return binary;
    }

    inline TAGGED_OBJECT_COLLISION_CELL TaggedObjectCollisionCellFromBinary(const std::vector<u_int8_t>& binary)
    {
        if (binary.size() != sizeof(TAGGED_OBJECT_COLLISION_CELL))
        {
            std::cerr << "Binary value is not " << sizeof(TAGGED_OBJECT_COLLISION_CELL) << " bytes" << std::endl;
            return TAGGED_OBJECT_COLLISION_CELL();
        }
        else
        {
            TAGGED_OBJECT_COLLISION_CELL loaded;
            memcpy(&loaded, &binary.front(), sizeof(TAGGED_OBJECT_COLLISION_CELL));
            return loaded;
        }
    }

    class TaggedObjectCollisionMapGrid
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
            u_int32_t our_component = GetImmutable(x_index, y_index, z_index).first.component;
            // Check neighbor 1
            if (our_component != GetImmutable(x_index, y_index, z_index - 1).first.component)
            {
                return true;
            }
            // Check neighbor 2
            else if (our_component != GetImmutable(x_index, y_index, z_index + 1).first.component)
            {
                return true;
            }
            // Check neighbor 3
            else if (our_component != GetImmutable(x_index, y_index - 1, z_index).first.component)
            {
                return true;
            }
            // Check neighbor 4
            else if (our_component != GetImmutable(x_index, y_index + 1, z_index).first.component)
            {
                return true;
            }
            // Check neighbor 5
            else if (our_component != GetImmutable(x_index - 1, y_index, z_index).first.component)
            {
                return true;
            }
            // Check neighbor 6
            else if (our_component != GetImmutable(x_index + 1, y_index, z_index).first.component)
            {
                return true;
            }
            // If none of the faces are exposed, it's not a surface voxel
            return false;
        }

        struct bucket_cell
        {
            u_int32_t location[3];
            u_int32_t closest_point[3];
            double distance_square;
            int32_t update_direction;
        };

        typedef VoxelGrid::VoxelGrid<bucket_cell> DistanceField;

        inline DistanceField BuildDistanceField(const std::vector<VoxelGrid::GRID_INDEX>& points) const
        {
            // Make the DistanceField container
            bucket_cell default_cell;
            default_cell.distance_square = INFINITY;
            DistanceField distance_field(GetOriginTransform(), GetResolution(), GetXSize(), GetYSize(), GetZSize(), default_cell);
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

        inline std::vector<std::vector<std::vector<std::vector<int>>>> MakeNeighborhoods()  const
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

        inline int GetDirectionNumber(const int dx, const int dy, const int dz) const
        {
            return ((dx + 1) * 9) + ((dy + 1) * 3) + (dz + 1);
        }

        inline double ComputeDistanceSquared(const int32_t x1, const int32_t y1, const int32_t z1, const int32_t x2, const int32_t y2, const int32_t z2) const
        {
            int32_t dx = x1 - x2;
            int32_t dy = y1 - y2;
            int32_t dz = z1 - z2;
            return double((dx * dx) + (dy * dy) + (dz * dz));
        }

        bool initialized_;
        std::string frame_;
        VoxelGrid::VoxelGrid<TAGGED_OBJECT_COLLISION_CELL> collision_field_;
        u_int32_t number_of_components_;
        bool components_valid_;

        std::vector<u_int8_t> PackBinaryRepresentation(const std::vector<TAGGED_OBJECT_COLLISION_CELL>& raw) const;

        std::vector<TAGGED_OBJECT_COLLISION_CELL> UnpackBinaryRepresentation(const std::vector<u_int8_t>& packed) const;

        int64_t MarkConnectedComponent(const int64_t x_index, const int64_t y_index, const int64_t z_index, const u_int32_t connected_component);

        std_msgs::ColorRGBA GenerateComponentColor(const u_int32_t component) const;

    public:

        inline TaggedObjectCollisionMapGrid(std::string frame, double resolution, double x_size, double y_size, double z_size, TAGGED_OBJECT_COLLISION_CELL OOB_value) : initialized_(true)
        {
            frame_ = frame;
            VoxelGrid::VoxelGrid<TAGGED_OBJECT_COLLISION_CELL> new_field(resolution, x_size, y_size, z_size, OOB_value);
            collision_field_ = new_field;
            number_of_components_ = 0;
            components_valid_ = false;
        }

        inline TaggedObjectCollisionMapGrid(Eigen::Affine3d origin_transform, std::string frame, double resolution, double x_size, double y_size, double z_size, TAGGED_OBJECT_COLLISION_CELL OOB_value) : initialized_(true)
        {
            frame_ = frame;
            VoxelGrid::VoxelGrid<TAGGED_OBJECT_COLLISION_CELL> new_field(origin_transform, resolution, x_size, y_size, z_size, OOB_value);
            collision_field_ = new_field;
            number_of_components_ = 0;
            components_valid_ = false;
        }

        inline TaggedObjectCollisionMapGrid() : initialized_(false), number_of_components_(0), components_valid_(false) {}

        inline bool IsInitialized() const
        {
            return initialized_;
        }

        inline bool AreComponentsValid() const
        {
            return components_valid_;
        }

        inline std::pair<const TAGGED_OBJECT_COLLISION_CELL&, bool> GetImmutable(const Eigen::Vector3d& location) const
        {
            return collision_field_.GetImmutable(location);
        }

        inline std::pair<const TAGGED_OBJECT_COLLISION_CELL&, bool> GetImmutable(const double x, const double y, const double z) const
        {
            return collision_field_.GetImmutable(x, y, z);
        }

        inline std::pair<const TAGGED_OBJECT_COLLISION_CELL&, bool> GetImmutable(const VoxelGrid::GRID_INDEX& index) const
        {
            return collision_field_.GetImmutable(index);
        }

        inline std::pair<const TAGGED_OBJECT_COLLISION_CELL&, bool> GetImmutable(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            return collision_field_.GetImmutable(x_index, y_index, z_index);
        }

        inline std::pair<TAGGED_OBJECT_COLLISION_CELL&, bool> GetMutable(const Eigen::Vector3d& location)
        {
            return collision_field_.GetMutable(location);
        }

        inline std::pair<TAGGED_OBJECT_COLLISION_CELL&, bool> GetMutable(const double x, const double y, const double z)
        {
            return collision_field_.GetMutable(x, y, z);
        }

        inline std::pair<TAGGED_OBJECT_COLLISION_CELL&, bool> GetMutable(const VoxelGrid::GRID_INDEX& index)
        {
            return collision_field_.GetMutable(index);
        }

        inline std::pair<TAGGED_OBJECT_COLLISION_CELL&, bool> GetMutable(const int64_t x_index, const int64_t y_index, const int64_t z_index)
        {
            return collision_field_.GetMutable(x_index, y_index, z_index);
        }

        inline bool Set(const double x, const double y, const double z, const TAGGED_OBJECT_COLLISION_CELL& value)
        {
            components_valid_ = false;
            return collision_field_.SetValue(x, y, z, value);
        }

        inline bool Set(const Eigen::Vector3d& location, const TAGGED_OBJECT_COLLISION_CELL& value)
        {
            components_valid_ = false;
            return collision_field_.SetValue(location, value);
        }

        inline bool Set(const int64_t x_index, const int64_t y_index, const int64_t z_index, const TAGGED_OBJECT_COLLISION_CELL& value)
        {
            components_valid_ = false;
            return collision_field_.SetValue(x_index, y_index, z_index, value);
        }

        inline bool Set(const VoxelGrid::GRID_INDEX& index, const TAGGED_OBJECT_COLLISION_CELL& value)
        {
            components_valid_ = false;
            return collision_field_.SetValue(index, value);
        }

        inline bool Set(const double x, const double y, const double z, TAGGED_OBJECT_COLLISION_CELL&& value)
        {
            components_valid_ = false;
            return collision_field_.SetValue(x, y, z, value);
        }

        inline bool Set(const Eigen::Vector3d& location, TAGGED_OBJECT_COLLISION_CELL&& value)
        {
            components_valid_ = false;
            return collision_field_.SetValue(location, value);
        }

        inline bool Set(const int64_t x_index, const int64_t y_index, const int64_t z_index, TAGGED_OBJECT_COLLISION_CELL&& value)
        {
            components_valid_ = false;
            return collision_field_.SetValue(x_index, y_index, z_index, value);
        }

        inline bool Set(const VoxelGrid::GRID_INDEX& index, TAGGED_OBJECT_COLLISION_CELL&& value)
        {
            components_valid_ = false;
            return collision_field_.SetValue(index, value);
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
            return collision_field_.GetCellSizes()[0];
        }

        inline TAGGED_OBJECT_COLLISION_CELL GetOOBValue() const
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

        inline std::string GetFrame() const
        {
            return frame_;
        }

        inline std::pair<u_int32_t, bool> GetNumConnectedComponents() const
        {
            return std::pair<u_int32_t, bool>(number_of_components_, components_valid_);
        }

        inline std::vector<int64_t> LocationToGridIndex(const double x, const double y, const double z) const
        {
            return collision_field_.LocationToGridIndex(x, y, z);
        }

        inline std::vector<double> GridIndexToLocation(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            return collision_field_.GridIndexToLocation(x_index, y_index, z_index);
        }

        bool SaveToFile(const std::string& filepath) const;

        bool LoadFromFile(const std::string &filepath);

        sdf_tools::TaggedObjectCollisionMap GetMessageRepresentation() const;

        bool LoadFromMessageRepresentation(const sdf_tools::TaggedObjectCollisionMap& message);

        u_int32_t UpdateConnectedComponents();

        std::map<u_int32_t, std::pair<int32_t, int32_t>> ComputeComponentTopology(const bool ignore_empty_components, const bool recompute_connected_components, const bool verbose);

        std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, u_int8_t>> ExtractComponentSurfaces(const bool ignore_empty_components) const;

        std::pair<int32_t, int32_t> ComputeHolesInSurface(const u_int32_t component, const std::unordered_map<VoxelGrid::GRID_INDEX, u_int8_t>& surface, const bool verbose) const;

        int32_t ComputeConnectivityOfSurfaceVertices(const std::unordered_map<VoxelGrid::GRID_INDEX, u_int8_t>& surface_vertex_connectivity) const;

        inline std::pair<sdf_tools::SignedDistanceField, std::pair<double, double>> ExtractSignedDistanceField(const float oob_value, const std::vector<u_int32_t>& objects_to_use) const
        {
            // To make this faster, we put the objects to use into a map
            std::map<u_int32_t, u_int8_t> object_use_map;
            for (size_t idx = 0; idx < objects_to_use.size(); idx++)
            {
                object_use_map[objects_to_use[idx]] = 1;
            }
            // Make the SDF
            SignedDistanceField new_sdf(GetOriginTransform(), frame_, GetResolution(), GetXSize(), GetYSize(), GetZSize(), oob_value);
            std::vector<VoxelGrid::GRID_INDEX> filled;
            std::vector<VoxelGrid::GRID_INDEX> free;
            for (int64_t x_index = 0; x_index < new_sdf.GetNumXCells(); x_index++)
            {
                for (int64_t y_index = 0; y_index < new_sdf.GetNumYCells(); y_index++)
                {
                    for (int64_t z_index = 0; z_index < new_sdf.GetNumZCells(); z_index++)
                    {
                        VoxelGrid::GRID_INDEX current_index(x_index, y_index, z_index);
                        const TAGGED_OBJECT_COLLISION_CELL& stored = GetImmutable(x_index, y_index, z_index).first;
                        // If it matches an object to use OR there are no objects supplied
                        if ((object_use_map[stored.object_id] == 1) || (objects_to_use.size() == 0))
                        {
                            if (stored.occupancy > 0.5)
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
                        else
                        {
                            // Mark as free space
                            free.push_back(current_index);
                        }
                    }
                }
            }
            // Make two distance fields (one for distance to filled voxels, one for distance to free voxels
            DistanceField filled_distance_field = BuildDistanceField(filled);
            DistanceField free_distance_field = BuildDistanceField(free);
            // Generate the SDF
            double max_distance = -INFINITY;
            double min_distance = INFINITY;
            for (int64_t x_index = 0; x_index < filled_distance_field.GetNumXCells(); x_index++)
            {
                for (int64_t y_index = 0; y_index < filled_distance_field.GetNumYCells(); y_index++)
                {
                    for (int64_t z_index = 0; z_index < filled_distance_field.GetNumZCells(); z_index++)
                    {
                        double distance1 = sqrt(filled_distance_field.GetImmutable(x_index, y_index, z_index).first.distance_square) * new_sdf.GetResolution();
                        double distance2 = sqrt(free_distance_field.GetImmutable(x_index, y_index, z_index).first.distance_square) * new_sdf.GetResolution();
                        double distance = distance1 - distance2;
                        if (distance > max_distance)
                        {
                            max_distance = distance;
                        }
                        if (distance < min_distance)
                        {
                            min_distance = distance;
                        }
                        new_sdf.Set(x_index, y_index, z_index, distance);
                    }
                }
            }
            std::pair<double, double> extrema(max_distance, min_distance);
            return std::pair<SignedDistanceField, std::pair<double, double>>(new_sdf, extrema);
        }

        visualization_msgs::Marker ExportForDisplay() const;

        visualization_msgs::Marker ExportForDisplayOccupancyOnly(const std_msgs::ColorRGBA& collision_color, const std_msgs::ColorRGBA& free_color, const std_msgs::ColorRGBA& unknown_color) const;

        visualization_msgs::Marker ExportConnectedComponentsForDisplay(bool color_unknown_components) const;
    };
}

#endif // TAGGED_OBJECT_COLLISION_MAP_HPP
