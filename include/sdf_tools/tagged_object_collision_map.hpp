#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <functional>
#include <unordered_map>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <random>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <eigen3/unsupported/Eigen/ArpackSupport>
#include <visualization_msgs/Marker.h>
#include <arc_utilities/voxel_grid.hpp>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/simple_kmeans_clustering.hpp>
#include <sdf_tools/sdf.hpp>
#include <sdf_tools/sdf_generation.hpp>
#include <sdf_tools/TaggedObjectCollisionMap.h>
#include <sdf_tools/topology_computation.hpp>

#ifndef TAGGED_OBJECT_COLLISION_MAP_HPP
#define TAGGED_OBJECT_COLLISION_MAP_HPP

#define ENABLE_UNORDERED_MAP_SIZE_HINTS

namespace sdf_tools
{
    struct TAGGED_OBJECT_COLLISION_CELL
    {
        float occupancy;
        uint32_t component;
        uint32_t object_id;
        uint32_t convex_segment;

        TAGGED_OBJECT_COLLISION_CELL() : occupancy(0.0), component(0u), object_id(0u), convex_segment(0u) {}

        TAGGED_OBJECT_COLLISION_CELL(const float in_occupancy, const uint32_t in_object_id) : occupancy(in_occupancy), component(0u), object_id(in_object_id), convex_segment(0u) {}

        TAGGED_OBJECT_COLLISION_CELL(const float in_occupancy, const uint32_t in_object_id, const uint32_t in_component, const uint32_t in_convex_segment) : occupancy(in_occupancy), component(in_component), object_id(in_object_id), convex_segment(in_convex_segment) {}

        bool SharesConvexSegment(const TAGGED_OBJECT_COLLISION_CELL& other) const
        {
            if ((convex_segment & other.convex_segment) > 0)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        std::vector<uint32_t> GetListOfConvexSegments() const
        {
            uint32_t temp_convex_segment = convex_segment;
            std::vector<uint32_t> convex_segments;
            for (uint32_t segment = 1; segment <= 32; segment++)
            {
                if ((temp_convex_segment & 0x00000001) == 1)
                {
                    convex_segments.push_back(segment);
                }
                temp_convex_segment = temp_convex_segment >> 1;
            }
            return convex_segments;
        }

        bool IsPartOfConvexSegment(const uint32_t segment) const
        {
            assert(segment >= 1);
            assert(segment <= 32);
            const uint32_t mask = arc_helpers::SetBit(0u, segment - 1, true);
            if ((mask & convex_segment) > 0)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        void AddToConvexSegment(const uint32_t segment)
        {
            assert(segment >= 1);
            assert(segment <= 32);
            convex_segment = arc_helpers::SetBit(convex_segment, segment - 1, true);
        }

        void RemoveFromConvexSegment(const uint32_t segment)
        {
            assert(segment >= 1);
            assert(segment <= 32);
            convex_segment = arc_helpers::SetBit(convex_segment, segment - 1, false);
        }
    };

    inline std::vector<uint8_t> TaggedObjectCollisionCellToBinary(const TAGGED_OBJECT_COLLISION_CELL& value)
    {
        std::vector<uint8_t> binary(sizeof(TAGGED_OBJECT_COLLISION_CELL));
        memcpy(&binary.front(), &value, sizeof(TAGGED_OBJECT_COLLISION_CELL));
        return binary;
    }

    inline TAGGED_OBJECT_COLLISION_CELL TaggedObjectCollisionCellFromBinary(const std::vector<uint8_t>& binary)
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

        inline static std_msgs::ColorRGBA GenerateComponentColor(const uint32_t component, const float alpha=1.0f)
        {
            return arc_helpers::GenerateUniqueColor<std_msgs::ColorRGBA>(component, alpha);
        }

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
            uint32_t our_component = GetImmutable(x_index, y_index, z_index).first.component;
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

        VoxelGrid::VoxelGrid<TAGGED_OBJECT_COLLISION_CELL> collision_field_;
        uint32_t number_of_components_;
        std::string frame_;
        bool initialized_;
        bool components_valid_;
        bool convex_segments_valid_;

        std::vector<uint8_t> PackBinaryRepresentation(const std::vector<TAGGED_OBJECT_COLLISION_CELL>& raw) const;

        std::vector<TAGGED_OBJECT_COLLISION_CELL> UnpackBinaryRepresentation(const std::vector<uint8_t>& packed) const;

        std::vector<VoxelGrid::GRID_INDEX> CheckIfConvex(const VoxelGrid::GRID_INDEX& candidate_index, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>& explored_indices, const VoxelGrid::VoxelGrid<std::vector<uint32_t>>& region_grid, const uint32_t current_convex_region) const;

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        inline TaggedObjectCollisionMapGrid(const std::string& frame, const double resolution, const double x_size, const double y_size, const double z_size, const TAGGED_OBJECT_COLLISION_CELL& default_value, const TAGGED_OBJECT_COLLISION_CELL& OOB_value) : initialized_(true)
        {
            frame_ = frame;
            VoxelGrid::VoxelGrid<TAGGED_OBJECT_COLLISION_CELL> new_field(resolution, x_size, y_size, z_size, default_value, OOB_value);
            collision_field_ = new_field;
            number_of_components_ = 0;
            components_valid_ = false;
            convex_segments_valid_ = false;
        }

        inline TaggedObjectCollisionMapGrid(const Eigen::Isometry3d& origin_transform, const std::string& frame, const double resolution, const double x_size, const double y_size, const double z_size, const TAGGED_OBJECT_COLLISION_CELL& default_value, const TAGGED_OBJECT_COLLISION_CELL& OOB_value) : initialized_(true)
        {
            frame_ = frame;
            VoxelGrid::VoxelGrid<TAGGED_OBJECT_COLLISION_CELL> new_field(origin_transform, resolution, x_size, y_size, z_size, default_value, OOB_value);
            collision_field_ = new_field;
            number_of_components_ = 0;
            components_valid_ = false;
            convex_segments_valid_ = false;
        }

        inline TaggedObjectCollisionMapGrid(const std::string& frame, const double resolution, const double x_size, const double y_size, const double z_size, const TAGGED_OBJECT_COLLISION_CELL& OOB_value) : initialized_(true)
        {
            frame_ = frame;
            VoxelGrid::VoxelGrid<TAGGED_OBJECT_COLLISION_CELL> new_field(resolution, x_size, y_size, z_size, OOB_value);
            collision_field_ = new_field;
            number_of_components_ = 0;
            components_valid_ = false;
            convex_segments_valid_ = false;
        }

        inline TaggedObjectCollisionMapGrid(const Eigen::Isometry3d& origin_transform, const std::string& frame, const double resolution, const double x_size, const double y_size, const double z_size, const TAGGED_OBJECT_COLLISION_CELL& OOB_value) : initialized_(true)
        {
            frame_ = frame;
            VoxelGrid::VoxelGrid<TAGGED_OBJECT_COLLISION_CELL> new_field(origin_transform, resolution, x_size, y_size, z_size, OOB_value);
            collision_field_ = new_field;
            number_of_components_ = 0;
            components_valid_ = false;
            convex_segments_valid_ = false;
        }

        inline TaggedObjectCollisionMapGrid() : number_of_components_(0), initialized_(false), components_valid_(false), convex_segments_valid_(false) {}

        inline bool IsInitialized() const
        {
            return initialized_;
        }

        inline bool AreComponentsValid() const
        {
            return components_valid_;
        }

        inline bool AreConvexSegmentsValid() const
        {
            return convex_segments_valid_;
        }

        inline std::pair<bool, bool> CheckIfCandidateCorner3d(const Eigen::Vector3d& location) const
        {
            const VoxelGrid::GRID_INDEX index = collision_field_.LocationToGridIndex3d(location);
            if (collision_field_.IndexInBounds(index))
            {
                return CheckIfCandidateCorner(index);
            }
            else
            {
                return std::pair<bool, bool>(false, false);
            }
        }

        inline std::pair<bool, bool> CheckIfCandidateCorner4d(const Eigen::Vector4d& location) const
        {
            const VoxelGrid::GRID_INDEX index = collision_field_.LocationToGridIndex4d(location);
            if (collision_field_.IndexInBounds(index))
            {
                return CheckIfCandidateCorner(index);
            }
            else
            {
                return std::pair<bool, bool>(false, false);
            }
        }

        inline std::pair<bool, bool> CheckIfCandidateCorner(const double x, const double y, const double z) const
        {
            const Eigen::Vector4d location(x, y, z, 1.0);
            return CheckIfCandidateCorner4d(location);
        }

        inline std::pair<bool, bool> CheckIfCandidateCorner(const VoxelGrid::GRID_INDEX& index) const
        {
            return CheckIfCandidateCorner(index.x, index.y, index.z);
        }

        inline std::pair<bool, bool> CheckIfCandidateCorner(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            assert(components_valid_);
            assert(convex_segments_valid_);
            const std::pair<const TAGGED_OBJECT_COLLISION_CELL&, bool> current_cell = GetImmutable(x_index, y_index, z_index);
            if (current_cell.second)
            {
                // Grab the six neighbors ONLY if they belong to a different component
                uint32_t different_neighbors = 0u;
                const std::pair<const TAGGED_OBJECT_COLLISION_CELL&, bool> xm1yz_cell = GetImmutable(x_index - 1, y_index, z_index);
                if (xm1yz_cell.second && (xm1yz_cell.first.component != current_cell.first.component))
                {
                    different_neighbors++;
                }
                const std::pair<const TAGGED_OBJECT_COLLISION_CELL&, bool> xp1yz_cell = GetImmutable(x_index + 1, y_index, z_index);
                if (xp1yz_cell.second && (xp1yz_cell.first.component != current_cell.first.component))
                {
                    different_neighbors++;
                }
                const std::pair<const TAGGED_OBJECT_COLLISION_CELL&, bool> xym1z_cell = GetImmutable(x_index, y_index - 1, z_index);
                if (xym1z_cell.second && (xym1z_cell.first.component != current_cell.first.component))
                {
                    different_neighbors++;
                }
                const std::pair<const TAGGED_OBJECT_COLLISION_CELL&, bool> xyp1z_cell = GetImmutable(x_index, y_index + 1, z_index);
                if (xyp1z_cell.second && (xyp1z_cell.first.component != current_cell.first.component))
                {
                    different_neighbors++;
                }
                const std::pair<const TAGGED_OBJECT_COLLISION_CELL&, bool> xyzm1_cell = GetImmutable(x_index, y_index, z_index - 1);
                if (xyzm1_cell.second && (xyzm1_cell.first.component != current_cell.first.component))
                {
                    different_neighbors++;
                }
                const std::pair<const TAGGED_OBJECT_COLLISION_CELL&, bool> xyzp1_cell = GetImmutable(x_index, y_index, z_index + 1);
                if (xyzp1_cell.second && (xyzp1_cell.first.component != current_cell.first.component))
                {
                    different_neighbors++;
                }
                // We now have between zero and six neighbors to work with
                if (different_neighbors <= 1u)
                {
                    // If there is one or fewer neighbors to work with, we are clearly not a corner
                    return std::pair<bool, bool>(false, true);
                }
                else
                {
                    // If there are 2 or more neighbors to work with, we are a candidate corner
                    return std::pair<bool, bool>(true, true);
                }
            }
            else
            {
                // Not in the grid
                return std::pair<bool, bool>(false, false);
            }
        }

        inline std::pair<const TAGGED_OBJECT_COLLISION_CELL&, bool> GetImmutable3d(const Eigen::Vector3d& location) const
        {
            return collision_field_.GetImmutable3d(location);
        }

        inline std::pair<const TAGGED_OBJECT_COLLISION_CELL&, bool> GetImmutable4d(const Eigen::Vector4d& location) const
        {
            return collision_field_.GetImmutable4d(location);
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

        inline std::pair<TAGGED_OBJECT_COLLISION_CELL&, bool> GetMutable3d(const Eigen::Vector3d& location)
        {
            return collision_field_.GetMutable3d(location);
        }

        inline std::pair<TAGGED_OBJECT_COLLISION_CELL&, bool> GetMutable4d(const Eigen::Vector4d& location)
        {
            return collision_field_.GetMutable4d(location);
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
            convex_segments_valid_ = false;
            return collision_field_.SetValue(x, y, z, value);
        }

        inline bool Set3d(const Eigen::Vector3d& location, const TAGGED_OBJECT_COLLISION_CELL& value)
        {
            components_valid_ = false;
            convex_segments_valid_ = false;
            return collision_field_.SetValue3d(location, value);
        }

        inline bool Set4d(const Eigen::Vector4d& location, const TAGGED_OBJECT_COLLISION_CELL& value)
        {
            components_valid_ = false;
            convex_segments_valid_ = false;
            return collision_field_.SetValue4d(location, value);
        }

        inline bool Set(const int64_t x_index, const int64_t y_index, const int64_t z_index, const TAGGED_OBJECT_COLLISION_CELL& value)
        {
            components_valid_ = false;
            convex_segments_valid_ = false;
            return collision_field_.SetValue(x_index, y_index, z_index, value);
        }

        inline bool Set(const VoxelGrid::GRID_INDEX& index, const TAGGED_OBJECT_COLLISION_CELL& value)
        {
            components_valid_ = false;
            convex_segments_valid_ = false;
            return collision_field_.SetValue(index, value);
        }

        inline bool Set(const double x, const double y, const double z, TAGGED_OBJECT_COLLISION_CELL&& value)
        {
            components_valid_ = false;
            convex_segments_valid_ = false;
            return collision_field_.SetValue(x, y, z, value);
        }

        inline bool Set3d(const Eigen::Vector3d& location, TAGGED_OBJECT_COLLISION_CELL&& value)
        {
            components_valid_ = false;
            convex_segments_valid_ = false;
            return collision_field_.SetValue3d(location, value);
        }

        inline bool Set4d(const Eigen::Vector4d& location, TAGGED_OBJECT_COLLISION_CELL&& value)
        {
            components_valid_ = false;
            convex_segments_valid_ = false;
            return collision_field_.SetValue4d(location, value);
        }

        inline bool Set(const int64_t x_index, const int64_t y_index, const int64_t z_index, TAGGED_OBJECT_COLLISION_CELL&& value)
        {
            components_valid_ = false;
            convex_segments_valid_ = false;
            return collision_field_.SetValue(x_index, y_index, z_index, value);
        }

        inline bool Set(const VoxelGrid::GRID_INDEX& index, TAGGED_OBJECT_COLLISION_CELL&& value)
        {
            components_valid_ = false;
            convex_segments_valid_ = false;
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
            return collision_field_.GetCellSizes().x();
        }

        inline TAGGED_OBJECT_COLLISION_CELL GetDefaultValue() const
        {
            return collision_field_.GetDefaultValue();
        }

        inline TAGGED_OBJECT_COLLISION_CELL GetOOBValue() const
        {
            return collision_field_.GetOOBValue();
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

        inline const Eigen::Isometry3d& GetOriginTransform() const
        {
            return collision_field_.GetOriginTransform();
        }

        inline const Eigen::Isometry3d& GetInverseOriginTransform() const
        {
            return collision_field_.GetInverseOriginTransform();
        }

        inline std::string GetFrame() const
        {
            return frame_;
        }

        inline std::pair<uint32_t, bool> GetNumConnectedComponents() const
        {
            return std::pair<uint32_t, bool>(number_of_components_, components_valid_);
        }

        inline VoxelGrid::GRID_INDEX LocationToGridIndex3d(const Eigen::Vector3d& location) const
        {
            return collision_field_.LocationToGridIndex3d(location);
        }

        inline VoxelGrid::GRID_INDEX LocationToGridIndex4d(const Eigen::Vector4d& location) const
        {
            return collision_field_.LocationToGridIndex4d(location);
        }

        inline VoxelGrid::GRID_INDEX LocationToGridIndex(double x, double y, double z) const
        {
            return collision_field_.LocationToGridIndex(x, y, z);
        }

        inline Eigen::Vector4d GridIndexToLocation(const VoxelGrid::GRID_INDEX& index) const
        {
            return collision_field_.GridIndexToLocation(index);
        }

        inline Eigen::Vector4d GridIndexToLocation(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            return collision_field_.GridIndexToLocation(x_index, y_index, z_index);
        }

        bool SaveToFile(const std::string& filepath) const;

        bool LoadFromFile(const std::string &filepath);

        sdf_tools::TaggedObjectCollisionMap GetMessageRepresentation() const;

        bool LoadFromMessageRepresentation(const sdf_tools::TaggedObjectCollisionMap& message);

        TaggedObjectCollisionMapGrid Resample(const double new_resolution) const
        {
            TaggedObjectCollisionMapGrid resampled(GetOriginTransform(), GetFrame(), new_resolution, GetXSize(), GetYSize(), GetZSize(), GetOOBValue());
            for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
            {
                for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
                {
                    for (int64_t z_index = 0; z_index < GetNumZCells(); z_index++)
                    {
                        const TAGGED_OBJECT_COLLISION_CELL& current_cell = GetImmutable(x_index, y_index, z_index).first;
                        const Eigen::Vector4d current_cell_location = GridIndexToLocation(x_index, y_index, z_index);
                        resampled.Set4d(current_cell_location, current_cell);
                    }
                }
            }
            return resampled;
        }

        uint32_t UpdateConnectedComponents()
        {
            // If the connected components are already valid, skip computing them again
            if (components_valid_)
            {
                return number_of_components_;
            }
            components_valid_ = false;
            // Make the helper functions
            const std::function<bool(const VoxelGrid::GRID_INDEX&, const VoxelGrid::GRID_INDEX&)> are_connected_fn
                = [&] (const VoxelGrid::GRID_INDEX& index1, const VoxelGrid::GRID_INDEX& index2)
            {
                auto query1 = collision_field_.GetImmutable(index1);
                auto query2 = collision_field_.GetImmutable(index2);
                assert(query1.second);
                assert(query2.second);
                if ((query1.first.occupancy > 0.5) == (query2.first.occupancy > 0.5))
                {
                    return true;
                }
                else
                {
                    return false;
                }
            };
            const std::function<int64_t(const VoxelGrid::GRID_INDEX&)> get_component_fn = [&] (const VoxelGrid::GRID_INDEX& index)
            {
                auto query = collision_field_.GetImmutable(index);
                if (query.second)
                {
                    return (int64_t)query.first.component;
                }
                else
                {
                    return (int64_t)-1;
                }
            };
            const std::function<void(const VoxelGrid::GRID_INDEX&, const uint32_t)> mark_component_fn = [&] (const VoxelGrid::GRID_INDEX& index, const uint32_t component)
            {
                auto query = collision_field_.GetMutable(index);
                if (query.second)
                {
                    collision_field_.SetValue(index, TAGGED_OBJECT_COLLISION_CELL(query.first.occupancy,
                                                                                  query.first.object_id,
                                                                                  component,
                                                                                  query.first.convex_segment));
                }
            };
            number_of_components_ = topology_computation::ComputeConnectedComponents(collision_field_, are_connected_fn, get_component_fn, mark_component_fn);
            components_valid_ = true;
            return number_of_components_;
        }

        enum COMPONENT_TYPES : uint8_t { FILLED_COMPONENTS=0x01, EMPTY_COMPONENTS=0x02, UNKNOWN_COMPONENTS=0x04 };

        std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>> ExtractComponentSurfaces(const COMPONENT_TYPES component_types_to_extract) const
        {
            // Make the helper functions
            const std::function<int64_t(const VoxelGrid::GRID_INDEX&)> get_component_fn = [&] (const VoxelGrid::GRID_INDEX& index)
            {
                auto query = collision_field_.GetImmutable(index);
                if (query.second)
                {
                    return (int64_t)query.first.component;
                }
                else
                {
                    return (int64_t)-1;
                }
            };
            const std::function<bool(const VoxelGrid::GRID_INDEX&)> is_surface_index_fn = [&] (const VoxelGrid::GRID_INDEX& index)
            {
                const TAGGED_OBJECT_COLLISION_CELL& current_cell = GetImmutable(index).first;
                if (current_cell.occupancy > 0.5)
                {
                    if ((component_types_to_extract & FILLED_COMPONENTS) > 0x00)
                    {
                        if (IsSurfaceIndex(index.x, index.y, index.y))
                        {
                            return true;
                        }
                    }
                }
                else if (current_cell.occupancy < 0.5)
                {
                    if ((component_types_to_extract & EMPTY_COMPONENTS) > 0x00)
                    {
                        if (IsSurfaceIndex(index.x, index.y, index.z))
                        {
                            return true;
                        }
                    }
                }
                else
                {
                    if ((component_types_to_extract & UNKNOWN_COMPONENTS) > 0x00)
                    {
                        if (IsSurfaceIndex(index.x, index.z, index.z))
                        {
                            return true;
                        }
                    }
                }
                return false;
            };
            return topology_computation::ExtractComponentSurfaces(collision_field_, get_component_fn, is_surface_index_fn);
        }

        std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>> ExtractFilledComponentSurfaces() const
        {
            return ExtractComponentSurfaces(FILLED_COMPONENTS);
        }

        std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>> ExtractUnknownComponentSurfaces() const
        {
            return ExtractComponentSurfaces(UNKNOWN_COMPONENTS);
        }

        std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>> ExtractEmptyComponentSurfaces() const
        {
            return ExtractComponentSurfaces(EMPTY_COMPONENTS);
        }

        std::map<uint32_t, std::pair<int32_t, int32_t>> ComputeComponentTopology(const COMPONENT_TYPES component_types_to_use, const bool recompute_connected_components, const bool verbose)
        {
            // Recompute the connected components if need be
            if (recompute_connected_components)
            {
                UpdateConnectedComponents();
            }
            // Make the helper functions
            const std::function<int64_t(const VoxelGrid::GRID_INDEX&)> get_component_fn = [&] (const VoxelGrid::GRID_INDEX& index)
            {
                auto query = collision_field_.GetImmutable(index);
                if (query.second)
                {
                    return (int64_t)query.first.component;
                }
                else
                {
                    return (int64_t)-1;
                }
            };
            const std::function<bool(const VoxelGrid::GRID_INDEX&)> is_surface_index_fn = [&] (const VoxelGrid::GRID_INDEX& index)
            {
                const TAGGED_OBJECT_COLLISION_CELL& current_cell = GetImmutable(index).first;
                if (current_cell.occupancy > 0.5)
                {
                    if ((component_types_to_use & FILLED_COMPONENTS) > 0x00)
                    {
                        if (IsSurfaceIndex(index.x, index.y, index.y))
                        {
                            return true;
                        }
                    }
                }
                else if (current_cell.occupancy < 0.5)
                {
                    if ((component_types_to_use & EMPTY_COMPONENTS) > 0x00)
                    {
                        if (IsSurfaceIndex(index.x, index.y, index.z))
                        {
                            return true;
                        }
                    }
                }
                else
                {
                    if ((component_types_to_use & UNKNOWN_COMPONENTS) > 0x00)
                    {
                        if (IsSurfaceIndex(index.x, index.z, index.z))
                        {
                            return true;
                        }
                    }
                }
                return false;
            };
            return topology_computation::ComputeComponentTopology(collision_field_, get_component_fn, is_surface_index_fn, verbose);
        }

        /* Extracts the active indices from a surface map as a vector, which is useful in contexts where a 1-dimensional index into the surface is needed
         */
        std::vector<VoxelGrid::GRID_INDEX> ExtractStaticSurface(const std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>& raw_surface) const
        {
            std::vector<VoxelGrid::GRID_INDEX> static_surface;
            // This may be larger than the actual surface we'll extrac
            static_surface.reserve(raw_surface.size());
            for (auto itr = raw_surface.begin(); itr != raw_surface.end(); ++itr)
            {
                const VoxelGrid::GRID_INDEX& index = itr->first;
                const uint8_t value = itr->second;
                if (value == 1)
                {
                    static_surface.push_back(index);
                }
            }
            // Try to reclaim the unnecessary vector capacity
            static_surface.shrink_to_fit();
            return static_surface;
        }

        std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t> ConvertToDynamicSurface(const std::vector<VoxelGrid::GRID_INDEX>& static_surface) const
        {
            std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t> dynamic_surface(static_surface.size());
            for (size_t idx = 0; idx < static_surface.size(); idx++)
            {
                const VoxelGrid::GRID_INDEX& grid_index = static_surface[idx];
                dynamic_surface[grid_index] = 1u;
            }
            return dynamic_surface;
        }

        std::unordered_map<VoxelGrid::GRID_INDEX, size_t> BuildSurfaceIndexMap(const std::vector<VoxelGrid::GRID_INDEX>& static_surface) const
        {
            std::unordered_map<VoxelGrid::GRID_INDEX, size_t> dynamic_surface(static_surface.size());
            for (size_t idx = 0; idx < static_surface.size(); idx++)
            {
                const VoxelGrid::GRID_INDEX& current_index = static_surface[idx];
                dynamic_surface[current_index] = idx;
            }
            return dynamic_surface;
        }

        std::pair<sdf_tools::SignedDistanceField, std::pair<double, double>> ExtractSignedDistanceField(const float oob_value, const std::vector<uint32_t>& objects_to_use) const
        {
            // To make this faster, we put the objects to use into a map
            std::map<uint32_t, uint8_t> object_use_map;
            for (size_t idx = 0; idx < objects_to_use.size(); idx++)
            {
                object_use_map[objects_to_use[idx]] = 1;
            }
            // Make the helper function
            const std::function<bool(const TAGGED_OBJECT_COLLISION_CELL& cell)> is_filled_fn = [&] (const TAGGED_OBJECT_COLLISION_CELL& stored)
            {
                // If it matches an object to use OR there are no objects supplied
                if ((object_use_map[stored.object_id] == 1) || (objects_to_use.size() == 0))
                {
                    if (stored.occupancy > 0.5)
                    {
                        // Mark as filled
                        return true;
                    }
                }
                return false;
            };
            return sdf_generation::ExtractSignedDistanceField(collision_field_, is_filled_fn, oob_value, GetFrame());
        }

        VoxelGrid::VoxelGrid<std::vector<uint32_t>> ComputeConvexRegions(const double max_check_radius) const;

        void GrowConvexRegion(const VoxelGrid::GRID_INDEX& start_index, VoxelGrid::VoxelGrid<std::vector<uint32_t>>& region_grid, const double max_check_radius, const uint32_t current_convex_region) const;

        EigenHelpers::VectorVector3d GenerateRayPrimitiveVectors(const uint32_t number_of_rays, const double cone_angle) const
        {
            (void)(number_of_rays);
            (void)(cone_angle);
            // Uniformly between [0.0, 2PI)
            std::vector<double> phis = {0.0, M_PI_4, M_PI_2, M_PI_4 * 3.0, M_PI, M_PI_4 * 5.0, M_PI_2 * 3.0, M_PI_4 * 7.0};
            // Uniformly between [cos(cone angle), 1.0]
            std::vector<double> zs = {0.55, 0.65, 0.75, 0.85, 0.95};
            // Sample the rays inside the cone
            EigenHelpers::VectorVector3d ray_primitive_vectors;
            for (size_t phidx = 0; phidx < phis.size(); phidx++)
            {
                for (size_t zdx = 0; zdx < zs.size(); zdx++)
                {
                    const double phi = phis[phidx];
                    const double z = zs[zdx];
                    // Compute the vector
                    const double x = sqrt(1.0 - (z * z)) * cos(phi);
                    const double y = sqrt(1.0 - (z * z)) * sin(phi);
                    Eigen::Vector3d raw_cone_vector(x, y, z);
                    ray_primitive_vectors.push_back(raw_cone_vector / raw_cone_vector.norm());
                }
            }
            ray_primitive_vectors.push_back(Eigen::Vector3d::UnitZ());
            return ray_primitive_vectors;
        }

        std::pair<std::vector<size_t>, std::vector<size_t>> CastSingleRay(const std::unordered_map<VoxelGrid::GRID_INDEX, size_t>& surface_index_map, const VoxelGrid::GRID_INDEX& current_surface_index, const Eigen::Vector3d& ray_unit_vector) const
        {
            std::map<size_t, uint8_t> line_of_sight_indices;
            std::map<size_t, uint8_t> non_line_of_sight_indices;
            const Eigen::Vector4d start_location = GridIndexToLocation(current_surface_index.x, current_surface_index.y, current_surface_index.z);
            Eigen::Vector3d current_location(start_location(0), start_location(1), start_location(2));
            const double ray_step_size = GetResolution() * 0.5;
            const Eigen::Vector3d ray_step_vector = ray_unit_vector * ray_step_size;
            bool in_grid = true;
            bool collided = false;
            // Step along the ray vector until we run off the side of the grid
            while (in_grid)
            {
                // Grab the index corresponding to our location
                const VoxelGrid::GRID_INDEX current_index = LocationToGridIndex(current_location.x(), current_location.y(), current_location.z());
                if (collision_field_.IndexInBounds(current_index))
                {
                    //std::cout << "CVGX " << PrettyPrint::PrettyPrint(current_index) << std::endl;
                    // Are we in the surface?
                    auto found_itr = surface_index_map.find(current_index);
                    if (found_itr != surface_index_map.end())
                    {
                        //std::cout << "+++++ On surface +++++" << std::endl;
                        const VoxelGrid::GRID_INDEX& found_index = found_itr->first;
                        const size_t found_surface_index = found_itr->second;
                        // If we are not the current surface index
                        if (!(found_index == current_surface_index))
                        {
                            if ((line_of_sight_indices[found_surface_index] == 1) || (collided == false))
                            {
                                line_of_sight_indices[found_surface_index] = 1u;
                                collided = true;
                            }
                            else
                            {
                                non_line_of_sight_indices[found_surface_index] = 1u;
                            }
                        }
                    }
                    // Either way, we take a step
                    current_location += ray_step_vector;
                }
                // We're not in the grid any more
                else
                {
                    in_grid = false;
                }
            }
            //std::cout << "LOS [map] " << PrettyPrint::PrettyPrint(line_of_sight_indices) << std::endl;
            //std::cout << "NLOS [map] " << PrettyPrint::PrettyPrint(non_line_of_sight_indices) << std::endl;
            // Get the vectors of indices
            std::vector<size_t> line_of_sight_indices_vector;
            line_of_sight_indices_vector.reserve(line_of_sight_indices.size());
            for (auto itr = line_of_sight_indices.begin(); itr != line_of_sight_indices.end(); ++itr)
            {
                if (itr->second == 1u)
                {
                    line_of_sight_indices_vector.push_back(itr->first);
                }
            }
            std::vector<size_t> non_line_of_sight_indices_vector;
            non_line_of_sight_indices_vector.reserve(non_line_of_sight_indices.size());
            for (auto itr = non_line_of_sight_indices.begin(); itr != non_line_of_sight_indices.end(); ++itr)
            {
                if (itr->second == 1u)
                {
                    non_line_of_sight_indices_vector.push_back(itr->first);
                }
            }
            // Shrink to fit as needed
            line_of_sight_indices_vector.shrink_to_fit();
            non_line_of_sight_indices_vector.shrink_to_fit();
            //std::cout << "LOS [vec] " << PrettyPrint::PrettyPrint(line_of_sight_indices_vector) << std::endl;
            //std::cout << "NLOS [vec] " << PrettyPrint::PrettyPrint(non_line_of_sight_indices_vector) << std::endl;
            return std::pair<std::vector<size_t>, std::vector<size_t>>(line_of_sight_indices_vector, non_line_of_sight_indices_vector);
        }

        std::pair<std::vector<size_t>, std::vector<size_t>> PerformRayCasting(const sdf_tools::SignedDistanceField& sdf, const std::unordered_map<VoxelGrid::GRID_INDEX, size_t>& surface_index_map, const VoxelGrid::GRID_INDEX& current_surface_index, const EigenHelpers::VectorVector3d& ray_primitive_vectors) const
        {
            std::vector<size_t> line_of_sight_indices;
            std::vector<size_t> non_line_of_sight_indices;
            // Are we inside an object?
            bool inside_object = false;
            const float occupancy = GetImmutable(current_surface_index).first.occupancy;
            if (occupancy < 0.5f)
            {
                inside_object = false;
            }
            else if (occupancy > 0.5f)
            {
                inside_object = true;
            }
            else
            {
                // LOL NOPE
                assert(occupancy != 0.5f);
            }
            // Get the current gradient
            std::vector<double> raw_gradient = sdf.GetGradient(current_surface_index.x, current_surface_index.y, current_surface_index.z, true);
            Eigen::Vector3d raw_gradient_vector = EigenHelpers::StdVectorDoubleToEigenVector3d(raw_gradient);
            // Turn the gradient into an inverse surface normal vector (i.e. points inwards)
            assert(raw_gradient_vector.norm() > 0.0);
            // In free space, the vector already points in the correct direction
            Eigen::Vector3d surface_normal = raw_gradient_vector / raw_gradient_vector.norm();
            // If we're inside an object, flip the vector
            if (inside_object)
            {
                surface_normal = surface_normal * -1.0;
            }
            // Compute the pointing rotation using the cross product
            Eigen::Vector3d base_normalized_vector = Eigen::Vector3d::UnitZ();
            Eigen::Vector3d cross_product = base_normalized_vector.cross(surface_normal);
            double dot_product = base_normalized_vector.dot(surface_normal);
            double angle = acos(dot_product);
            // Make the rotation
            Eigen::AngleAxisd surface_normal_rotation(angle, cross_product);
            Eigen::Matrix3d surface_normal_rotation_matrix(surface_normal_rotation);
            // Safety check
            Eigen::Vector3d check_vector = surface_normal_rotation_matrix * base_normalized_vector;
            check_vector = check_vector / check_vector.norm();
            //std::cout << "Surface normal " << PrettyPrint::PrettyPrint(surface_normal) << std::endl;
            //std::cout << "Check vector " << PrettyPrint::PrettyPrint(check_vector) << std::endl;
            //assert(EigenHelpers::CloseEnough(check_vector, surface_normal, 0.01));
            // Perform raycasting for each ray
            for (size_t idx = 0; idx < ray_primitive_vectors.size(); idx++)
            {
                // Get the real ray unit vector
                const Eigen::Vector3d& ray_primitive_vector = ray_primitive_vectors[idx];
                const Eigen::Vector3d ray_unit_vector = surface_normal_rotation_matrix * ray_primitive_vector;
                //std::cout << "Ray vector " << PrettyPrint::PrettyPrint(ray_unit_vector) << std::endl;
                // Cast the single ray
                std::pair<std::vector<size_t>, std::vector<size_t>> single_raycast_results = CastSingleRay(surface_index_map, current_surface_index, ray_unit_vector);
                //std::cout << "Raycasting results - " << PrettyPrint::PrettyPrint(single_raycast_results) << std::endl;
                // Store the results
                line_of_sight_indices.insert(line_of_sight_indices.end(), single_raycast_results.first.begin(), single_raycast_results.first.end());
                non_line_of_sight_indices.insert(non_line_of_sight_indices.end(), single_raycast_results.second.begin(), single_raycast_results.second.end());
            }
            return std::pair<std::vector<size_t>, std::vector<size_t>>(line_of_sight_indices, non_line_of_sight_indices);
        }

        //std::pair<Eigen::MatrixXd, std::pair<Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>>> ComputeSparseLineOfSight(const std::vector<VoxelGrid::GRID_INDEX>& static_surface, const uint32_t number_of_rays, const double cone_angle) const
        Eigen::MatrixXd ComputeSparseLineOfSight(const std::vector<VoxelGrid::GRID_INDEX>& static_surface, const uint32_t number_of_rays, const double cone_angle) const
        {
            // Make our signed distance field
            sdf_tools::SignedDistanceField sdf = ExtractSignedDistanceField(INFINITY, std::vector<uint32_t>()).first;
            // Make a dynamic surface map
            std::unordered_map<VoxelGrid::GRID_INDEX, size_t> dynamic_surface_map = BuildSurfaceIndexMap(static_surface);
            // Make the ray primitive vectors
            EigenHelpers::VectorVector3d ray_primitive_vectors = GenerateRayPrimitiveVectors(number_of_rays, cone_angle);
            // Make containers
//            std::vector<Eigen::Triplet<double>> line_of_sight;
//            std::vector<Eigen::Triplet<double>> line_of_sight_not;
            Eigen::MatrixXd line_of_sight_matrix = Eigen::MatrixXd::Zero(static_surface.size(), static_surface.size());
            // Go through the surface
            for (size_t idx = 0; idx < static_surface.size(); idx++)
            {
                const VoxelGrid::GRID_INDEX& current_surface_index = static_surface[idx];
                // Perform raycasting
                // Returns a vector of line-of-sight surface indices, and a vector of non-line-of-sight surface indices
                const std::pair<std::vector<size_t>, std::vector<size_t>> raycast_results = PerformRayCasting(sdf, dynamic_surface_map, current_surface_index, ray_primitive_vectors);
                // Update the matrices
                for (size_t sdx = 0; sdx < raycast_results.first.size(); sdx++)
                {
                    const size_t surface_index = raycast_results.first[sdx];
//                    line_of_sight.push_back(Eigen::Triplet<double>(idx, surface_index, 1.0));
//                    line_of_sight.push_back(Eigen::Triplet<double>(surface_index, idx, 1.0));
                    line_of_sight_matrix(idx, surface_index) = 1.0;
                    line_of_sight_matrix(surface_index, idx) = 1.0;
                }
                for (size_t sdx = 0; sdx < raycast_results.second.size(); sdx++)
                {
                    const size_t surface_index = raycast_results.second[sdx];
//                    line_of_sight_not.push_back(Eigen::Triplet<double>(idx, surface_index, 1.0));
//                    line_of_sight_not.push_back(Eigen::Triplet<double>(surface_index, idx, 1.0));
                    line_of_sight_matrix(idx, surface_index) = 0.0;
                    line_of_sight_matrix(surface_index, idx) = 0.0;
                }
                // Add ourselves to the line of sight
//                line_of_sight.push_back(Eigen::Triplet<double>(idx, idx, 1.0));
                line_of_sight_matrix(idx, idx) = 1.0;
            }
//            // Put together the sparse matrices
//            Eigen::SparseMatrix<double> line_of_sight_sparse;
//            line_of_sight_sparse.setFromTriplets(line_of_sight.begin(), line_of_sight.end());
//            Eigen::SparseMatrix<double> line_of_sight_not_sparse;
//            line_of_sight_not_sparse.setFromTriplets(line_of_sight_not.begin(), line_of_sight_not.end());
            // Return them all
            //return std::pair<Eigen::MatrixXd, std::pair<Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>>>(line_of_sight_matrix, std::pair<Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>>(line_of_sight_sparse, line_of_sight_not_sparse));
            return line_of_sight_matrix;
        }

        std::pair<Eigen::VectorXd, Eigen::MatrixXd> ExtractKLargestEigenvaluesAndEigenvectors(const Eigen::EigenSolver<Eigen::MatrixXd>::EigenvalueType& raw_eigenvalues, const Eigen::EigenSolver<Eigen::MatrixXd>::EigenvectorsType& raw_eigenvectors, const uint32_t num_values) const
        {
            assert((int64_t)num_values <= raw_eigenvalues.size());
            // Collect the eigenvalue + index pairs
            std::vector<std::pair<double, size_t>> eigenvalue_index_pairs(raw_eigenvalues.size());
            for (size_t idx = 0; idx < eigenvalue_index_pairs.size(); idx++)
            {
                const double current_eigenvalue = raw_eigenvalues((long)idx).real();
                eigenvalue_index_pairs[idx].first = current_eigenvalue;
                eigenvalue_index_pairs[idx].second = idx;
            }
            // Sort the eigenvalue/index pairs by eigenvalue
            std::function<bool(const std::pair<double, size_t>&, const std::pair<double, size_t>&)> compare_fn = [] (const std::pair<double, size_t>& p1, const std::pair<double, size_t>& p2) { if (p1.first < p2.first) { return true; } else { return false; } };
            // Sorts them in ascending order
            std::sort(eigenvalue_index_pairs.begin(), eigenvalue_index_pairs.end(), compare_fn);
            // Now, we extract the last num_values eigenvalues and eigenvectors and put them into a vector + matrix
            // Each column of this matrix is an eigenvector, so the # of rows corresponds to the number of datapoints, and the # of columns is the number of clusters
            Eigen::VectorXd k_largest_eigenvalues = Eigen::VectorXd::Zero(num_values);
            Eigen::MatrixXd k_largest_eigenvectors = Eigen::MatrixXd::Zero(raw_eigenvalues.rows(), num_values);
            for (uint32_t num_value = 0; num_value < num_values; num_value++)
            {
                // Get the index of the ascending-sorted vector
                const int64_t eigenvalue_index = (eigenvalue_index_pairs.size() - 1) - num_value;
                // Get the index corresponding to the num_valueth-largest eigenvalue
                const double current_eigenvalue = eigenvalue_index_pairs[eigenvalue_index].first;
                k_largest_eigenvalues(num_value) = current_eigenvalue;
                const size_t eigenvector_index = eigenvalue_index_pairs[eigenvalue_index].second;
                // Grab the corresponding eigenvector
                const Eigen::VectorXcd current_eigenvector = raw_eigenvectors.col((int64_t)eigenvector_index);
                // Copy it over into the real world
                Eigen::VectorXd real_current_eigenvector = Eigen::VectorXd::Zero(current_eigenvector.size());
                for (int64_t vdx = 0; vdx < real_current_eigenvector.size(); vdx++)
                {
                    real_current_eigenvector(vdx) = current_eigenvector(vdx).real();
                }
                // Set it in the matrix
                k_largest_eigenvectors.col(num_value) = real_current_eigenvector;
            }
            // Return
            return std::pair<Eigen::VectorXd, Eigen::MatrixXd>(k_largest_eigenvalues, k_largest_eigenvectors);
        }

        std::vector<uint32_t> PerformKMeansSpectralClustering(const Eigen::EigenSolver<Eigen::MatrixXd>::EigenvalueType& raw_eigenvalues, const Eigen::EigenSolver<Eigen::MatrixXd>::EigenvectorsType& raw_eigenvectors, const uint32_t num_clusters) const
        {
            assert(num_clusters > 0);
            // Grab the num_clusters largest eigenvalues & corresponding eigenvectors
            // Each column of this matrix is an eigenvector, so the # of rows corresponds to the number of datapoints, and the # of columns is the number of clusters
            const Eigen::MatrixXd k_largest_eigenvectors = ExtractKLargestEigenvaluesAndEigenvectors(raw_eigenvalues, raw_eigenvectors, num_clusters).second;
            // Convert them into datapoints for kmeans clustering
            std::vector<Eigen::VectorXd> clustering_data(k_largest_eigenvectors.rows());
            for (size_t datapoint_index = 0; datapoint_index < clustering_data.size(); datapoint_index++)
            {
                Eigen::VectorXd datapoint = Eigen::VectorXd::Zero(num_clusters);
                for (int64_t datavalue_index = 0; datavalue_index < (int64_t)num_clusters; datavalue_index++)
                {
                    const double raw_value = k_largest_eigenvectors(datapoint_index, datavalue_index);
                    datapoint(datavalue_index) = raw_value;
                }
                clustering_data[datapoint_index] = datapoint;
            }
            std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)> distance_fn = [] (const Eigen::VectorXd& v1, const Eigen::VectorXd& v2) { return EigenHelpers::Distance(v1, v2); };
            std::function<Eigen::VectorXd(const std::vector<Eigen::VectorXd>&)> average_fn = [] (const std::vector<Eigen::VectorXd>& data) { return EigenHelpers::AverageEigenVectorXd(data); };
            std::vector<uint32_t> cluster_labels = simple_kmeans_clustering::SimpleKMeansClustering::Cluster(clustering_data, distance_fn, average_fn, num_clusters, true);
            return cluster_labels;
        }

        double ComputeConvexityMetric(const Eigen::MatrixXd& los_matrix, const std::vector<uint32_t>& cluster_labels) const
        {
            const double alpha = 1.0; // This was used in the paper
            assert(los_matrix.rows() == los_matrix.cols());
            assert(cluster_labels.size() == (size_t)los_matrix.rows());
            const uint32_t num_clusters = *std::max_element(cluster_labels.begin(), cluster_labels.end()) + 1;
            double total_convexity_metric = 0.0;
            for (uint32_t cluster = 0; cluster < num_clusters; cluster++)
            {
                double intravisible = 0.0;
                double interoccluded = 0.0;
                for (size_t idx = 0; idx < cluster_labels.size(); idx++)
                {
                    // We only care about elements in our own cluster
                    if (cluster_labels[idx] == cluster)
                    {
                        // Loop through our row in the LOS matrix
                        for (size_t other_idx = 0; other_idx < cluster_labels.size(); other_idx++)
                        {
                            // If the other element is part of our cluster AND is visible
                            if ((cluster_labels[other_idx] == cluster) && (los_matrix(idx, other_idx) == 1.0))
                            {
                                intravisible += 1.0; // This double counts, but we need to double count for convexity = 1 in the single cluster case
                            }
                            // If the other element is not part of our cluster AND is not visible
                            else if ((cluster_labels[other_idx] != cluster) && (los_matrix(idx, other_idx) == 0.0))
                            {
                                interoccluded += 1.0;
                            }
                        }
                    }
                }
                const double cluster_convexity_metric = intravisible + (alpha * interoccluded);
                total_convexity_metric += cluster_convexity_metric;
            }
            total_convexity_metric = total_convexity_metric / (double)(cluster_labels.size() * cluster_labels.size());
            return total_convexity_metric;
        }

        std::vector<std::vector<size_t>> ClusterSurfaceFromLOSMatrix(const Eigen::MatrixXd& los_matrix, const uint32_t max_num_clusters) const
        {
            assert(los_matrix.rows() == los_matrix.cols());
            assert(max_num_clusters > 0);
            // Perform spectral clustering
            // Compute degree matrix
            Eigen::MatrixXd degree_matrix_diagonals = los_matrix.rowwise().sum();
            Eigen::MatrixXd degree_matrix = Eigen::MatrixXd::Zero(los_matrix.rows(), los_matrix.cols());
            degree_matrix.diagonal() = degree_matrix_diagonals;
            // Compute the unnormalized Laplacian
            Eigen::MatrixXd unnormalized_laplacian = degree_matrix - los_matrix;
            // Compute Eigenvalues & Eigenvectors
            Eigen::EigenSolver<Eigen::MatrixXd> solver(unnormalized_laplacian);
            Eigen::EigenSolver<Eigen::MatrixXd>::EigenvalueType eigen_values = solver.eigenvalues();
            Eigen::EigenSolver<Eigen::MatrixXd>::EigenvectorsType eigen_vectors = solver.eigenvectors();
            // Perform k-means clustering over a range of # of clusters
            double best_convexity = 0.0;
            std::vector<uint32_t> best_clustering;
            for (uint32_t num_clusters = 1; num_clusters <= max_num_clusters; num_clusters++)
            {
                if (num_clusters > (uint32_t)los_matrix.rows())
                {
                    std::cerr << "Number of clusters is larger than elements in surface, stopping clustering process" << std::endl;
                    break;
                }
                std::vector<uint32_t> cluster_labels = PerformKMeansSpectralClustering(eigen_values, eigen_vectors, num_clusters);
                const double convexity = ComputeConvexityMetric(los_matrix, cluster_labels);
                std::cout << "K-means clustering at " << num_clusters << " clusters with convexity " << convexity << std::endl;
                if (convexity > best_convexity)
                {
                    best_convexity = convexity;
                    best_clustering = cluster_labels;
                }
            }
            // Safety check
            assert(best_clustering.size() == (size_t)los_matrix.rows());
            // Turn the clustering labels into separate clusters
            const uint32_t num_clusters = *std::max_element(best_clustering.begin(), best_clustering.end()) + 1;
            std::vector<std::vector<size_t>> clustered_surfaces(num_clusters);
            for (size_t idx = 0; idx < best_clustering.size(); idx++)
            {
                const uint32_t cluster_label = best_clustering[idx];
                clustered_surfaces[cluster_label].push_back(idx);
            }
            return clustered_surfaces;
        }

        std::vector<std::vector<VoxelGrid::GRID_INDEX>> ComputeWeaklyConvexSurfaceSegments(const std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>& surface, const uint32_t max_num_clusters) const
        {
            std::vector<VoxelGrid::GRID_INDEX> static_surface = ExtractStaticSurface(surface);
            Eigen::MatrixXd LOS_matrix = ComputeSparseLineOfSight(static_surface, 30u, (M_PI_2 * 0.5)); // For now, these values are actually ignored
            std::cout << "LOS matrix:\n" << LOS_matrix << std::endl;
            std::vector<std::vector<size_t>> convex_surface_segment_indices = ClusterSurfaceFromLOSMatrix(LOS_matrix, max_num_clusters);
            // Convert the 0-n indices into grid indices
            std::vector<std::vector<VoxelGrid::GRID_INDEX>> convex_surface_segments(convex_surface_segment_indices.size());
            for (size_t segment_idx = 0; segment_idx < convex_surface_segment_indices.size(); segment_idx++)
            {
                const std::vector<size_t>& current_segment_indices = convex_surface_segment_indices[segment_idx];
                convex_surface_segments[segment_idx].reserve(current_segment_indices.size());
                for (size_t index_idx = 0; index_idx < current_segment_indices.size(); index_idx++)
                {
                    const size_t current_segment_index = current_segment_indices[index_idx];
                    const VoxelGrid::GRID_INDEX& current_segment_surface_index = static_surface[current_segment_index];
                    convex_surface_segments[segment_idx].push_back(current_segment_surface_index);
                }
                assert(convex_surface_segments[segment_idx].size() == current_segment_indices.size());
            }
            assert(convex_surface_segment_indices.size() == convex_surface_segments.size());
            return convex_surface_segments;
        }

        std::map<uint32_t, uint32_t> UpdateConvexSegments(const bool enable_experimental_features=false)
        {
            // First, we need connected components to be accurate
            UpdateConnectedComponents();
            // Some day, we will do real work here. Until then, this is a dummy that does nothing
            if (enable_experimental_features)
            {
                const std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>> free_space_surfaces = ExtractEmptyComponentSurfaces();
                for (auto itr = free_space_surfaces.begin(); itr != free_space_surfaces.end(); ++itr)
                {
                    const uint32_t component_id = itr->first;
                    const std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>& component_surface = itr->second;
                    // Try to segemnt into convex segments
                    const std::vector<std::vector<VoxelGrid::GRID_INDEX>> weakly_convex_surface_segments = ComputeWeaklyConvexSurfaceSegments(component_surface, 32u);
                    std::cout << "Segmented free space component " << component_id << " surface into " << weakly_convex_surface_segments.size() << " weakly convex segments" << std::endl;
                    // Not yet sure how to best go from segmented surface to segmented volume
                }
            }
            convex_segments_valid_ = true;
            // Return a map of object_id to # of convex segments in the object
            std::map<uint32_t, uint32_t> convex_segment_counts;
            for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
            {
                for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
                {
                    for (int64_t z_index = 0; z_index < GetNumZCells(); z_index++)
                    {
                        const TAGGED_OBJECT_COLLISION_CELL& cell = GetImmutable(x_index, y_index, z_index).first;
                        const uint32_t cell_object_id = cell.object_id;
                        const std::vector<uint32_t> cell_convex_segments = cell.GetListOfConvexSegments();
                        if (cell_convex_segments.size() > 0)
                        {
                            const uint32_t max_segment_number = *std::max_element(cell_convex_segments.begin(), cell_convex_segments.end());
                            if (max_segment_number > convex_segment_counts[cell_object_id])
                            {
                                convex_segment_counts[cell_object_id] = max_segment_number;
                            }
                        }
                    }
                }
            }
            return convex_segment_counts;
        }

        std::map<uint32_t, sdf_tools::SignedDistanceField> MakeObjectSDFs(const std::vector<uint32_t>& object_ids) const
        {
            std::map<uint32_t, sdf_tools::SignedDistanceField> per_object_sdfs;
            for (size_t idx = 0; idx < object_ids.size(); idx++)
            {
                const uint32_t object_id = object_ids[idx];
                per_object_sdfs[object_id] = ExtractSignedDistanceField(std::numeric_limits<double>::infinity(), std::vector<uint32_t>{object_id}).first;
            }
            return per_object_sdfs;
        }

        std::map<uint32_t, sdf_tools::SignedDistanceField> MakeObjectSDFs() const
        {
            std::map<uint32_t, uint32_t> object_id_map;
            for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
            {
                for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
                {
                    for (int64_t z_index = 0; z_index < GetNumZCells(); z_index++)
                    {
                        const TAGGED_OBJECT_COLLISION_CELL& cell = GetImmutable(x_index, y_index, z_index).first;
                        const uint32_t cell_object_id = cell.object_id;
                        if (cell_object_id > 0)
                        {
                            object_id_map[cell_object_id] = 1u;
                        }
                    }
                }
            }
            return MakeObjectSDFs(arc_helpers::GetKeys(object_id_map));
        }

        visualization_msgs::Marker ExportForDisplay(const float alpha, const std::vector<uint32_t>& objects_to_draw=std::vector<uint32_t>()) const;

        visualization_msgs::Marker ExportForDisplay(const std::map<uint32_t, std_msgs::ColorRGBA>& object_color_map=std::map<uint32_t, std_msgs::ColorRGBA>()) const;

        visualization_msgs::Marker ExportContourOnlyForDisplay(const float alpha, const std::vector<uint32_t>& objects_to_draw=std::vector<uint32_t>()) const;

        visualization_msgs::Marker ExportContourOnlyForDisplay(const std::map<uint32_t, std_msgs::ColorRGBA>& object_color_map=std::map<uint32_t, std_msgs::ColorRGBA>()) const;

        visualization_msgs::Marker ExportForDisplayOccupancyOnly(const std_msgs::ColorRGBA& collision_color, const std_msgs::ColorRGBA& free_color, const std_msgs::ColorRGBA& unknown_color) const;

        visualization_msgs::Marker ExportConnectedComponentsForDisplay(bool color_unknown_components) const;

        visualization_msgs::Marker ExportConvexSegmentForDisplay(const uint32_t object_id, const uint32_t convex_segment) const;

        visualization_msgs::Marker ExportSurfaceForDisplay(const std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>& surface, const std_msgs::ColorRGBA& surface_color) const;
    };
}

#endif // TAGGED_OBJECT_COLLISION_MAP_HPP
