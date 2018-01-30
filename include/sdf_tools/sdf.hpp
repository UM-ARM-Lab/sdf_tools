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
#include <sdf_tools/SDF.h>

#ifndef SDF_HPP
#define SDF_HPP

inline std::vector<uint8_t> FloatToBinary(float value)
{
    uint32_t binary_value = 0;
    memcpy(&binary_value, &value, sizeof(uint32_t));
    std::vector<uint8_t> binary(4);
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

inline float FloatFromBinary(std::vector<uint8_t>& binary)
{
    if (binary.size() != 4)
    {
        std::cerr << "Binary value is not 4 bytes" << std::endl;
        return NAN;
    }
    else
    {
        uint32_t binary_value = 0;
        // Copy in byte 4, most-significant byte
        binary_value = binary_value | binary[0];
        binary_value = binary_value << 8;
        // Copy in byte 3
        binary_value = binary_value | binary[1];
        binary_value = binary_value << 8;
        // Copy in byte 2
        binary_value = binary_value | binary[2];
        binary_value = binary_value << 8;
        // Copy in byte 1, least-significant byte
        binary_value = binary_value | binary[3];
        // Convert binary to float and store
        float field_value = 0.0;
        memcpy(&field_value, &binary_value, sizeof(float));
        return field_value;
    }
}

namespace sdf_tools
{
    class SignedDistanceField
    {
    protected:

        VoxelGrid::VoxelGrid<float> distance_field_;
        std::string frame_;
        bool initialized_;
        bool locked_;

        std::vector<uint8_t> GetInternalBinaryRepresentation(const std::vector<float>& field_data) const;

        std::vector<float> UnpackFieldFromBinaryRepresentation(const std::vector<uint8_t>& binary) const;

        /*
         * You *MUST* provide valid indices to this function, hence why it is protected (there are safe wrappers available - use them!)
         */
        void FollowGradientsToLocalMaximaUnsafe(VoxelGrid::VoxelGrid<Eigen::Vector3d>& watershed_map, const int64_t x_index, const int64_t y_index, const int64_t z_index) const;

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        inline SignedDistanceField(std::string frame, double resolution, double x_size, double y_size, double z_size, float OOB_value) : initialized_(true), locked_(false)
        {
            frame_ = frame;
            VoxelGrid::VoxelGrid<float> new_field(resolution, x_size, y_size, z_size, OOB_value);
            distance_field_ = new_field;
        }

        inline SignedDistanceField(Eigen::Isometry3d origin_transform, std::string frame, double resolution, double x_size, double y_size, double z_size, float OOB_value) : initialized_(true), locked_(false)
        {
            frame_ = frame;
            VoxelGrid::VoxelGrid<float> new_field(origin_transform, resolution, x_size, y_size, z_size, OOB_value);
            distance_field_ = new_field;
        }

        inline SignedDistanceField() : initialized_(false), locked_(false) {}

        inline bool IsInitialized() const
        {
            return initialized_;
        }

        inline bool IsLocked() const
        {
            return locked_;
        }

        inline void Lock()
        {
            locked_ = true;
        }

        inline void Unlock()
        {
            locked_ = false;
        }

        inline float Get(const double x, const double y, const double z) const
        {
            return distance_field_.GetImmutable(x, y, z).first;
        }

        inline float Get3d(const Eigen::Vector3d& location) const
        {
            return distance_field_.GetImmutable3d(location).first;
        }

        inline float Get4d(const Eigen::Vector4d& location) const
        {
            return distance_field_.GetImmutable4d(location).first;
        }

        inline float Get(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            return distance_field_.GetImmutable(x_index, y_index, z_index).first;
        }

        inline std::pair<float, bool> GetSafe(const double x, const double y, const double z) const
        {
            return distance_field_.GetImmutable(x, y, z);
        }

        inline std::pair<float, bool> GetSafe3d(const Eigen::Vector3d& location) const
        {
            return distance_field_.GetImmutable3d(location);
        }

        inline std::pair<float, bool> GetSafe4d(const Eigen::Vector4d& location) const
        {
            return distance_field_.GetImmutable4d(location);
        }

        inline std::pair<float, bool> GetSafe(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            return distance_field_.GetImmutable(x_index, y_index, z_index);
        }

        /*
         * Setter functions MUST be used carefully - If you arbitrarily change SDF values, it is not a proper SDF any more!
         *
         * Use of these functions can be prevented by calling SignedDistanceField::Lock() on the SDF, at which point these functions
         * will fail with a warning printed to std_err.
         */
        inline bool Set(const double x, const double y, const double z, float value)
        {
            if (!locked_)
            {
                return distance_field_.SetValue(x, y, z, value);
            }
            else
            {
                std::cerr << "Attempt to set value in locked SDF" << std::endl;
                return false;
            }
        }

        inline bool Set3d(const Eigen::Vector3d& location, float value)
        {
            if (!locked_)
            {
                return distance_field_.SetValue3d(location, value);
            }
            else
            {
                std::cerr << "Attempt to set value in locked SDF" << std::endl;
                return false;
            }
        }

        inline bool Set4d(const Eigen::Vector4d& location, float value)
        {
            if (!locked_)
            {
                return distance_field_.SetValue4d(location, value);
            }
            else
            {
                std::cerr << "Attempt to set value in locked SDF" << std::endl;
                return false;
            }
        }

        inline bool Set(const int64_t x_index, const int64_t y_index, const int64_t z_index, const float value)
        {
            if (!locked_)
            {
                return distance_field_.SetValue(x_index, y_index, z_index, value);
            }
            else
            {
                std::cerr << "Attempt to set value in locked SDF" << std::endl;
                return false;
            }
        }

        inline bool Set(const VoxelGrid::GRID_INDEX& index, const float value)
        {
            if (!locked_)
            {
                return distance_field_.SetValue(index, value);
            }
            else
            {
                std::cerr << "Attempt to set value in locked SDF" << std::endl;
                return false;
            }
        }

        inline bool CheckInBounds3d(const Eigen::Vector3d& location) const
        {
            return distance_field_.LocationInBounds3d(location);
        }

        inline bool CheckInBounds4d(const Eigen::Vector4d& location) const
        {
            return distance_field_.LocationInBounds4d(location);
        }

        inline bool CheckInBounds(const double x, const double y, const double z) const
        {
            return distance_field_.LocationInBounds(x, y, z);
        }

        inline bool CheckInBounds(const VoxelGrid::GRID_INDEX& index) const
        {
            return distance_field_.IndexInBounds(index);
        }

        inline bool CheckInBounds(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            return distance_field_.IndexInBounds(x_index, y_index, z_index);
        }

        inline double GetXSize() const
        {
            return distance_field_.GetXSize();
        }

        inline double GetYSize() const
        {
            return distance_field_.GetYSize();
        }

        inline double GetZSize() const
        {
            return distance_field_.GetZSize();
        }

        inline double GetResolution() const
        {
            return distance_field_.GetCellSizes().x();
        }

        inline float GetOOBValue() const
        {
            return distance_field_.GetDefaultValue();
        }

        inline int64_t GetNumXCells() const
        {
            return distance_field_.GetNumXCells();
        }

        inline int64_t GetNumYCells() const
        {
            return distance_field_.GetNumYCells();
        }

        inline int64_t GetNumZCells() const
        {
            return distance_field_.GetNumZCells();
        }

    protected:

        inline double GetCorrectedCenter(const int64_t x_idx, const int64_t y_idx, const int64_t z_idx) const
        {
            if (distance_field_.IndexInBounds(x_idx, y_idx, z_idx))
            {
                const double nominal_sdf_distance = Get(x_idx, y_idx, z_idx);
                const double cell_center_distance_offset = GetResolution() * 0.5;
                const double center_adjusted_nominal_distance = (nominal_sdf_distance >= 0.0) ? nominal_sdf_distance - cell_center_distance_offset : nominal_sdf_distance + cell_center_distance_offset;
                return center_adjusted_nominal_distance;
            }
            else
            {
                throw std::invalid_argument("Index out of bounds");
            }
        }

        inline double EstimateDistanceNonBoundaryCell(const Eigen::Vector4d& query_location, const int64_t x_idx, const int64_t y_idx, const int64_t z_idx, const double nominal_sdf_distance) const
        {
            const Eigen::Vector4d cell_center_location = GridIndexToLocation(x_idx, y_idx, z_idx);
            const Eigen::Vector4d cell_center_to_query_vector = query_location - cell_center_location;
            const std::vector<double> raw_gradient = GetGradient(x_idx, y_idx, z_idx, true);
            assert(raw_gradient.size() == 3);
            const Eigen::Vector4d gradient(raw_gradient[0], raw_gradient[1], raw_gradient[2], 0.0);
            // Adjust for calculating distance to boundary of voxels instead of center of voxels
            const double gradient_norm = gradient.norm();
            // This adjustment is nominally resolution / 2, however, we can inflate this slightly with the gradient
            // to handle diagonal graidents where the distance is longer.
            // Think of it as
            // (change in distance / unit change in position) * change in position = change in distance
            const double cell_center_distance_offset = gradient_norm * GetResolution() * 0.5;
            const double center_adjusted_nominal_distance = (nominal_sdf_distance >= 0.0) ? nominal_sdf_distance - cell_center_distance_offset : nominal_sdf_distance + cell_center_distance_offset;
            // We figure out how much of the center->location vector is along the gradient vector
            // Naively this would be VectorProjection(gradient, center->location).norm() with +/- determined
            // separately using the dot product. In pure math, this would be:
            // ((center->location.dot(gradient) / gradient.norm()) * (gradient / gradient.norm())).norm();
            // We only want the magnitude of the projected vector, i.e.
            // center->location.dot(gradient) / gradient.norm()
            // which has the correct sign so we don't need to separately determine sign
            const double distance_adjustment = cell_center_to_query_vector.dot(gradient) / gradient_norm;
            const double adjusted_distance = center_adjusted_nominal_distance + distance_adjustment;
            assert((adjusted_distance >= 0.0) == (nominal_sdf_distance >= 0.0));
            return adjusted_distance;
        }

        inline double EstimateDistanceBoundaryCell(const Eigen::Vector4d& query_location, const int64_t x_idx, const int64_t y_idx, const int64_t z_idx, const double nominal_sdf_distance) const
        {
            // Grab the closest neighboring cell that is across the boundary
            double closest_across_boundary_squared_distance = std::numeric_limits<double>::infinity();
            VoxelGrid::GRID_INDEX closest_across_boundary_index(-1, -1, -1);
            Eigen::Vector4d closest_across_boundary_location(0.0, 0.0, 0.0, 1.0);
            for (int64_t test_x_idx = x_idx - 1; test_x_idx <= x_idx + 1; test_x_idx++)
            {
                for (int64_t test_y_idx = y_idx - 1; test_y_idx <= y_idx + 1; test_y_idx++)
                {
                    for (int64_t test_z_idx = z_idx - 1; test_z_idx <= z_idx + 1; test_z_idx++)
                    {
                        const auto query = GetSafe(test_x_idx, test_y_idx, test_z_idx);
                        if (query.second)
                        {
                            const double test_nominal_distance = (double)query.first;
                            if ((test_nominal_distance >= 0.0) != (nominal_sdf_distance >= 0.0))
                            {
                                const Eigen::Vector4d test_cell_location = GridIndexToLocation(test_x_idx, test_y_idx, test_z_idx);
                                const double squared_distance = (query_location - test_cell_location).squaredNorm();
                                if (squared_distance < closest_across_boundary_squared_distance)
                                {
                                    closest_across_boundary_squared_distance = squared_distance;
                                    closest_across_boundary_index = VoxelGrid::GRID_INDEX(test_x_idx, test_y_idx, test_z_idx);
                                    closest_across_boundary_location = test_cell_location;
                                }
                            }
                        }
                    }
                }
            }
            assert(distance_field_.IndexInBounds(closest_across_boundary_index));
            const Eigen::Vector4d cell_center_location = GridIndexToLocation(x_idx, y_idx, z_idx);
            // We can assume that the true boundary lies eactly in the middle of the current cell center and the across-boundary cell center
            // We model this with a hyperplane of the form (point, normal)
            // The point is the midpoint between cell centers
            const Eigen::Vector4d boundary_plane_point = EigenHelpers::Interpolate4d(cell_center_location, closest_across_boundary_location, 0.5);
            // Helpfully, the normal is just plane_point->grid_aligned_cell_center
            const Eigen::Vector4d boundary_plane_normal = cell_center_location - boundary_plane_point;
            // Now we just compute the distance from our query point to the plane
            // Naively this would be VectorProjection(normal, point->query).norm()
            // In pure math, this would be:
            // ((point->query.dot(normal) / normal.norm()) * (normal / normal.norm())).norm();
            // We only want the magnitude of the projected vector, i.e.
            // point->query.dot(normal) / normal.norm()
            const Eigen::Vector4d point_to_query_vector = query_location - boundary_plane_point;
            const double abs_distance = point_to_query_vector.dot(boundary_plane_normal) / boundary_plane_normal.norm();
            // Set sign appropriately
            if (nominal_sdf_distance >= 0.0)
            {
                return abs_distance;
            }
            else
            {
                return -abs_distance;
            }
        }

        inline double LinearInterpolateX(const Eigen::Vector4d& corner_location, const Eigen::Vector4d& query_location,
                                         const double mx_dist, const double px_dist) const
        {
            const double interpolation_ratio = (query_location(0) - corner_location(0)) / GetResolution();
            return EigenHelpers::Interpolate(mx_dist, px_dist, interpolation_ratio);
        }

        inline double LinearInterpolateY(const Eigen::Vector4d& corner_location, const Eigen::Vector4d& query_location,
                                         const double my_dist, const double py_dist) const
        {
            const double interpolation_ratio = (query_location(1) - corner_location(1)) / GetResolution();
            return EigenHelpers::Interpolate(my_dist, py_dist, interpolation_ratio);
        }

        inline double LinearInterpolateZ(const Eigen::Vector4d& corner_location, const Eigen::Vector4d& query_location,
                                         const double mz_dist, const double pz_dist) const
        {
            const double interpolation_ratio = (query_location(2) - corner_location(2)) / GetResolution();
            return EigenHelpers::Interpolate(mz_dist, pz_dist, interpolation_ratio);
        }

        inline double BilinearInterpolate(const double low_d1, const double high_d1, const double low_d2, const double high_d2, const double query_d1, const double query_d2, const double l1l2_val, const double l1h2_val, const double h1l2_val, const double h1h2_val) const
        {
            Eigen::Matrix<double, 1, 2> d1_offsets;
            d1_offsets(0, 0) = high_d1 - query_d1;
            d1_offsets(0, 1) = query_d1 - low_d1;
            Eigen::Matrix<double, 2, 2> values;
            values(0, 0) = l1l2_val;
            values(0, 1) = l1h2_val;
            values(1, 0) = h1l2_val;
            values(1, 1) = h1h2_val;
            Eigen::Matrix<double, 2, 1> d2_offsets;
            d2_offsets(0, 0) = high_d2 - query_d2;
            d2_offsets(1, 0) = query_d2 - low_d2;
            const double multiplier = 1.0 / ((high_d1 - low_d1) * (high_d2 - low_d2));
            const double bilinear_interpolated = multiplier * d1_offsets * values * d2_offsets;
            return bilinear_interpolated;
        }

        inline double BilinearInterpolateDistanceXY(const Eigen::Vector4d& corner_location, const Eigen::Vector4d& query_location,
                                                    const double mxmy_dist, const double mxpy_dist, const double pxmy_dist, const double pxpy_dist) const
        {
            return BilinearInterpolate(corner_location(0), corner_location(0) + GetResolution(), corner_location(1), corner_location(1) + GetResolution(), query_location(0), query_location(1), mxmy_dist, mxpy_dist, pxmy_dist, pxpy_dist);
        }

        inline double BilinearInterpolateDistanceYZ(const Eigen::Vector2d& corner_location, const Eigen::Vector2d& query_location,
                                                    const double mymz_dist, const double mypz_dist, const double pymz_dist, const double pypz_dist) const
        {
            return BilinearInterpolate(corner_location(1), corner_location(1) + GetResolution(), corner_location(2), corner_location(2) + GetResolution(), query_location(1), query_location(2), mymz_dist, mypz_dist, pymz_dist, pypz_dist);
        }

        inline double BilinearInterpolateDistanceXZ(const Eigen::Vector2d& corner_location, const Eigen::Vector2d& query_location,
                                                    const double mxmz_dist, const double mxpz_dist, const double pxmz_dist, const double pxpz_dist) const
        {
            return BilinearInterpolate(corner_location(0), corner_location(0) + GetResolution(), corner_location(2), corner_location(2) + GetResolution(), query_location(0), query_location(2), mxmz_dist, mxpz_dist, pxmz_dist, pxpz_dist);
        }

        inline double TrilinearInterpolateDistance(const Eigen::Vector4d& corner_location, const Eigen::Vector4d& query_location,
                                                   const double mxmymz_dist, const double mxmypz_dist, const double mxpymz_dist, const double mxpypz_dist,
                                                   const double pxmymz_dist, const double pxmypz_dist, const double pxpymz_dist, const double pxpypz_dist) const
        {
            const double mz_bilinear_interpolated = BilinearInterpolateDistanceXY(corner_location, query_location,
                                                                                  mxmymz_dist, mxpymz_dist, pxmymz_dist, pxpymz_dist);
            const double pz_bilinear_interpolated = BilinearInterpolateDistanceXY(corner_location, query_location,
                                                                                  mxmypz_dist, mxpypz_dist, pxmypz_dist, pxpypz_dist);
            return LinearInterpolateZ(corner_location, query_location, mz_bilinear_interpolated, pz_bilinear_interpolated);
        }

        inline double EstimateDistanceInterpolateFromNeighbors(const Eigen::Vector4d& query_location, const int64_t x_idx, const int64_t y_idx, const int64_t z_idx) const
        {
            try
            {
                // Switch between all the possible options of where we are
                const Eigen::Vector4d cell_center_location = GridIndexToLocation(x_idx, y_idx, z_idx);
                const Eigen::Vector4d query_offset = query_location - cell_center_location;
                // Catch the easiest case
                if ((query_offset(0) == 0.0) && (query_offset(1) == 0.0) && (query_offset(2) == 0.0))
                {
                    return GetCorrectedCenter(x_idx, y_idx, z_idx);
                }
                // +X+Y+Z
                else if ((query_offset(0) >= 0.0) && (query_offset(1) >= 0.0) && (query_offset(2) >= 0.0))
                {
                    return TrilinearInterpolateDistance(cell_center_location, query_location,
                                                        GetCorrectedCenter(x_idx, y_idx, z_idx),
                                                        GetCorrectedCenter(x_idx, y_idx, z_idx + 1),
                                                        GetCorrectedCenter(x_idx, y_idx + 1, z_idx),
                                                        GetCorrectedCenter(x_idx, y_idx + 1, z_idx + 1),
                                                        GetCorrectedCenter(x_idx + 1, y_idx, z_idx),
                                                        GetCorrectedCenter(x_idx + 1, y_idx, z_idx + 1),
                                                        GetCorrectedCenter(x_idx + 1, y_idx + 1, z_idx),
                                                        GetCorrectedCenter(x_idx + 1, y_idx + 1, z_idx + 1));
                }
                // +X+Y-Z
                else if ((query_offset(0) >= 0.0) && (query_offset(1) >= 0.0) && (query_offset(2) < 0.0))
                {
                    const Eigen::Vector4d corner_location = GridIndexToLocation(x_idx, y_idx, z_idx - 1);
                    return TrilinearInterpolateDistance(corner_location, query_location,
                                                        GetCorrectedCenter(x_idx, y_idx, z_idx -1),
                                                        GetCorrectedCenter(x_idx, y_idx, z_idx),
                                                        GetCorrectedCenter(x_idx, y_idx + 1, z_idx - 1),
                                                        GetCorrectedCenter(x_idx, y_idx + 1, z_idx),
                                                        GetCorrectedCenter(x_idx + 1, y_idx, z_idx - 1),
                                                        GetCorrectedCenter(x_idx + 1, y_idx, z_idx),
                                                        GetCorrectedCenter(x_idx + 1, y_idx + 1, z_idx - 1),
                                                        GetCorrectedCenter(x_idx + 1, y_idx + 1, z_idx));
                }
                // +X-Y+Z
                else if ((query_offset(0) >= 0.0) && (query_offset(1) < 0.0) && (query_offset(2) >= 0.0))
                {
                    const Eigen::Vector4d corner_location = GridIndexToLocation(x_idx, y_idx - 1, z_idx);

                    return TrilinearInterpolateDistance(corner_location, query_location,
                                                        GetCorrectedCenter(x_idx, y_idx - 1, z_idx),
                                                        GetCorrectedCenter(x_idx, y_idx - 1, z_idx + 1),
                                                        GetCorrectedCenter(x_idx, y_idx, z_idx),
                                                        GetCorrectedCenter(x_idx, y_idx, z_idx + 1),
                                                        GetCorrectedCenter(x_idx + 1, y_idx - 1, z_idx),
                                                        GetCorrectedCenter(x_idx + 1, y_idx - 1, z_idx + 1),
                                                        GetCorrectedCenter(x_idx + 1, y_idx, z_idx),
                                                        GetCorrectedCenter(x_idx + 1, y_idx, z_idx + 1));
                }
                // +X-Y-Z
                else if ((query_offset(0) >= 0.0) && (query_offset(1) < 0.0) && (query_offset(2) < 0.0))
                {
                    const Eigen::Vector4d corner_location = GridIndexToLocation(x_idx, y_idx - 1, z_idx - 1);

                    return TrilinearInterpolateDistance(corner_location, query_location,
                                                        GetCorrectedCenter(x_idx, y_idx - 1, z_idx -1),
                                                        GetCorrectedCenter(x_idx, y_idx - 1, z_idx),
                                                        GetCorrectedCenter(x_idx, y_idx, z_idx - 1),
                                                        GetCorrectedCenter(x_idx, y_idx, z_idx),
                                                        GetCorrectedCenter(x_idx + 1, y_idx - 1, z_idx - 1),
                                                        GetCorrectedCenter(x_idx + 1, y_idx - 1, z_idx),
                                                        GetCorrectedCenter(x_idx + 1, y_idx, z_idx - 1),
                                                        GetCorrectedCenter(x_idx + 1, y_idx, z_idx));
                }
                // -X+Y+Z
                else if ((query_offset(0) < 0.0) && (query_offset(1) >= 0.0) && (query_offset(2) >= 0.0))
                {
                    const Eigen::Vector4d corner_location = GridIndexToLocation(x_idx - 1, y_idx, z_idx);
                    return TrilinearInterpolateDistance(corner_location, query_location,
                                                        GetCorrectedCenter(x_idx - 1, y_idx, z_idx),
                                                        GetCorrectedCenter(x_idx - 1, y_idx, z_idx + 1),
                                                        GetCorrectedCenter(x_idx - 1, y_idx + 1, z_idx),
                                                        GetCorrectedCenter(x_idx - 1, y_idx + 1, z_idx + 1),
                                                        GetCorrectedCenter(x_idx, y_idx, z_idx),
                                                        GetCorrectedCenter(x_idx, y_idx, z_idx + 1),
                                                        GetCorrectedCenter(x_idx, y_idx + 1, z_idx),
                                                        GetCorrectedCenter(x_idx, y_idx + 1, z_idx + 1));
                }
                // -X+Y-Z
                else if ((query_offset(0) < 0.0) && (query_offset(1) >= 0.0) && (query_offset(2) < 0.0))
                {
                    const Eigen::Vector4d corner_location = GridIndexToLocation(x_idx - 1, y_idx, z_idx - 1);
                    return TrilinearInterpolateDistance(corner_location, query_location,
                                                        GetCorrectedCenter(x_idx - 1, y_idx, z_idx -1),
                                                        GetCorrectedCenter(x_idx - 1, y_idx, z_idx),
                                                        GetCorrectedCenter(x_idx - 1, y_idx + 1, z_idx - 1),
                                                        GetCorrectedCenter(x_idx - 1, y_idx + 1, z_idx),
                                                        GetCorrectedCenter(x_idx, y_idx, z_idx - 1),
                                                        GetCorrectedCenter(x_idx, y_idx, z_idx),
                                                        GetCorrectedCenter(x_idx, y_idx + 1, z_idx - 1),
                                                        GetCorrectedCenter(x_idx, y_idx + 1, z_idx));
                }
                // -X-Y+Z
                else if ((query_offset(0) < 0.0) && (query_offset(1) < 0.0) && (query_offset(2) >= 0.0))
                {
                    const Eigen::Vector4d corner_location = GridIndexToLocation(x_idx - 1, y_idx - 1, z_idx);
                    return TrilinearInterpolateDistance(corner_location, query_location,
                                                        GetCorrectedCenter(x_idx - 1, y_idx - 1, z_idx),
                                                        GetCorrectedCenter(x_idx - 1, y_idx - 1, z_idx + 1),
                                                        GetCorrectedCenter(x_idx - 1, y_idx, z_idx),
                                                        GetCorrectedCenter(x_idx - 1, y_idx, z_idx + 1),
                                                        GetCorrectedCenter(x_idx, y_idx - 1, z_idx),
                                                        GetCorrectedCenter(x_idx, y_idx - 1, z_idx + 1),
                                                        GetCorrectedCenter(x_idx, y_idx, z_idx),
                                                        GetCorrectedCenter(x_idx, y_idx, z_idx + 1));
                }
                // -X-Y-Z
                else if ((query_offset(0) < 0.0) && (query_offset(1) < 0.0) && (query_offset(2) < 0.0))
                {
                    const Eigen::Vector4d corner_location = GridIndexToLocation(x_idx - 1, y_idx - 1, z_idx - 1);
                    return TrilinearInterpolateDistance(corner_location, query_location,
                                                        GetCorrectedCenter(x_idx - 1, y_idx - 1, z_idx -1),
                                                        GetCorrectedCenter(x_idx - 1, y_idx - 1, z_idx),
                                                        GetCorrectedCenter(x_idx - 1, y_idx, z_idx - 1),
                                                        GetCorrectedCenter(x_idx - 1, y_idx, z_idx),
                                                        GetCorrectedCenter(x_idx, y_idx - 1, z_idx - 1),
                                                        GetCorrectedCenter(x_idx, y_idx - 1, z_idx),
                                                        GetCorrectedCenter(x_idx, y_idx, z_idx - 1),
                                                        GetCorrectedCenter(x_idx, y_idx, z_idx));
                }
                else
                {
                    throw std::runtime_error("Should not be possible to reach this!");
                }
            }
            catch (const std::invalid_argument& ex)
            {
                const std::string msg = "Query location in outer half-cell of grid - message: " + std::string(ex.what());
                throw std::runtime_error(msg);
            }
            catch (const std::runtime_error& ex)
            {
                const std::string msg = "Interpolation case not properly handled - message: " + std::string(ex.what());
                throw std::runtime_error(msg);
            }
        }

            /*
            // Z-Axis
            else if ((query_offset(0) == 0.0) && (query_offset(1) == 0.0))
            {
                if (query_offset(2) > 0.0)
                {
                    return LinearInterpolateZ(cell_center_location, query_location, Get(x_idx, y_idx, z_idx), Get(x_idx, y_idx, z_idx + 1));
                }
                else
                {
                    const Eigen::Vector4d corner_location = GridIndexToLocation(x_idx, y_idx, z_idx - 1);
                    return LinearInterpolateZ(corner_location, query_location, Get(x_idx, y_idx, z_idx - 1), Get(x_idx, y_idx, z_idx));
                }
            }
            // X-Axis
            else if ((query_offset(1) == 0.0) && (query_offset(2) == 0.0))
            {
                if (query_offset(0) > 0.0)
                {
                    return LinearInterpolateX(cell_center_location, query_location, Get(x_idx, y_idx, z_idx), Get(x_idx, y_idx + 1, z_idx));
                }
                else
                {
                    const Eigen::Vector4d corner_location = GridIndexToLocation(x_idx - 1, y_idx, z_idx);
                    return LinearInterpolateX(corner_location, query_location, Get(x_idx - 1, y_idx, z_idx), Get(x_idx, y_idx, z_idx));
                }
            }
            // Y-Axis
            else if ((query_offset(0) == 0.0) && (query_offset(2) == 0.0))
            {
                if (query_offset(1) > 0.0)
                {
                    return LinearInterpolateY(cell_center_location, query_location, Get(x_idx, y_idx, z_idx), Get(x_idx, y_idx + 1, z_idx));
                }
                else
                {
                    const Eigen::Vector4d corner_location = GridIndexToLocation(x_idx, y_idx - 1, z_idx);
                    return LinearInterpolateY(corner_location, query_location, Get(x_idx, y_idx - 1, z_idx), Get(x_idx, y_idx, z_idx));
                }
            }
            // YZ-Plane
            else if (query_offset(0) == 0.0)
            {
                if ((query_offset(1) > 0.0) && (query_offset(2) > 0.0))
                {
                    return BilinearInterpolateYZ(cell_center_location, query_location,
                                                 Get(x_idx, y_idx, z_idx), Get(x_idx, y_idx, z_idx + 1),
                                                 Get(x_idx, y_idx + 1, z_idx), Get(x_idx, y_idx + 1, z_idx + 1));
                }
                else if ((query_offset(1) > 0.0) && (query_offset(2) < 0.0))
                {
                    // CHECK
                    const Eigen::Vector4d corner_location = GridIndexToLocation(x_idx, y_idx, z_idx - 1);
                    return BilinearInterpolateYZ(corner_location, query_location,
                                                 Get(x_idx, y_idx, z_idx -1), Get(x_idx, y_idx, z_idx),
                                                 Get(x_idx, y_idx + 1, z_idx - 1), Get(x_idx, y_idx + 1, z_idx));
                }
                else if ((query_offset(1) < 0.0) && (query_offset(2) > 0.0))
                {
                    ;
                }
                else if ((query_offset(1) < 0.0) && (query_offset(2) < 0.0))
                {
                    ;
                }
                else
                {
                    assert(false && "Should not be possible to reach this!");
                }
            }
            // XZ-Plane
            else if (query_offset(1) == 0.0)
            {
                if ((query_offset(0) > 0.0) && (query_offset(2) > 0.0))
                {
                    ;
                }
                else if ((query_offset(0) > 0.0) && (query_offset(2) < 0.0))
                {
                    ;
                }
                else if ((query_offset(0) < 0.0) && (query_offset(2) > 0.0))
                {
                    ;
                }
                else if ((query_offset(0) < 0.0) && (query_offset(2) < 0.0))
                {
                    ;
                }
                else
                {
                    assert(false && "Should not be possible to reach this!");
                }
            }
            // XY-Plane
            else if (query_offset(2) == 0.0)
            {
                if ((query_offset(0) > 0.0) && (query_offset(1) > 0.0))
                {
                    ;
                }
                else if ((query_offset(0) > 0.0) && (query_offset(1) < 0.0))
                {
                    ;
                }
                else if ((query_offset(0) < 0.0) && (query_offset(1) > 0.0))
                {
                    ;
                }
                else if ((query_offset(0) < 0.0) && (query_offset(1) < 0.0))
                {
                    ;
                }
                else
                {
                    assert(false && "Should not be possible to reach this!");
                }
            }
            else
            {
                assert(false && "Should not be possible to reach this!");
            }
        }
        */

        inline double EstimateDistanceInternal(const Eigen::Vector4d& query_location, const int64_t x_idx, const int64_t y_idx, const int64_t z_idx) const
        {
#ifdef TEST_ESTIMATE_DISTANCE
            // We think the bi/tri-linear interpolation is a better method, but it doesn't work everywhere
            try
            {
                return EstimateDistanceInterpolateFromNeighbors(query_location, x_idx, y_idx, z_idx);
            }
            // Fall back to different methods
            catch (...)
            {
#endif
                // We need to handle the cases of boundary and non-boundary cells separately
                // Non-boundary cells can use a simpler approximation of distance adjustment,
                // since these cells have a more useful gradient
                // Boundary cells need special treatment since the gradient may exhibit discretization
                // errors that would produce an incorrect estimate using the simpler method
                const double nominal_sdf_distance = (double)distance_field_.GetImmutable(x_idx, y_idx, z_idx).first;
                // Account for diagonal distances
                if (std::abs(nominal_sdf_distance) > (GetResolution() * std::sqrt(3.001)))
                {
                    return EstimateDistanceNonBoundaryCell(query_location, x_idx, y_idx, z_idx, nominal_sdf_distance);
                }
                else
                {
                    return EstimateDistanceBoundaryCell(query_location, x_idx, y_idx, z_idx, nominal_sdf_distance);
                }
#ifdef TEST_ESTIMATE_DISTANCE
            }
#endif
        }

    public:

        inline std::pair<double, bool> EstimateDistance(const double x, const double y, const double z) const
        {
            return EstimateDistance4d(Eigen::Vector4d(x, y, z, 1.0));
        }

        inline std::pair<double, bool> EstimateDistance3d(const Eigen::Vector3d& location) const
        {
            const VoxelGrid::GRID_INDEX index = distance_field_.LocationToGridIndex3d(location);
            if (distance_field_.IndexInBounds(index))
            {
                return std::make_pair(EstimateDistanceInternal(Eigen::Vector4d(location.x(), location.y(), location.z(), 1.0), index.x, index.y, index.z), true);
            }
            else
            {
                return std::make_pair((double)distance_field_.GetOOBValue(), false);
            }
        }

        inline std::pair<double, bool> EstimateDistance4d(const Eigen::Vector4d& location) const
        {
            const VoxelGrid::GRID_INDEX index = distance_field_.LocationToGridIndex4d(location);
            if (distance_field_.IndexInBounds(index))
            {
                return std::make_pair(EstimateDistanceInternal(location, index.x, index.y, index.z), true);
            }
            else
            {
                return std::make_pair((double)distance_field_.GetOOBValue(), false);
            }
        }

        inline std::vector<double> GetGradient(const double x, const double y, const double z, const bool enable_edge_gradients=false) const
        {
            return GetGradient4d(Eigen::Vector4d(x, y, z, 1.0), enable_edge_gradients);
        }

        inline std::vector<double> GetGradient3d(const Eigen::Vector3d& location, const bool enable_edge_gradients=false) const
        {
            const VoxelGrid::GRID_INDEX index = distance_field_.LocationToGridIndex3d(location);
            if (distance_field_.IndexInBounds(index))
            {
                return GetGradient(index, enable_edge_gradients);
            }
            else
            {
                return std::vector<double>();
            }
        }

        inline std::vector<double> GetGradient4d(const Eigen::Vector4d& location, const bool enable_edge_gradients=false) const
        {
            const VoxelGrid::GRID_INDEX index = distance_field_.LocationToGridIndex4d(location);
            if (distance_field_.IndexInBounds(index))
            {
                return GetGradient(index, enable_edge_gradients);
            }
            else
            {
                return std::vector<double>();
            }
        }

        inline std::vector<double> GetGradient(const VoxelGrid::GRID_INDEX& index, const bool enable_edge_gradients=false) const
        {
            return GetGradient(index.x, index.y, index.z, enable_edge_gradients);
        }

        inline std::vector<double> GetGradient(const int64_t x_index, const int64_t y_index, const int64_t z_index, const bool enable_edge_gradients=false) const
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
                // If we're on the edge, handle it specially
                else if (enable_edge_gradients)
                {
                    // Get the "best" indices we can use
                    int64_t low_x_index = std::max((int64_t)0, x_index - 1);
                    int64_t high_x_index = std::min(GetNumXCells() - 1, x_index + 1);
                    int64_t low_y_index = std::max((int64_t)0, y_index - 1);
                    int64_t high_y_index = std::min(GetNumYCells() - 1, y_index + 1);
                    int64_t low_z_index = std::max((int64_t)0, z_index - 1);
                    int64_t high_z_index = std::min(GetNumZCells() - 1, z_index + 1);
                    // Compute the axis increments
                    double x_increment = (high_x_index - low_x_index) * GetResolution();
                    double y_increment = (high_y_index - low_y_index) * GetResolution();
                    double z_increment = (high_z_index - low_z_index) * GetResolution();
                    // Compute the gradients for each axis - by default these are zero
                    double gx = 0.0;
                    double gy = 0.0;
                    double gz = 0.0;
                    // Only if the increments are non-zero do we compute the gradient of an axis
                    if (x_increment > 0.0)
                    {
                        double inv_x_increment = 1.0 / x_increment;
                        double high_x_value = Get(high_x_index, y_index, z_index);
                        double low_x_value = Get(low_x_index, y_index, z_index);
                        // Compute the gradient
                        gx = (high_x_value - low_x_value) * inv_x_increment;
                    }
                    if (y_increment > 0.0)
                    {
                        double inv_y_increment = 1.0 / y_increment;
                        double high_y_value = Get(x_index, high_y_index, z_index);
                        double low_y_value = Get(x_index, low_y_index, z_index);
                        // Compute the gradient
                        gy = (high_y_value - low_y_value) * inv_y_increment;
                    }
                    if (z_increment > 0.0)
                    {
                        double inv_z_increment = 1.0 / z_increment;
                        double high_z_value = Get(x_index, y_index, high_z_index);
                        double low_z_value = Get(x_index, y_index, low_z_index);
                        // Compute the gradient
                        gz = (high_z_value - low_z_value) * inv_z_increment;
                    }
                    // Assemble and return the computed gradient
                    return std::vector<double>{gx, gy, gz};
                }
                // Edge gradients disabled, return no gradient
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

        inline Eigen::Vector3d ProjectOutOfCollision(const double x, const double y, const double z, const double stepsize_multiplier = 1.0 / 10.0) const
        {
            const Eigen::Vector4d result = ProjectOutOfCollision4d(Eigen::Vector4d(x, y, z, 1.0), stepsize_multiplier);
            return result.head<3>();
        }

        inline Eigen::Vector3d ProjectOutOfCollisionToMinimumDistance(const double x, const double y, const double z, const double minimum_distance, const double stepsize_multiplier = 1.0 / 10.0) const
        {
            const Eigen::Vector4d result = ProjectOutOfCollisionToMinimumDistance4d(Eigen::Vector4d(x, y, z, 1.0), minimum_distance, stepsize_multiplier);
            return result.head<3>();
        }

        inline Eigen::Vector3d ProjectOutOfCollision3d(const Eigen::Vector3d& location, const double stepsize_multiplier = 1.0 / 10.0) const
        {
            return ProjectOutOfCollision(location.x(), location.y(), location.z(), stepsize_multiplier);
        }

        inline Eigen::Vector3d ProjectOutOfCollisionToMinimumDistance3d(const Eigen::Vector3d& location, const double minimum_distance, const double stepsize_multiplier = 1.0 / 10.0) const
        {
            return ProjectOutOfCollisionToMinimumDistance(location.x(), location.y(), location.z(), minimum_distance, stepsize_multiplier);
        }

        inline Eigen::Vector4d ProjectOutOfCollision4d(const Eigen::Vector4d& location, const double stepsize_multiplier = 1.0 / 10.0) const
        {
            return ProjectOutOfCollisionToMinimumDistance4d(location, 0.0, stepsize_multiplier);
        }

        inline Eigen::Vector4d ProjectOutOfCollisionToMinimumDistance4d(const Eigen::Vector4d& location, const double minimum_distance, const double stepsize_multiplier = 1.0 / 10.0) const
        {
            // To avoid potential problems with alignment, we need to pass location by reference, so we make a local copy
            // here that we can change. https://eigen.tuxfamily.org/dox/group__TopicPassingByValue.html
            Eigen::Vector4d mutable_location = location;
            // If we are in bounds, start the projection process, otherwise return the location unchanged
            if (CheckInBounds4d(mutable_location))
            {
                // Add a small collision margin to account for rounding and similar
                const double minimum_distance_with_margin = minimum_distance + GetResolution() * stepsize_multiplier * 1e-3;
                const double max_stepsize = GetResolution() * stepsize_multiplier;
                const bool enable_edge_gradients = true;

                double sdf_dist = EstimateDistance4d(mutable_location).first;
                while (sdf_dist <= minimum_distance)
                {
                    const std::vector<double> gradient = GetGradient4d(mutable_location, enable_edge_gradients);
                    assert(gradient.size() == 3);
                    const Eigen::Vector4d grad_eigen(gradient[0], gradient[1], gradient[2], 0.0);
                    assert(grad_eigen.norm() > GetResolution() * 0.25); // Sanity check
                    // Don't step any farther than is needed
                    const double step_distance = std::min(max_stepsize, minimum_distance_with_margin - sdf_dist);
                    mutable_location += grad_eigen.normalized() * step_distance;
                    sdf_dist = EstimateDistance4d(mutable_location).first;
                }
            }
            return mutable_location;
        }

        inline const Eigen::Isometry3d& GetOriginTransform() const
        {
            return distance_field_.GetOriginTransform();
        }

        inline const Eigen::Isometry3d& GetInverseOriginTransform() const
        {
            return distance_field_.GetInverseOriginTransform();
        }

        inline std::string GetFrame() const
        {
            return frame_;
        }

        inline VoxelGrid::GRID_INDEX LocationToGridIndex3d(const Eigen::Vector3d& location) const
        {
            return distance_field_.LocationToGridIndex3d(location);
        }

        inline VoxelGrid::GRID_INDEX LocationToGridIndex4d(const Eigen::Vector4d& location) const
        {
            return distance_field_.LocationToGridIndex4d(location);
        }

        inline VoxelGrid::GRID_INDEX LocationToGridIndex(const double x, const double y, const double z) const
        {
            return distance_field_.LocationToGridIndex(x, y, z);
        }

        inline Eigen::Vector4d GridIndexToLocation(const VoxelGrid::GRID_INDEX& index) const
        {
            return distance_field_.GridIndexToLocation(index);
        }

        inline Eigen::Vector4d GridIndexToLocation(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            return distance_field_.GridIndexToLocation(x_index, y_index, z_index);
        }

        bool SaveToFile(const std::string& filepath);

        bool LoadFromFile(const std::string& filepath);

        sdf_tools::SDF GetMessageRepresentation();

        bool LoadFromMessageRepresentation(sdf_tools::SDF& message);

        visualization_msgs::Marker ExportForDisplay(float alpha = 0.01f) const;

        visualization_msgs::Marker ExportForDisplayCollisionOnly(float alpha = 0.01f) const;

        visualization_msgs::Marker ExportForDebug(float alpha = 0.5f) const;

        /*
         * The following function can be *VERY EXPENSIVE* to compute, since it performs gradient ascent across the SDF
         */
        VoxelGrid::VoxelGrid<Eigen::Vector3d> ComputeLocalMaximaMap() const;

        inline bool GradientIsEffectiveFlat(const Eigen::Vector3d& gradient) const
        {
            // A gradient is at a local maxima if the absolute value of all components (x,y,z) are less than 1/2 SDF resolution
            double half_resolution = GetResolution() * 0.5;
            if (fabs(gradient.x()) <= half_resolution && fabs(gradient.y()) <= half_resolution && fabs(gradient.z()) <= half_resolution)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        inline VoxelGrid::GRID_INDEX GetNextFromGradient(const VoxelGrid::GRID_INDEX& index, const Eigen::Vector3d& gradient) const
        {
            // Given the gradient, pick the "best fit" of the 26 neighboring points
            VoxelGrid::GRID_INDEX next_index = index;
            double half_resolution = GetResolution() * 0.5;
            if (gradient.x() > half_resolution)
            {
                next_index.x++;
            }
            else if (gradient.x() < -half_resolution)
            {
                next_index.x--;
            }
            if (gradient.y() > half_resolution)
            {
                next_index.y++;
            }
            else if (gradient.y() < -half_resolution)
            {
                next_index.y--;
            }
            if (gradient.z() > half_resolution)
            {
                next_index.z++;
            }
            else if (gradient.z() < -half_resolution)
            {
                next_index.z--;
            }
            return next_index;
        }
    };
}

#endif // SDF_HPP
