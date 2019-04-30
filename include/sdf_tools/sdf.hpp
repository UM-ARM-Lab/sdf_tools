#ifndef SDF_HPP
#define SDF_HPP

#include <stdlib.h>
#include <vector>
#include <string>
#include <stdexcept>
#include <Eigen/Geometry>
#include <unsupported/Eigen/AutoDiff>
#include <visualization_msgs/Marker.h>
#include <arc_utilities/serialization.hpp>
#include <arc_utilities/voxel_grid.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <sdf_tools/SDF.h>

namespace sdf_tools
{
using VoxelGrid::GRID_INDEX;

class SignedDistanceField : public VoxelGrid::VoxelGrid<float>
{
protected:

  std::string frame_;
  bool locked_;

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<SignedDistanceField> Ptr;
  typedef std::shared_ptr<const SignedDistanceField> ConstPtr;

  inline SignedDistanceField(const std::string& frame,
                             double resolution,
                             double x_size,
                             double y_size,
                             double z_size,
                             float OOB_value)
    : VoxelGrid::VoxelGrid<float>(resolution,
                                  x_size, y_size, z_size,
                                  OOB_value),
      frame_(frame), locked_(false) {}

  inline SignedDistanceField(const Eigen::Isometry3d& origin_transform,
                             const std::string& frame,
                             double resolution,
                             double x_size,
                             double y_size,
                             double z_size,
                             float OOB_value)
    : VoxelGrid::VoxelGrid<float>(origin_transform, resolution,
                                  x_size, y_size, z_size,
                                  OOB_value),
      frame_(frame), locked_(false) {}

  inline SignedDistanceField(const std::string& frame,
                             double resolution,
                             int64_t x_cells,
                             int64_t y_cells,
                             int64_t z_cells,
                             float OOB_value)
    : VoxelGrid::VoxelGrid<float>(resolution,
                                  x_cells, y_cells, z_cells,
                                  OOB_value),
      frame_(frame), locked_(false) {}

  inline SignedDistanceField(const Eigen::Isometry3d& origin_transform,
                             const std::string& frame,
                             double resolution,
                             int64_t x_cells,
                             int64_t y_cells,
                             int64_t z_cells,
                             float OOB_value)
    : VoxelGrid::VoxelGrid<float>(origin_transform, resolution,
                                  x_cells, y_cells, z_cells,
                                  OOB_value),
      frame_(frame), locked_(false) {}

  inline SignedDistanceField()
    : VoxelGrid::VoxelGrid<float>(), frame_(""), locked_(false) {}

  virtual VoxelGrid<float>* Clone() const
  {
    return new SignedDistanceField(
          static_cast<const SignedDistanceField&>(*this));
  }

  /////////////////////////////////////////////////////////////////////////
  // Misc data access
  ////////////////////////////////////////////////////////////////////////////

  inline double GetResolution() const
  {
    return GetCellSizes().x();
  }

  inline std::string GetFrame() const
  {
    return frame_;
  }

  inline void SetFrame(const std::string& new_frame)
  {
    frame_ = new_frame;
  }

  /////////////////////////////////////////////////////////////////////////
  // Access/mutability protection functions
  ////////////////////////////////////////////////////////////////////////////

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

  /////////////////////////////////////////////////////////////////////////
  // Mutable access and setter functions MUST be used carefully!
  // If you arbitrarily change SDF values, it is not a proper SDF any more!
  //
  // Use of these functions can be prevented by calling
  // SignedDistanceField::Lock() on the SDF
  ////////////////////////////////////////////////////////////////////////

  virtual std::pair<float&, bool> GetMutable3d(
      const Eigen::Vector3d& location)
  {
    const GRID_INDEX index = LocationToGridIndex3d(location);
    if (IndexInBounds(index))
    {
      return GetMutable(index);
    }
    else
    {
      return std::pair<float&, bool>(oob_value_, false);
    }
  }

  virtual std::pair<float&, bool> GetMutable4d(
      const Eigen::Vector4d& location)
  {
    const GRID_INDEX index = LocationToGridIndex4d(location);
    if (IndexInBounds(index))
    {
      return GetMutable(index);
    }
    else
    {
      return std::pair<float&, bool>(oob_value_, false);
    }
  }

  virtual std::pair<float&, bool> GetMutable(const double x,
                                             const double y,
                                             const double z)
  {
    const Eigen::Vector4d location(x, y, z, 1.0);
    return GetMutable4d(location);
  }

  virtual std::pair<float&, bool> GetMutable(const GRID_INDEX& index)
  {
    if (IndexInBounds(index) && !locked_)
    {
      return std::pair<float&, bool>(
            AccessIndex(GetDataIndex(index)), true);
    }
    else
    {
      return std::pair<float&, bool>(oob_value_, false);
    }
  }

  virtual std::pair<float&, bool> GetMutable(const int64_t x_index,
                                             const int64_t y_index,
                                             const int64_t z_index)
  {
    if (IndexInBounds(x_index, y_index, z_index) && !locked_)
    {
      return std::pair<float&, bool>(
            AccessIndex(GetDataIndex(x_index, y_index, z_index)), true);
    }
    else
    {
      return std::pair<float&, bool>(oob_value_, false);
    }
  }

  virtual bool SetValue3d(const Eigen::Vector3d& location,
                          const float& value)
  {
    const GRID_INDEX index = LocationToGridIndex3d(location);
    if (IndexInBounds(index))
    {
      return SetValue(index, value);
    }
    else
    {
      return false;
    }
  }

  virtual bool SetValue4d(const Eigen::Vector4d& location,
                          const float& value)
  {
    const GRID_INDEX index = LocationToGridIndex4d(location);
    if (IndexInBounds(index))
    {
      return SetValue(index, value);
    }
    else
    {
      return false;
    }
  }

  virtual bool SetValue(const double x,
                        const double y,
                        const double z,
                        const float& value)
  {
    const Eigen::Vector4d location(x, y, z, 1.0);
    return SetValue4d(location, value);
  }

  virtual bool SetValue(const GRID_INDEX& index,
                        const float& value)
  {
    if (IndexInBounds(index) && !locked_)
    {
      AccessIndex(GetDataIndex(index)) = value;
      return true;
    }
    else
    {
      return false;
    }
  }

  virtual bool SetValue(const int64_t x_index,
                        const int64_t y_index,
                        const int64_t z_index,
                        const float& value)
  {
    if (IndexInBounds(x_index, y_index, z_index) && !locked_)
    {
      AccessIndex(GetDataIndex(x_index, y_index, z_index)) = value;
      return true;
    }
    else
    {
      return false;
    }
  }

  virtual bool SetValue3d(const Eigen::Vector3d& location,
                          float&& value)
  {
    const GRID_INDEX index = LocationToGridIndex3d(location);
    if (IndexInBounds(index))
    {
      return SetValue(index, value);
    }
    else
    {
      return false;
    }
  }

  virtual bool SetValue4d(const Eigen::Vector4d& location,
                          float&& value)
  {
    const GRID_INDEX index = LocationToGridIndex4d(location);
    if (IndexInBounds(index))
    {
      return SetValue(index, value);
    }
    else
    {
      return false;
    }
  }

  virtual bool SetValue(const double x,
                        const double y,
                        const double z,
                        float&& value)
  {
    const Eigen::Vector4d location(x, y, z, 1.0);
    return SetValue4d(location, value);
  }

  virtual bool SetValue(const GRID_INDEX& index,
                        float&& value)
  {
    if (IndexInBounds(index) && !locked_)
    {
      AccessIndex(GetDataIndex(index)) = value;
      return true;
    }
    else
    {
      return false;
    }
  }

  virtual bool SetValue(const int64_t x_index,
                        const int64_t y_index,
                        const int64_t z_index,
                        float&& value)
  {
    if (IndexInBounds(x_index, y_index, z_index) && !locked_)
    {
      AccessIndex(GetDataIndex(x_index, y_index, z_index)) = value;
      return true;
    }
    else
    {
      return false;
    }
  }

  /////////////////////////////////////////////////////////////////////////
  // Gradient functions
  /////////////////////////////////////////////////////////////////////////

  using GradientFunction = std::function<std::vector<double>(int64_t,int64_t,int64_t,bool)>;

  /** the return vector is a flat array, which will be reshape to be
   * X/Y/Z/gradient when saved as numpy **/
  inline VoxelGrid<std::vector<double>> GetFullGradient(const GradientFunction& gradient_function,
                                                        const bool enable_edge_gradients = false) const
  {
    VoxelGrid<std::vector<double>> gradient_grid{origin_transform_, GetResolution(), GetNumXCells(), GetNumYCells(),
                                                 GetNumZCells(), std::vector<double>(3, oob_value_)};
    for (auto x_idx{0l}; x_idx < GetNumXCells(); ++x_idx)
    {
      for (auto y_idx{0l}; y_idx < GetNumYCells(); ++y_idx)
      {
        for (auto z_idx{0l}; z_idx < GetNumZCells(); ++z_idx)
        {
          auto const gradient = gradient_function(x_idx, y_idx, z_idx, enable_edge_gradients);
          gradient_grid.SetValue(x_idx, y_idx, z_idx, gradient);
        }
      }
    }
    return gradient_grid;
  }


  inline std::vector<double> GetGradient(
      const double x, const double y, const double z,
      const bool enable_edge_gradients = false) const
  {
    return GetGradient4d(Eigen::Vector4d(x, y, z, 1.0), enable_edge_gradients);
  }

  inline std::vector<double> GetGradient3d(
      const Eigen::Vector3d& location,
      const bool enable_edge_gradients = false) const
  {
    const GRID_INDEX index = LocationToGridIndex3d(location);
    if (IndexInBounds(index))
    {
      return GetGradient(index, enable_edge_gradients);
    }
    else
    {
      return std::vector<double>();
    }
  }

  inline std::vector<double> GetGradient4d(
      const Eigen::Vector4d& location,
      const bool enable_edge_gradients = false) const
  {
    const GRID_INDEX index = LocationToGridIndex4d(location);
    if (IndexInBounds(index))
    {
      return GetGradient(index, enable_edge_gradients);
    }
    else
    {
      return std::vector<double>();
    }
  }

  inline std::vector<double> GetGradient(
      const GRID_INDEX& index,
      const bool enable_edge_gradients = false) const
  {
    return GetGradient(index.x, index.y, index.z, enable_edge_gradients);
  }

  inline std::vector<double> GetGradient(
      const int64_t x_index, const int64_t y_index, const int64_t z_index,
      const bool enable_edge_gradients = false) const
  {
    const std::vector<double> grid_aligned_gradient
        = GetGridAlignedGradient(x_index, y_index, z_index,
                                 enable_edge_gradients);
    if (grid_aligned_gradient.size() == 3)
    {
      const Eigen::Quaterniond grid_rotation(origin_transform_.rotation());
      // Derived from EigenHelpers::RotateVector, but without extra copies
      const Eigen::Quaterniond temp(0.0,
                                    grid_aligned_gradient[0],
                                    grid_aligned_gradient[1],
                                    grid_aligned_gradient[2]);
      const Eigen::Quaterniond result
          = grid_rotation * (temp * grid_rotation.inverse());
      return std::vector<double>{result.x(),
                                 result.y(),
                                 result.z()};
    }
    else
    {
      return std::vector<double>();
    }
  }

  inline std::vector<double> GetGridAlignedGradient(
      const int64_t x_index, const int64_t y_index, const int64_t z_index,
      const bool enable_edge_gradients = false) const
  {
    // Make sure the index is inside bounds
    if (IndexInBounds(x_index, y_index, z_index))
    {
      // See if the index we're trying to query is one cell in from the edge
      if ((x_index > 0) && (y_index > 0) && (z_index > 0)
          && (x_index < (GetNumXCells() - 1))
          && (y_index < (GetNumYCells() - 1))
          && (z_index < (GetNumZCells() - 1)))
      {
        const double inv_twice_resolution = 1.0 / (2.0 * GetResolution());
        const double gx = (GetImmutable(x_index + 1, y_index, z_index).first
                           - GetImmutable(x_index - 1, y_index, z_index).first)
                          * inv_twice_resolution;
        const double gy = (GetImmutable(x_index, y_index + 1, z_index).first
                           - GetImmutable(x_index, y_index - 1, z_index).first)
                          * inv_twice_resolution;
        const double gz = (GetImmutable(x_index, y_index, z_index + 1).first
                           - GetImmutable(x_index, y_index, z_index - 1).first)
                          * inv_twice_resolution;
        return std::vector<double>{gx, gy, gz};
      }
      // If we're on the edge, handle it specially
      // TODO: we actually need to handle corners even more carefully,
      // since if the SDF is build with a virtual border, these cells will
      // get zero gradient from this approach!
      else if (enable_edge_gradients)
      {
        // Get the "best" indices we can use
        const int64_t low_x_index = std::max((int64_t)0, x_index - 1);
        const int64_t high_x_index = std::min(GetNumXCells() - 1, x_index + 1);
        const int64_t low_y_index = std::max((int64_t)0, y_index - 1);
        const int64_t high_y_index = std::min(GetNumYCells() - 1, y_index + 1);
        const int64_t low_z_index = std::max((int64_t)0, z_index - 1);
        const int64_t high_z_index = std::min(GetNumZCells() - 1, z_index + 1);
        // Compute the axis increments
        const double x_increment
            = (high_x_index - low_x_index) * GetResolution();
        const double y_increment
            = (high_y_index - low_y_index) * GetResolution();
        const double z_increment
            = (high_z_index - low_z_index) * GetResolution();
        // Compute the gradients for each axis - by default these are zero
        double gx = 0.0;
        double gy = 0.0;
        double gz = 0.0;
        // Only if the increments are non-zero do we compute the axis gradient
        if (x_increment > 0.0)
        {
          const double inv_x_increment = 1.0 / x_increment;
          const double high_x_value
              = GetImmutable(high_x_index, y_index, z_index).first;
          const double low_x_value
              = GetImmutable(low_x_index, y_index, z_index).first;
          // Compute the gradient
          gx = (high_x_value - low_x_value) * inv_x_increment;
        }
        if (y_increment > 0.0)
        {
          const double inv_y_increment = 1.0 / y_increment;
          const double high_y_value
              = GetImmutable(x_index, high_y_index, z_index).first;
          const double low_y_value
              = GetImmutable(x_index, low_y_index, z_index).first;
          // Compute the gradient
          gy = (high_y_value - low_y_value) * inv_y_increment;
        }
        if (z_increment > 0.0)
        {
          const double inv_z_increment = 1.0 / z_increment;
          const double high_z_value
              = GetImmutable(x_index, y_index, high_z_index).first;
          const double low_z_value
              = GetImmutable(x_index, y_index, low_z_index).first;
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

  inline std::vector<double> GetSmoothGradient3d(
      const Eigen::Vector3d& location,
      const double nominal_window_size) const
  {
    return GetSmoothGradient(location.x(), location.y(), location.z(),
                             nominal_window_size);
  }

  inline std::vector<double> GetSmoothGradient4d(
      const Eigen::Vector4d& location,
      const double nominal_window_size) const
  {
    return GetSmoothGradient(location(0), location(1), location(2),
                             nominal_window_size);
  }

  inline std::vector<double> GetSmoothGradient(
      const double x, const double y, const double z,
      const double nominal_window_size) const
  {
    const double ideal_window_size = std::abs(nominal_window_size);
    if (LocationInBounds(x, y, z))
    {
      const double min_x = x - ideal_window_size;
      const double max_x = x + ideal_window_size;
      const double min_y = y - ideal_window_size;
      const double max_y = y + ideal_window_size;
      const double min_z = z - ideal_window_size;
      const double max_z = z + ideal_window_size;
      // Retrieve distance estimates
      const std::pair<double, bool> point_distance = EstimateDistance(x, y, z);
      const std::pair<double, bool> mx_distance = EstimateDistance(min_x, y, z);
      const std::pair<double, bool> px_distance = EstimateDistance(max_x, y, z);
      const std::pair<double, bool> my_distance = EstimateDistance(x, min_y, z);
      const std::pair<double, bool> py_distance = EstimateDistance(x, max_y, z);
      const std::pair<double, bool> mz_distance = EstimateDistance(x, y, min_z);
      const std::pair<double, bool> pz_distance = EstimateDistance(x, y, max_z);
      // Compute gradient for each axis
      const double gx = ComputeAxisSmoothGradient(point_distance,
                                                  mx_distance,
                                                  px_distance,
                                                  x, min_x, max_x);
      const double gy = ComputeAxisSmoothGradient(point_distance,
                                                  my_distance,
                                                  py_distance,
                                                  y, min_y, max_y);
      const double gz = ComputeAxisSmoothGradient(point_distance,
                                                  mz_distance,
                                                  pz_distance,
                                                  z, min_z, max_z);
      return std::vector<double>{gx, gy, gz};
    }
    else
    {
      return std::vector<double>();
    }
  }

  inline std::vector<double> GetSmoothGradient(
      const GRID_INDEX& index, const double nominal_window_size) const
  {
    return GetSmoothGradient4d(GridIndexToLocation(index), nominal_window_size);
  }

  inline std::vector<double> GetSmoothGradient(
      const int64_t x_index, const int64_t y_index, const int64_t z_index,
      const double nominal_window_size) const
  {
    return GetSmoothGradient4d(
          GridIndexToLocation(x_index, y_index, z_index), nominal_window_size);
  }

  inline std::vector<double> GetAutoDiffGradient3d(
      const Eigen::Vector3d& location) const
  {
    return GetAutoDiffGradient(location.x(), location.y(), location.z());
  }

  inline std::vector<double> GetAutoDiffGradient4d(
      const Eigen::Vector4d& location) const
  {
    return GetAutoDiffGradient(location(0), location(1), location(2));
  }

  // TODO: this does not work if you query at cell centers!
  inline std::vector<double> GetAutoDiffGradient(
      const double x, const double y, const double z) const
  {
    const GRID_INDEX index = LocationToGridIndex(x, y, z);
    if (IndexInBounds(index))
    {
      // Use with AutoDiffScalar
      typedef Eigen::AutoDiffScalar<Eigen::Vector4d> AScalar;
      typedef Eigen::Matrix<AScalar, 4, 1> APosition;
      APosition Alocation;
      Alocation(0) = x;
      Alocation(1) = y;
      Alocation(2) = z;
      Alocation(3) = 1.0;
      Alocation(0).derivatives() = Eigen::Vector4d::Unit(0);
      Alocation(1).derivatives() = Eigen::Vector4d::Unit(1);
      Alocation(2).derivatives() = Eigen::Vector4d::Unit(2);
      Alocation(3).derivatives() = Eigen::Vector4d::Unit(3);
      AScalar Adist = EstimateDistanceInterpolateFromNeighbors<AScalar>(
                        Alocation, index.x, index.y, index.z);
      return std::vector<double>{
        Adist.derivatives()(0), Adist.derivatives()(1), Adist.derivatives()(2)};
    }
    else
    {
      return std::vector<double>();
    }
  }

  inline std::vector<double> GetAutoDiffGradient(const GRID_INDEX& index) const
  {
    return GetAutoDiffGradient4d(GridIndexToLocation(index));
  }

  inline std::vector<double> GetAutoDiffGradient(
      const int64_t x_index, const int64_t y_index, const int64_t z_index) const
  {
    return GetAutoDiffGradient4d(
          GridIndexToLocation(x_index, y_index, z_index));
  }

protected:

  inline double ComputeAxisSmoothGradient(
      const std::pair<double, bool>& query_point_distance_estimate,
      const std::pair<double, bool>& minus_axis_distance_estimate,
      const std::pair<double, bool>& plus_axis_distance_estimate,
      const double query_point_axis_value,
      const double minus_point_axis_value,
      const double plus_point_axis_value) const
  {
    if (query_point_distance_estimate.second
        && minus_axis_distance_estimate.second
        && plus_axis_distance_estimate.second)
    {
      const double window_size = plus_point_axis_value
                                 - minus_point_axis_value;
      const double distance_delta = plus_axis_distance_estimate.first
                                    - minus_axis_distance_estimate.first;
      return distance_delta / window_size;
    }
    else if (query_point_distance_estimate.second
             && minus_axis_distance_estimate.second)
    {
      const double window_size = query_point_axis_value
                                 - minus_point_axis_value;
      const double distance_delta = query_point_distance_estimate.first
                                    - minus_axis_distance_estimate.first;
      return distance_delta / window_size;
    }
    else if (query_point_distance_estimate.second
             && plus_axis_distance_estimate.second)
    {
      const double window_size = plus_point_axis_value
                                 - query_point_axis_value;
      const double distance_delta = plus_axis_distance_estimate.first
                                    - query_point_distance_estimate.first;
      return distance_delta / window_size;
    }
    else
    {
      throw std::runtime_error(
            "Window size for GetSmoothGradient is too large for SDF");
    }
  }

  template<typename T>
  inline T BilinearInterpolate(const double low_d1,
                               const double high_d1,
                               const double low_d2,
                               const double high_d2,
                               const T query_d1,
                               const T query_d2,
                               const double l1l2_val,
                               const double l1h2_val,
                               const double h1l2_val,
                               const double h1h2_val) const
  {
    Eigen::Matrix<T, 1, 2> d1_offsets;
    d1_offsets(0, 0) = high_d1 - query_d1;
    d1_offsets(0, 1) = query_d1 - low_d1;
    Eigen::Matrix<T, 2, 2> values;
    values(0, 0) = l1l2_val;
    values(0, 1) = l1h2_val;
    values(1, 0) = h1l2_val;
    values(1, 1) = h1h2_val;
    Eigen::Matrix<T, 2, 1> d2_offsets;
    d2_offsets(0, 0) = high_d2 - query_d2;
    d2_offsets(1, 0) = query_d2 - low_d2;
    const T multiplier = 1.0 / ((high_d1 - low_d1) * (high_d2 - low_d2));
    const T bilinear_interpolated
        = multiplier * d1_offsets * values * d2_offsets;
    return bilinear_interpolated;
  }

  template<typename T>
  inline T BilinearInterpolateDistanceXY(
      const Eigen::Vector4d& corner_location,
      const Eigen::Matrix<T, 4, 1>& query_location,
      const double mxmy_dist, const double mxpy_dist,
      const double pxmy_dist, const double pxpy_dist) const
  {
    return BilinearInterpolate(corner_location(0),
                               corner_location(0) + GetResolution(),
                               corner_location(1),
                               corner_location(1) + GetResolution(),
                               query_location(0),
                               query_location(1),
                               mxmy_dist, mxpy_dist,
                               pxmy_dist, pxpy_dist);
  }

  template<typename T>
  inline T TrilinearInterpolateDistance(
      const Eigen::Vector4d& corner_location,
      const Eigen::Matrix<T, 4, 1>& query_location,
      const double mxmymz_dist, const double mxmypz_dist,
      const double mxpymz_dist, const double mxpypz_dist,
      const double pxmymz_dist, const double pxmypz_dist,
      const double pxpymz_dist, const double pxpypz_dist) const
  {
    // Do bilinear interpolation in the lower XY plane
    const T mz_bilinear_interpolated
        = BilinearInterpolateDistanceXY(corner_location, query_location,
                                        mxmymz_dist, mxpymz_dist,
                                        pxmymz_dist, pxpymz_dist);
    // Do bilinear interpolation in the upper XY plane
    const T pz_bilinear_interpolated
        = BilinearInterpolateDistanceXY(corner_location, query_location,
                                        mxmypz_dist, mxpypz_dist,
                                        pxmypz_dist, pxpypz_dist);
    // Perform linear interpolation/extrapolation between lower and upper planes
    const double inv_resolution = 1.0 / GetResolution();
    const T distance_delta
        = pz_bilinear_interpolated - mz_bilinear_interpolated;
    const T distance_slope = distance_delta * inv_resolution;
    const T query_z_delta = query_location(2) - T(corner_location(2));
    return mz_bilinear_interpolated + (query_z_delta * distance_slope);
  }

  inline double GetCorrectedCenterDistance(const int64_t x_idx,
                                           const int64_t y_idx,
                                           const int64_t z_idx) const
  {
    const std::pair<const float&, bool> query
        = GetImmutable(x_idx, y_idx, z_idx);
    if (query.second)
    {
      const double nominal_sdf_distance = (double)query.first;
      const double cell_center_distance_offset = GetResolution() * 0.5;
      if (nominal_sdf_distance >= 0.0)
      {
        return nominal_sdf_distance - cell_center_distance_offset;
      }
      else
      {
        return nominal_sdf_distance + cell_center_distance_offset;
      }
    }
    else
    {
        throw std::invalid_argument("Index out of bounds");
    }
  }

  template<typename T>
  std::pair<int64_t, int64_t> GetAxisInterpolationIndices(
      const int64_t initial_index,
      const int64_t axis_size,
      const T axis_offset) const
  {
    int64_t lower = initial_index;
    int64_t upper = initial_index;
    if (axis_offset >= 0.0)
    {
      upper = initial_index + 1;
      if (upper >= axis_size)
      {
        upper = initial_index;
        lower = initial_index -1;
        if (lower < 0)
        {
          lower = initial_index;
        }
      }
    }
    else
    {
      lower = initial_index - 1;
      if (lower < 0)
      {
        upper = initial_index + 1;
        lower = initial_index;
        if (upper >= axis_size)
        {
          upper = initial_index;
        }
      }
    }
    return std::make_pair(lower, upper);
  }

  template<typename T>
  inline T EstimateDistanceInterpolateFromNeighborsGridFrame(
          const Eigen::Matrix<T, 4, 1>& grid_frame_query_location,
          const int64_t x_idx, const int64_t y_idx, const int64_t z_idx) const
  {
      // Switch between all the possible options of where we are
      const Eigen::Vector4d cell_center_location
          = GridIndexToLocationGridFrame(x_idx, y_idx, z_idx);
      const Eigen::Matrix<T, 4, 1> query_offset
          = grid_frame_query_location - cell_center_location.cast<T>();
      // Catch the easiest case
  //    if ((query_offset(0) == 0.0)
  //        && (query_offset(1) == 0.0)
  //        && (query_offset(2) == 0.0))
  //    {
  //      return GetCorrectedCenterDistance(x_idx, y_idx, z_idx);
  //    }
      // Find the best-matching 8 surrounding cell centers
      const std::pair<int64_t, int64_t> x_axis_indices
          = GetAxisInterpolationIndices(x_idx, GetNumXCells(), query_offset(0));
      const std::pair<int64_t, int64_t> y_axis_indices
          = GetAxisInterpolationIndices(y_idx, GetNumYCells(), query_offset(1));
      const std::pair<int64_t, int64_t> z_axis_indices
          = GetAxisInterpolationIndices(z_idx, GetNumZCells(), query_offset(2));
      const Eigen::Vector4d lower_corner_location
          = GridIndexToLocationGridFrame(x_axis_indices.first,
                                         y_axis_indices.first,
                                         z_axis_indices.first);
      const double mxmymz_distance
          = GetCorrectedCenterDistance(x_axis_indices.first,
                                       y_axis_indices.first,
                                       z_axis_indices.first);
      const double mxmypz_distance
          = GetCorrectedCenterDistance(x_axis_indices.first,
                                       y_axis_indices.first,
                                       z_axis_indices.second);
      const double mxpymz_distance
          = GetCorrectedCenterDistance(x_axis_indices.first,
                                       y_axis_indices.second,
                                       z_axis_indices.first);
      const double mxpypz_distance
          = GetCorrectedCenterDistance(x_axis_indices.first,
                                       y_axis_indices.second,
                                       z_axis_indices.second);
      const double pxmymz_distance
          = GetCorrectedCenterDistance(x_axis_indices.second,
                                       y_axis_indices.first,
                                       z_axis_indices.first);
      const double pxmypz_distance
          = GetCorrectedCenterDistance(x_axis_indices.second,
                                       y_axis_indices.first,
                                       z_axis_indices.second);
      const double pxpymz_distance
          = GetCorrectedCenterDistance(x_axis_indices.second,
                                       y_axis_indices.second,
                                       z_axis_indices.first);
      const double pxpypz_distance
          = GetCorrectedCenterDistance(x_axis_indices.second,
                                       y_axis_indices.second,
                                       z_axis_indices.second);
      return TrilinearInterpolateDistance(lower_corner_location,
                                          grid_frame_query_location,
                                          mxmymz_distance, mxmypz_distance,
                                          mxpymz_distance, mxpypz_distance,
                                          pxmymz_distance, pxmypz_distance,
                                          pxpymz_distance, pxpypz_distance);
  }

  template<typename T>
  inline T EstimateDistanceInterpolateFromNeighbors(
      const Eigen::Matrix<T, 4, 1>& query_location,
      const int64_t x_idx, const int64_t y_idx, const int64_t z_idx) const
  {
    // Get the query location in grid frame
    const Eigen::Matrix<T, 4, 1> grid_frame_query_location
        = inverse_origin_transform_ * query_location;
    return EstimateDistanceInterpolateFromNeighborsGridFrame(
          grid_frame_query_location,
          x_idx, y_idx, z_idx);
  }

public:

  /////////////////////////////////////////////////////////////////////
  // Estimate distance functions
  /////////////////////////////////////////////////////////////////////

  inline std::pair<double, bool> EstimateDistance(const double x,
                                                  const double y,
                                                  const double z) const
  {
      return EstimateDistance4d(Eigen::Vector4d(x, y, z, 1.0));
  }

  inline std::pair<double, bool> EstimateDistance3d(
      const Eigen::Vector3d& location) const
  {
    const GRID_INDEX index = LocationToGridIndex3d(location);
    if (IndexInBounds(index))
    {
      return std::make_pair(
            EstimateDistanceInterpolateFromNeighbors<double>(
              Eigen::Vector4d(location.x(), location.y(), location.z(), 1.0),
              index.x, index.y, index.z),
            true);
    }
    else
    {
      return std::make_pair((double)GetOOBValue(), false);
    }
  }

  inline std::pair<double, bool> EstimateDistance4d(
      const Eigen::Vector4d& location) const
  {
    const GRID_INDEX index = LocationToGridIndex4d(location);
    if (IndexInBounds(index))
    {
      return std::make_pair(EstimateDistanceInterpolateFromNeighbors<double>(
                              location, index.x, index.y, index.z),
                            true);
    }
    else
    {
      return std::make_pair((double)GetOOBValue(), false);
    }
  }

  inline std::pair<double, bool> DistanceToBoundary(const double x,
                                                    const double y,
                                                    const double z) const
  {
    return DistanceToBoundary4d(Eigen::Vector4d(x, y, z, 1.0));
  }

  inline std::pair<double, bool> DistanceToBoundary3d(
          const Eigen::Vector3d& location) const
  {
    return DistanceToBoundary(location.x(), location.y(), location.z());
  }

  inline std::pair<double, bool> DistanceToBoundary4d(
          const Eigen::Vector4d& location) const
  {
      const auto aligned_location = inverse_origin_transform_ * location;
      const auto displacements = Eigen::Array3d(
                  std::min(aligned_location(0), x_size_ - aligned_location(0)),
                  std::min(aligned_location(1), y_size_ - aligned_location(1)),
                  std::min(aligned_location(2), z_size_ - aligned_location(2)));
      const bool point_inside = (displacements >= 0.0).all();
      const Eigen::Array3d distances = displacements.abs();
      Eigen::Array3d::Index min_index;
      distances.minCoeff(&min_index);
      return {displacements(min_index), point_inside};
  }


  /////////////////////////////////////////////////////////////////////
  // Projection functions
  /////////////////////////////////////////////////////////////////////

  inline Eigen::Vector3d ProjectOutOfCollision(
      const double x, const double y, const double z,
      const double stepsize_multiplier = 1.0 / 8.0) const
  {
    const Eigen::Vector4d result
        = ProjectOutOfCollision4d(Eigen::Vector4d(x, y, z, 1.0),
                                  stepsize_multiplier);
    return result.head<3>();
  }

  inline Eigen::Vector3d ProjectOutOfCollision3d(
      const Eigen::Vector3d& location,
      const double stepsize_multiplier = 1.0 / 8.0) const
  {
    return ProjectOutOfCollision(location.x(), location.y(), location.z(),
                                 stepsize_multiplier);
  }

  inline Eigen::Vector4d ProjectOutOfCollision4d(
      const Eigen::Vector4d& location,
      const double stepsize_multiplier = 1.0 / 8.0) const
  {
    return ProjectOutOfCollisionToMinimumDistance4d(location, 0.0,
                                                    stepsize_multiplier);
  }

  inline Eigen::Vector3d ProjectOutOfCollisionToMinimumDistance(
      const double x, const double y, const double z,
      const double minimum_distance,
      const double stepsize_multiplier = 1.0 / 8.0) const
  {
    return ProjectOutOfCollisionToMinimumDistance4d(
          Eigen::Vector4d(x, y, z, 1.0),
          minimum_distance, stepsize_multiplier).head<3>();
  }

  inline Eigen::Vector3d ProjectOutOfCollisionToMinimumDistance3d(
      const Eigen::Vector3d& location, const double minimum_distance,
      const double stepsize_multiplier = 1.0 / 8.0) const
  {
    return ProjectOutOfCollisionToMinimumDistance(
          location.x(), location.y(), location.z(),
          minimum_distance, stepsize_multiplier);
  }

  inline Eigen::Vector4d ProjectOutOfCollisionToMinimumDistance4d(
      const Eigen::Vector4d& location,
      const double minimum_distance,
      const double stepsize_multiplier = 1.0 / 8.0) const
  {
    // To avoid potential problems with alignment, we need to pass location
    // by reference, so we make a local copy here that we can change.
    // https://eigen.tuxfamily.org/dox/group__TopicPassingByValue.html
    Eigen::Vector4d mutable_location = location;
    // If we are passed a value outside of the SDF,
    // first move it into the valid region of the SDF
    if (!LocationInBounds4d(location))
    {
      mutable_location = ProjectIntoValidVolume4d(location);
    }

    // Convert everything to local frame here so that we don't
    // need to do so repeatedly in the projection loop
    const Eigen::Vector4d location_in_local_frame =
          inverse_origin_transform_ * mutable_location;
    const Eigen::Vector4d result_in_local_frame =
        ProjectOutOfCollisionToMinimumDistanceGridFrame(
          location_in_local_frame,
          minimum_distance,
          stepsize_multiplier);
    const Eigen::Vector4d result_in_world_frame =
          GetOriginTransform() * result_in_local_frame;
    return result_in_world_frame;
  }

  Eigen::Vector4d ProjectOutOfCollisionToMinimumDistanceGridFrame(
          const Eigen::Vector4d& grid_frame_location,
          const double minimum_distance,
          const double stepsize_multiplier) const
  {
    // To avoid potential problems with alignment, we need to pass location
    // by reference, so we make a local copy here that we can change.
    // https://eigen.tuxfamily.org/dox/group__TopicPassingByValue.html
    Eigen::Vector4d mutable_location = grid_frame_location;
    // TODO: make this additional margin configurable
    // Add a small collision margin to account for rounding and similar
    const double minimum_distance_with_margin
      = minimum_distance + GetResolution() * stepsize_multiplier * 1e-4;
    const double max_stepsize = GetResolution() * stepsize_multiplier;
    const bool enable_edge_gradients = true;
    // index and sdf_dist are used to track the exact grid frame location
    // and the particular cell we are in at each loop iteration
    GRID_INDEX index = PointInFrameToGridIndex4d(mutable_location);
    double sdf_dist = EstimateDistanceInterpolateFromNeighborsGridFrame(
          mutable_location, index.x, index.y, index.z);
    while (sdf_dist <= minimum_distance)
    {
      const std::vector<double> gradient = GetGridAlignedGradient(
            index.x, index.y, index.z, enable_edge_gradients);
      if (gradient.size() == 3)
      {
        const Eigen::Vector4d grad_vector(
              gradient[0], gradient[1], gradient[2], 0.0);
        if (grad_vector.norm() > GetResolution() * 0.25) // Sanity check
        {
          // Don't step any farther than is needed
          const double step_distance = std::min(
                max_stepsize, minimum_distance_with_margin - sdf_dist);
          mutable_location += grad_vector.normalized() * step_distance;
          // Update indices and distances with the correct values
          // for the new location
          index = PointInFrameToGridIndex4d(mutable_location);
          sdf_dist = EstimateDistanceInterpolateFromNeighborsGridFrame(
                mutable_location, index.x, index.y, index.z);
        }
        else
        {
          throw std::runtime_error("Encountered flat gradient - stuck");
        }
      }
      else
      {
        throw std::runtime_error("Failed to compute gradient - out of SDF?");
      }
    }
    return mutable_location;
  }

  inline Eigen::Vector3d ProjectIntoValidVolume(
          const double x, const double y, const double z) const
  {
      const Eigen::Vector4d result =
              ProjectIntoValidVolume4d(Eigen::Vector4d(x, y, z, 1.0));
      return result.head<3>();
  }

  inline Eigen::Vector3d ProjectIntoValidVolumeToMinimumDistance(
          const double x, const double y, const double z,
          const double minimum_distance) const
  {
      const Eigen::Vector4d result =
          ProjectIntoValidVolumeToMinimumDistance4d(
            Eigen::Vector4d(x, y, z, 1.0), minimum_distance);
      return result.head<3>();
  }

  inline Eigen::Vector3d ProjectIntoValidVolume3d(
          const Eigen::Vector3d& location) const
  {
      return ProjectIntoValidVolume(location.x(), location.y(), location.z());
  }

  inline Eigen::Vector3d ProjectIntoValidVolumeToMinimumDistance3d(
          const Eigen::Vector3d& location,
          const double minimum_distance) const
  {
      return ProjectIntoValidVolumeToMinimumDistance(
            location.x(), location.y(), location.z(), minimum_distance);
  }

  inline Eigen::Vector4d ProjectIntoValidVolume4d(
          const Eigen::Vector4d& location) const
  {
      return ProjectIntoValidVolumeToMinimumDistance4d(location, 0.0);
  }

  inline Eigen::Vector4d ProjectIntoValidVolumeToMinimumDistance4d(
          const Eigen::Vector4d& location,
          const double minimum_distance) const
  {
      const auto grid_frame_location = inverse_origin_transform_ * location;
      // TODO: make this additional margin configurable - it's used to
      // ensure that the resulting value is inside the valid region even
      // after rotating back to the world frame
      const double dist_margin = minimum_distance + GetResolution() * 1e-4;
      const double x = arc_helpers::ClampValue(
                  grid_frame_location(0), dist_margin, x_size_ - dist_margin);
      const double y = arc_helpers::ClampValue(
                  grid_frame_location(1), dist_margin, y_size_ - dist_margin);
      const double z = arc_helpers::ClampValue(
                  grid_frame_location(2), dist_margin, z_size_ - dist_margin);
      // To avoid numerical problems, only return a modified
      // location if we actually had to change something
      bool change_made = false;
      change_made |= (x != grid_frame_location(0));
      change_made |= (y != grid_frame_location(1));
      change_made |= (z != grid_frame_location(2));
      if (change_made)
      {
          return GetOriginTransform() * Eigen::Vector4d(x, y, z, 1.0);
      }
      else
      {
          return location;
      }
  }

  /////////////////////////////////////////////////////////////////////
  // Local extrema map computation
  /////////////////////////////////////////////////////////////////////

protected:
  /*
   * You *MUST* provide valid indices to this function, hence why it is
   * protected (there are safe wrappers available - use them!)
   */
  void FollowGradientsToLocalExtremaUnsafe(
      VoxelGrid<Eigen::Vector3d>& watershed_map,
      const int64_t x_index,
      const int64_t y_index,
      const int64_t z_index) const;

  bool GradientIsEffectiveFlat(const Eigen::Vector3d& gradient) const;

  GRID_INDEX GetNextFromGradient(const GRID_INDEX& index,
                                 const Eigen::Vector3d& gradient) const;
public:

  /*
   * The following function can be *VERY EXPENSIVE* to compute, since it
   * performs gradient ascent/descent across the SDF
   */
  VoxelGrid<Eigen::Vector3d> ComputeLocalExtremaMap() const;

  /////////////////////////////////////////////////////////////////////
  // Serialization, saving, loading, etc.
  /////////////////////////////////////////////////////////////////////

  virtual uint64_t SerializeSelf(
      std::vector<uint8_t>& buffer,
      const std::function<uint64_t(
        const float&, std::vector<uint8_t>&)>& value_serializer
      = arc_utilities::SerializeFixedSizePOD<float>) const;

  virtual uint64_t DeserializeSelf(
      const std::vector<uint8_t>& buffer, const uint64_t current,
      const std::function<std::pair<float, uint64_t>(
        const std::vector<uint8_t>&, const uint64_t)>& value_deserializer
      = arc_utilities::DeserializeFixedSizePOD<float>);

  static void SaveToFile(const SignedDistanceField& sdf,
                         const std::string& filepath,
                         const bool compress);

  static SignedDistanceField LoadFromFile(const std::string& filepath);

  static sdf_tools::SDF GetMessageRepresentation(
      const SignedDistanceField& sdf);

  static SignedDistanceField LoadFromMessageRepresentation(
      const sdf_tools::SDF& message);

  visualization_msgs::Marker ExportForDisplay(const float alpha = 0.01f) const;

  visualization_msgs::Marker ExportForDisplayCollisionOnly(
      const float alpha = 0.01f) const;
};
}

#endif // SDF_HPP
