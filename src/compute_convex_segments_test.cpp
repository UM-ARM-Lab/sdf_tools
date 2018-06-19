#include "arc_utilities/pretty_print.hpp"
#include "sdf_tools/tagged_object_collision_map.hpp"
#include "sdf_tools/sdf.hpp"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include <functional>
#include <arc_utilities/eigen_helpers_conversions.hpp>

void test_compute_convex_segments(
    const std::function<void(
      const visualization_msgs::MarkerArray&)>& display_fn)
{
  const double res = 1.0;
  const double size = 100.0;
  const Eigen::Isometry3d origin_transform
      = Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Quaterniond(
          Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));
  sdf_tools::TaggedObjectCollisionMapGrid tocmap(origin_transform, "world", res, size, size, 1.0, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(0.0, 0u));
  for (int64_t x_idx = 0; x_idx < tocmap.GetNumXCells(); x_idx++)
  {
    for (int64_t y_idx = 0; y_idx < tocmap.GetNumYCells(); y_idx++)
    {
      if ((x_idx < 10) || (y_idx < 10) || (x_idx >= tocmap.GetNumXCells() - 10) || (y_idx >= tocmap.GetNumYCells() - 10))
      {
        tocmap.SetValue(x_idx, y_idx, 0, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(1.0, 1u));
      }
      else if ((x_idx >= 40) && (y_idx >= 40) && (x_idx < 60) && (y_idx < 60))
      {
        tocmap.SetValue(x_idx, y_idx, 0, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(1.0, 2u));
      }
      if ((x_idx == 0) || (y_idx == 0) || (x_idx == tocmap.GetNumXCells() - 1) || (y_idx == tocmap.GetNumYCells() - 1))
      {
        tocmap.SetValue(x_idx, y_idx, 0, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(0.0, 0u));
      }
    }
  }
  visualization_msgs::MarkerArray display_markers;
  visualization_msgs::Marker env_marker = tocmap.ExportForDisplay();
  env_marker.id = 1;
  env_marker.ns = "environment";
  display_markers.markers.push_back(env_marker);
  visualization_msgs::Marker components_marker = tocmap.ExportConnectedComponentsForDisplay(false);
  components_marker.id = 1;
  components_marker.ns = "environment_components";
  display_markers.markers.push_back(components_marker);
  visualization_msgs::Marker sdf_marker = tocmap.ExtractSignedDistanceField(0.0, std::vector<uint32_t>()).first.ExportForDisplay(1.0f);
  sdf_marker.id = 1;
  sdf_marker.ns = "environment_sdf";
  display_markers.markers.push_back(sdf_marker);
  const double connected_threshold = 2.0;
  const uint32_t number_of_convex_segments = tocmap.UpdateConvexSegments(connected_threshold);
  std::cout << "Identified " << number_of_convex_segments
            << " convex segments via SDF->maxima map->connected components"
            << std::endl;
  // Draw all the convex segments for each object
  visualization_msgs::Marker convex_segments_rep;
  // Populate the header
  convex_segments_rep.header.frame_id = tocmap.GetFrame();
  // Populate the options
  convex_segments_rep.ns = "convex_segments";
  convex_segments_rep.id = 1;
  convex_segments_rep.type = visualization_msgs::Marker::CUBE_LIST;
  convex_segments_rep.action = visualization_msgs::Marker::ADD;
  convex_segments_rep.lifetime = ros::Duration(0.0);
  convex_segments_rep.frame_locked = false;
  convex_segments_rep.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(
                       tocmap.GetOriginTransform());
  convex_segments_rep.scale.x = tocmap.GetResolution();
  convex_segments_rep.scale.y = tocmap.GetResolution();
  convex_segments_rep.scale.z = tocmap.GetResolution();
  // Add all the cells of the SDF to the message
  for (int64_t x_index = 0; x_index < tocmap.GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < tocmap.GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < tocmap.GetNumZCells(); z_index++)
      {
        // Convert grid indices into a real-world location
        const Eigen::Vector4d location
            = tocmap.GridIndexToLocationGridFrame(x_index, y_index, z_index);
        geometry_msgs::Point new_point;
        new_point.x = location(0);
        new_point.y = location(1);
        new_point.z = location(2);
        const auto& current_cell
            = tocmap.GetImmutable(x_index, y_index, z_index).first;
        if (current_cell.occupancy < 0.5)
        {
          const std_msgs::ColorRGBA object_color
              = arc_helpers::GenerateUniqueColor<std_msgs::ColorRGBA>(current_cell.convex_segment, 1.0f);
          if (object_color.a > 0.0)
          {
            convex_segments_rep.points.push_back(new_point);
            convex_segments_rep.colors.push_back(object_color);
          }
        }
      }
    }
  }
  display_markers.markers.push_back(convex_segments_rep);
  for (uint32_t object_id = 0u; object_id <= 4u; object_id++)
  {
    for (uint32_t convex_segment = 1u; convex_segment <= number_of_convex_segments; convex_segment++)
    {
      const visualization_msgs::Marker segment_marker = tocmap.ExportConvexSegmentForDisplay(object_id, convex_segment);
      if (segment_marker.points.size() > 0)
      {
        display_markers.markers.push_back(segment_marker);
      }
    }
  }
  const auto sdf_result
      = tocmap.ExtractSignedDistanceField(std::numeric_limits<float>::infinity(),
                                          std::vector<uint32_t>());
  const sdf_tools::SignedDistanceField& sdf = sdf_result.first;
  const VoxelGrid::VoxelGrid<Eigen::Vector3d> maxima_map = sdf.ComputeLocalExtremaMap();
  // Make gradient markers
  for (int64_t x_idx = 0; x_idx < sdf.GetNumXCells(); x_idx++)
  {
    for (int64_t y_idx = 0; y_idx < sdf.GetNumYCells(); y_idx++)
    {
      for (int64_t z_idx = 0; z_idx < sdf.GetNumZCells(); z_idx++)
      {
        const Eigen::Vector4d location
            = sdf.GridIndexToLocation(x_idx, y_idx, z_idx);
        const Eigen::Vector3d extrema = maxima_map.GetImmutable(x_idx, y_idx, z_idx).first;
        if (!std::isinf(extrema.x())
            && !std::isinf(extrema.y())
            && !std::isinf(extrema.z()))
        {
          visualization_msgs::Marker maxima_rep;
          // Populate the header
          maxima_rep.header.frame_id = "world";
          // Populate the options
          maxima_rep.ns = "maxima";
          maxima_rep.id = (int32_t)sdf.HashDataIndex(x_idx, y_idx, z_idx);
          maxima_rep.action = visualization_msgs::Marker::ADD;
          maxima_rep.lifetime = ros::Duration(0.0);
          maxima_rep.frame_locked = false;
          maxima_rep.pose
              = EigenHelpersConversions::EigenIsometry3dToGeometryPose(Eigen::Isometry3d::Identity());
          const double distance = (extrema - location.block<3, 1>(0, 0)).norm();
          if (distance > sdf.GetResolution())
          {
            maxima_rep.type = visualization_msgs::Marker::ARROW;
            maxima_rep.points.push_back(EigenHelpersConversions::EigenVector4dToGeometryPoint(location));
            maxima_rep.points.push_back(EigenHelpersConversions::EigenVector3dToGeometryPoint(extrema));
            maxima_rep.scale.x = sdf.GetResolution() * 0.06125;
            maxima_rep.scale.y = sdf.GetResolution() * 0.125;
            maxima_rep.scale.z = 0.0;
          }
          else
          {
            maxima_rep.type = visualization_msgs::Marker::SPHERE;
            maxima_rep.points.push_back(EigenHelpersConversions::EigenVector4dToGeometryPoint(location));
            maxima_rep.scale.x = sdf.GetResolution() * 0.125;
            maxima_rep.scale.y = sdf.GetResolution() * 0.125;
            maxima_rep.scale.z = sdf.GetResolution() * 0.125;
          }
          maxima_rep.color = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(1.0, 0.5, 0.0, 1.0);
          display_markers.markers.push_back(maxima_rep);
        }
      }
    }
  }
  display_fn(display_markers);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "compute_convex_segments_test");
  ros::NodeHandle nh;
  ros::Publisher display_pub
      = nh.advertise<visualization_msgs::MarkerArray>(
          "display_test_voxel_grid", 1, true);
  const std::function<void(const visualization_msgs::MarkerArray&)>& display_fn
      = [&] (const visualization_msgs::MarkerArray& markers)
  {
    display_pub.publish(markers);
  };
  test_compute_convex_segments(display_fn);
  ros::spin();
  return 0;
}
