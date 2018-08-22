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
  const int64_t x_size = 100;
  const int64_t y_size = 100;
  const int64_t z_size = 50;
  const Eigen::Isometry3d origin_transform
      = Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Quaterniond(
          Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));
  sdf_tools::TaggedObjectCollisionMapGrid tocmap(origin_transform, "world", res, x_size, y_size, z_size, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(0.0, 0u));
  for (int64_t x_idx = 0; x_idx < tocmap.GetNumXCells(); x_idx++)
  {
    for (int64_t y_idx = 0; y_idx < tocmap.GetNumYCells(); y_idx++)
    {
      for (int64_t z_idx = 0; z_idx < tocmap.GetNumZCells(); z_idx++)
      {
        if ((x_idx < 10) || (y_idx < 10) || (x_idx >= tocmap.GetNumXCells() - 10) || (y_idx >= tocmap.GetNumYCells() - 10))
        {
          tocmap.SetValue(x_idx, y_idx, z_idx, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(1.0, 1u));
        }
        else if ((x_idx >= 40) && (y_idx >= 40) && (x_idx < 60) && (y_idx < 60))
        {
          tocmap.SetValue(x_idx, y_idx, z_idx, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(1.0, 2u));
        }
        if (((x_idx >= 45) && (x_idx < 55)) || ((y_idx >= 45) && (y_idx < 55)))
        {
          tocmap.SetValue(x_idx, y_idx, z_idx, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(0.0, 0u));
        }
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
  const double connected_threshold = 1.75;
  const uint32_t number_of_convex_segments_manual_border = tocmap.UpdateConvexSegments(connected_threshold, false);
  std::cout << "Identified " << number_of_convex_segments_manual_border
            << " convex segments via SDF->maxima map->connected components (no border added)"
            << std::endl;
  for (uint32_t object_id = 0u; object_id <= 4u; object_id++)
  {
    for (uint32_t convex_segment = 1u; convex_segment <= number_of_convex_segments_manual_border; convex_segment++)
    {
      visualization_msgs::Marker segment_marker = tocmap.ExportConvexSegmentForDisplay(object_id, convex_segment);
      if (segment_marker.points.size() > 0)
      {
        segment_marker.ns += "_no_border";
        display_markers.markers.push_back(segment_marker);
      }
    }
  }
  const uint32_t number_of_convex_segments_virtual_border = tocmap.UpdateConvexSegments(connected_threshold, true);
  std::cout << "Identified " << number_of_convex_segments_virtual_border
            << " convex segments via SDF->maxima map->connected components (virtual border added)"
            << std::endl;
  for (uint32_t object_id = 0u; object_id <= 4u; object_id++)
  {
    for (uint32_t convex_segment = 1u; convex_segment <= number_of_convex_segments_virtual_border; convex_segment++)
    {
      visualization_msgs::Marker segment_marker = tocmap.ExportConvexSegmentForDisplay(object_id, convex_segment);
      if (segment_marker.points.size() > 0)
      {
        segment_marker.ns += "_virtual_border";
        display_markers.markers.push_back(segment_marker);
      }
    }
  }
  const auto sdf_result
      = tocmap.ExtractSignedDistanceField(std::numeric_limits<float>::infinity(), std::vector<uint32_t>(), true, false);
  std::cout << "(no border) SDF extrema: " << PrettyPrint::PrettyPrint(sdf_result.second) << std::endl;
  const sdf_tools::SignedDistanceField& sdf = sdf_result.first;
  visualization_msgs::Marker sdf_marker = sdf.ExportForDisplay(1.0f);
  sdf_marker.id = 1;
  sdf_marker.ns = "environment_sdf_no_border";
  display_markers.markers.push_back(sdf_marker);
  const auto virtual_border_sdf_result
      = tocmap.ExtractSignedDistanceField(std::numeric_limits<float>::infinity(), std::vector<uint32_t>(), true, true);
  std::cout << "(virtual border) SDF extrema: " << PrettyPrint::PrettyPrint(virtual_border_sdf_result.second) << std::endl;
  const sdf_tools::SignedDistanceField& virtual_border_sdf = virtual_border_sdf_result.first;
  visualization_msgs::Marker virtual_border_sdf_marker = virtual_border_sdf.ExportForDisplay(1.0f);
  virtual_border_sdf_marker.id = 1;
  virtual_border_sdf_marker.ns = "environment_sdf_virtual_border";
  display_markers.markers.push_back(virtual_border_sdf_marker);
  // Make extrema markers
  const VoxelGrid::VoxelGrid<Eigen::Vector3d> maxima_map = virtual_border_sdf.ComputeLocalExtremaMap();
  for (int64_t x_idx = 0; x_idx < maxima_map.GetNumXCells(); x_idx++)
  {
    for (int64_t y_idx = 0; y_idx < maxima_map.GetNumYCells(); y_idx++)
    {
      for (int64_t z_idx = 0; z_idx < maxima_map.GetNumZCells(); z_idx++)
      {
        const Eigen::Vector4d location
            = maxima_map.GridIndexToLocation(x_idx, y_idx, z_idx);
        const Eigen::Vector3d extrema = maxima_map.GetImmutable(x_idx, y_idx, z_idx).first;
        if (!std::isinf(extrema.x())
            && !std::isinf(extrema.y())
            && !std::isinf(extrema.z()))
        {
          const double distance = (extrema - location.block<3, 1>(0, 0)).norm();
          if (distance < sdf.GetResolution())
          {
            visualization_msgs::Marker maxima_rep;
            // Populate the header
            maxima_rep.header.frame_id = "world";
            // Populate the options
            maxima_rep.ns = "extrema";
            maxima_rep.id = (int32_t)sdf.HashDataIndex(x_idx, y_idx, z_idx);
            maxima_rep.action = visualization_msgs::Marker::ADD;
            maxima_rep.lifetime = ros::Duration(0.0);
            maxima_rep.frame_locked = false;
            maxima_rep.pose.position = EigenHelpersConversions::EigenVector4dToGeometryPoint(location);
            maxima_rep.pose.orientation = EigenHelpersConversions::EigenQuaterniondToGeometryQuaternion(Eigen::Quaterniond::Identity());
            maxima_rep.type = visualization_msgs::Marker::SPHERE;
            maxima_rep.scale.x = sdf.GetResolution();
            maxima_rep.scale.y = sdf.GetResolution();
            maxima_rep.scale.z = sdf.GetResolution();
            maxima_rep.color = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(1.0, 0.5, 0.0, 1.0);
            display_markers.markers.push_back(maxima_rep);
          }
        }
        else
        {
          std::cout << "Encountered inf extrema @ (" << x_idx << "," << y_idx << "," << z_idx << ")" << std::endl;
        }
      }
    }
  }
  std::cout << "(0,0,0) " << PrettyPrint::PrettyPrint(virtual_border_sdf.GetGradient((int64_t)0, (int64_t)0, (int64_t)0, true)) << std::endl;
  std::cout << "(1,1,1) " << PrettyPrint::PrettyPrint(virtual_border_sdf.GetGradient((int64_t)1, (int64_t)1, (int64_t)1, true)) << std::endl;
  std::cout << "(2,2,2) " << PrettyPrint::PrettyPrint(virtual_border_sdf.GetGradient((int64_t)2, (int64_t)2, (int64_t)2, true)) << std::endl;
  std::cout << "(0,0,0) " << PrettyPrint::PrettyPrint(virtual_border_sdf.GetSmoothGradient((int64_t)0, (int64_t)0, (int64_t)0, res)) << std::endl;
  std::cout << "(1,1,1) " << PrettyPrint::PrettyPrint(virtual_border_sdf.GetSmoothGradient((int64_t)1, (int64_t)1, (int64_t)1, res)) << std::endl;
  std::cout << "(2,2,2) " << PrettyPrint::PrettyPrint(virtual_border_sdf.GetSmoothGradient((int64_t)2, (int64_t)2, (int64_t)2, res)) << std::endl;
  std::cout << "(0,0,0) " << PrettyPrint::PrettyPrint(virtual_border_sdf.GetAutoDiffGradient((int64_t)0, (int64_t)0, (int64_t)0)) << std::endl;
  std::cout << "(1,1,1) " << PrettyPrint::PrettyPrint(virtual_border_sdf.GetAutoDiffGradient((int64_t)1, (int64_t)1, (int64_t)1)) << std::endl;
  std::cout << "(2,2,2) " << PrettyPrint::PrettyPrint(virtual_border_sdf.GetAutoDiffGradient((int64_t)2, (int64_t)2, (int64_t)2)) << std::endl;
  std::cout << "(0,0,0) " << PrettyPrint::PrettyPrint(maxima_map.GetImmutable((int64_t)0, (int64_t)0, (int64_t)0).first) << std::endl;
  std::cout << "(1,1,1) " << PrettyPrint::PrettyPrint(maxima_map.GetImmutable((int64_t)1, (int64_t)1, (int64_t)1).first) << std::endl;
  std::cout << "(2,2,2) " << PrettyPrint::PrettyPrint(maxima_map.GetImmutable((int64_t)2, (int64_t)2, (int64_t)2).first) << std::endl;
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
