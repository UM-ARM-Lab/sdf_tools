#include "arc_utilities/pretty_print.hpp"
#include "sdf_tools/collision_map.hpp"
#include "sdf_tools/sdf.hpp"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include <functional>
#include <arc_utilities/eigen_helpers_conversions.hpp>

void test_estimate_distance(
    const std::function<void(
      const visualization_msgs::MarkerArray&)>& display_fn)
{
  const double res = 1.0;
  const double size = 10.0;
  const Eigen::Isometry3d origin_transform
      = Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Quaterniond(
          Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ()));
  auto map = sdf_tools::CollisionMapGrid(origin_transform, "world", res, size, size, 1.0, sdf_tools::COLLISION_CELL(0.0));
  map.SetValue4d(origin_transform * Eigen::Vector4d(5.0, 5.0, 0.0, 1.0), sdf_tools::COLLISION_CELL(1.0));
  map.SetValue4d(origin_transform * Eigen::Vector4d(5.0, 6.0, 0.0, 1.0), sdf_tools::COLLISION_CELL(1.0));
  map.SetValue4d(origin_transform * Eigen::Vector4d(6.0, 5.0, 0.0, 1.0), sdf_tools::COLLISION_CELL(1.0));
  map.SetValue4d(origin_transform * Eigen::Vector4d(6.0, 6.0, 0.0, 1.0), sdf_tools::COLLISION_CELL(1.0));
  map.SetValue4d(origin_transform * Eigen::Vector4d(7.0, 7.0, 0.0, 1.0), sdf_tools::COLLISION_CELL(1.0));
  map.SetValue4d(origin_transform * Eigen::Vector4d(2.0, 2.0, 0.0, 1.0), sdf_tools::COLLISION_CELL(1.0));
  map.SetValue4d(origin_transform * Eigen::Vector4d(3.0, 2.0, 0.0, 1.0), sdf_tools::COLLISION_CELL(1.0));
  map.SetValue4d(origin_transform * Eigen::Vector4d(4.0, 2.0, 0.0, 1.0), sdf_tools::COLLISION_CELL(1.0));
  map.SetValue4d(origin_transform * Eigen::Vector4d(2.0, 3.0, 0.0, 1.0), sdf_tools::COLLISION_CELL(1.0));
  map.SetValue4d(origin_transform * Eigen::Vector4d(2.0, 4.0, 0.0, 1.0), sdf_tools::COLLISION_CELL(1.0));
  map.SetValue4d(origin_transform * Eigen::Vector4d(2.0, 7.0, 0.0, 1.0), sdf_tools::COLLISION_CELL(1.0));
  const std_msgs::ColorRGBA collision_color = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(1.0, 0.0, 0.0, 0.5);
  const std_msgs::ColorRGBA free_color = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(0.0, 0.0, 0.0, 0.0);
  const std_msgs::ColorRGBA unknown_color = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(0.0, 0.0, 0.0, 0.0);
  const auto map_marker = map.ExportForDisplay(collision_color, free_color, unknown_color);
  const auto sdf = map.ExtractSignedDistanceField(1e6).first;
  const auto sdf_marker = sdf.ExportForDisplay(0.05f);
  // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
  visualization_msgs::Marker estimated_distance_rep;
  // Populate the header
  estimated_distance_rep.header.frame_id = "world";
  // Populate the options
  estimated_distance_rep.ns = "estimated_distance_display";
  estimated_distance_rep.id = 1;
  estimated_distance_rep.type = visualization_msgs::Marker::CUBE_LIST;
  estimated_distance_rep.action = visualization_msgs::Marker::ADD;
  estimated_distance_rep.lifetime = ros::Duration(0.0);
  estimated_distance_rep.frame_locked = false;
  estimated_distance_rep.pose
      = EigenHelpersConversions::EigenIsometry3dToGeometryPose(sdf.GetOriginTransform());
  const double step = sdf.GetResolution() * 0.125 * 0.25;
  estimated_distance_rep.scale.x = sdf.GetResolution() * step;// * 0.125;
  estimated_distance_rep.scale.y = sdf.GetResolution() * step;// * 0.125;
  estimated_distance_rep.scale.z = sdf.GetResolution() * 0.95;// * 0.125;// * 0.125;
  // Add all the cells of the SDF to the message
  double min_distance = 0.0;
  double max_distance = 0.0;
  // Add colors for all the cells of the SDF to the message
  for (double x = 0; x < sdf.GetXSize(); x += step)
  {
    for (double y = 0; y < sdf.GetYSize(); y += step)
    {
      double z = 0.5;
      //for (double z = 0; z <= sdf.GetZSize(); z += step)
      {
        const Eigen::Vector4d point(x, y, z, 1.0);
        const Eigen::Vector4d point_in_grid_frame = origin_transform * point;
        // Update minimum/maximum distance variables
        const double distance
            = sdf.EstimateDistance4d(point_in_grid_frame).first;
        if (distance > max_distance)
        {
          max_distance = distance;
        }
        if (distance < min_distance)
        {
          min_distance = distance;
        }
      }
    }
  }
  std::cout << "Min dist " << min_distance << " Max dist " << max_distance << std::endl;
  for (double x = 0; x < sdf.GetXSize(); x += step)
  {
    for (double y = 0; y < sdf.GetYSize(); y += step)
    {
      double z = 0.5;
      //for (double z = 0; z <= sdf.GetZSize(); z += step)
      {
        const Eigen::Vector4d point(x, y, z, 1.0);
        const Eigen::Vector4d point_in_grid_frame = origin_transform * point;
        const double distance
            = sdf.EstimateDistance4d(point_in_grid_frame).first;
        if (distance >= 0.0)
        {
          const std_msgs::ColorRGBA new_color
              = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>
                ::InterpolateHotToCold(distance, 0.0, max_distance);
          estimated_distance_rep.colors.push_back(new_color);
        }
        else
        {
          const std_msgs::ColorRGBA new_color
              = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(1.0, 0.0, 1.0, 1.0);
          estimated_distance_rep.colors.push_back(new_color);
        }
        geometry_msgs::Point new_point;
        new_point.x = x;
        new_point.y = y;
        new_point.z = z;
        estimated_distance_rep.points.push_back(new_point);
      }
    }
  }
  visualization_msgs::MarkerArray markers;
  markers.markers = {map_marker, sdf_marker, estimated_distance_rep};
  display_fn(markers);
//  std::cout << std::setprecision(12) << sdf.EstimateDistance(6.4, 6.4, 6.4).first << std::endl;
//  std::cout << std::setprecision(12) << sdf.EstimateDistance(6.5, 6.5, 6.5).first << std::endl;
//  std::cout << std::setprecision(12) << sdf.EstimateDistance(6.5, 6.5, 6.6).first << std::endl;
//  std::cout << std::setprecision(12) << sdf.EstimateDistance(6.5, 6.6, 6.5).first << std::endl;
//  std::cout << std::setprecision(12) << sdf.EstimateDistance(6.6, 6.5, 6.5).first << std::endl;
//  std::cout << std::setprecision(12) << sdf.EstimateDistance(6.6, 6.6, 6.6).first << std::endl;
//  std::cout << std::setprecision(12) << sdf.EstimateDistance(6.6, 6.6, 6.7).first << std::endl;
//  std::cout << std::setprecision(12) << sdf.EstimateDistance(6.6, 6.7, 6.6).first << std::endl;
//  std::cout << std::setprecision(12) << sdf.EstimateDistance(6.7, 6.6, 6.6).first << std::endl;
//  std::cout << std::setprecision(12) << sdf.EstimateDistance(6.7, 6.7, 6.7).first << std::endl;
//  std::cout << "-----" << std::endl;
//  std::cout << std::setprecision(12) << sdf.EstimateDistance(5.8, 5.9, 5.7).first << std::endl;
//  std::cout << PrettyPrint::PrettyPrint(sdf.ProjectOutOfCollisionToMinimumDistance4d(Eigen::Vector4d(5.8, 5.9, 5.7, 1.0), 0.001, 0.06125)) << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "estimate_distance_test");
  ros::NodeHandle nh;
  ros::Publisher display_pub
      = nh.advertise<visualization_msgs::MarkerArray>(
          "display_test_voxel_grid", 1, true);
  const std::function<void(const visualization_msgs::MarkerArray&)>& display_fn
      = [&] (const visualization_msgs::MarkerArray& markers)
  {
    display_pub.publish(markers);
  };
  test_estimate_distance(display_fn);
  ros::spin();
  return 0;
}
