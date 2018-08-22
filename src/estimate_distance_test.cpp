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
  const auto sdf = map.ExtractSignedDistanceField(1e6, true, false).first;
  const auto sdf_marker = sdf.ExportForDisplay(0.05f);
  // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
  visualization_msgs::Marker distance_rep;
  // Populate the header
  distance_rep.header.frame_id = "world";
  // Populate the options
  distance_rep.ns = "estimated_distance_display";
  distance_rep.id = 1;
  distance_rep.type = visualization_msgs::Marker::CUBE_LIST;
  distance_rep.action = visualization_msgs::Marker::ADD;
  distance_rep.lifetime = ros::Duration(0.0);
  distance_rep.frame_locked = false;
  distance_rep.pose
      = EigenHelpersConversions::EigenIsometry3dToGeometryPose(sdf.GetOriginTransform());
  const double step = sdf.GetResolution() * 0.125 * 0.25;
  distance_rep.scale.x = sdf.GetResolution() * step;// * 0.125;
  distance_rep.scale.y = sdf.GetResolution() * step;// * 0.125;
  distance_rep.scale.z = sdf.GetResolution() * 0.95;// * 0.125;// * 0.125;
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
          distance_rep.colors.push_back(new_color);
        }
        else
        {
          const std_msgs::ColorRGBA new_color
              = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(1.0, 0.0, 1.0, 1.0);
          distance_rep.colors.push_back(new_color);
        }
        geometry_msgs::Point new_point;
        new_point.x = x;
        new_point.y = y;
        new_point.z = z;
        distance_rep.points.push_back(new_point);
      }
    }
  }
  visualization_msgs::MarkerArray markers;
  markers.markers = {map_marker, sdf_marker, distance_rep};
  // Make gradient markers
  for (int64_t x_idx = 0; x_idx < sdf.GetNumXCells(); x_idx++)
  {
    for (int64_t y_idx = 0; y_idx < sdf.GetNumYCells(); y_idx++)
    {
      for (int64_t z_idx = 0; z_idx < sdf.GetNumZCells(); z_idx++)
      {
        const Eigen::Vector4d location
            = sdf.GridIndexToLocation(x_idx, y_idx, z_idx);
        const std::vector<double> discrete_gradient
            = sdf.GetGradient4d(location, true);
        std::cout << "Discrete gradient " << PrettyPrint::PrettyPrint(discrete_gradient) << std::endl;
        const std::vector<double> smooth_gradient
            = sdf.GetSmoothGradient4d(location, sdf.GetResolution() * 0.125);
        std::cout << "Smooth gradient " << PrettyPrint::PrettyPrint(smooth_gradient) << std::endl;
        const std::vector<double> autodiff_gradient
            = sdf.GetAutoDiffGradient4d(location);
        std::cout << "Autodiff gradient " << PrettyPrint::PrettyPrint(autodiff_gradient) << std::endl;
        if (discrete_gradient.size() == 3)
        {
          const Eigen::Vector4d gradient_vector(discrete_gradient[0],
                                                discrete_gradient[1],
                                                discrete_gradient[2],
                                                0.0);
          visualization_msgs::Marker gradient_rep;
          // Populate the header
          gradient_rep.header.frame_id = "world";
          // Populate the options
          gradient_rep.ns = "discrete_gradient";
          gradient_rep.id = (int32_t)sdf.HashDataIndex(x_idx, y_idx, z_idx);
          gradient_rep.type = visualization_msgs::Marker::ARROW;
          gradient_rep.action = visualization_msgs::Marker::ADD;
          gradient_rep.lifetime = ros::Duration(0.0);
          gradient_rep.frame_locked = false;
          gradient_rep.pose
              = EigenHelpersConversions::EigenIsometry3dToGeometryPose(Eigen::Isometry3d::Identity());
          gradient_rep.points.push_back(EigenHelpersConversions::EigenVector4dToGeometryPoint(location));
          gradient_rep.points.push_back(EigenHelpersConversions::EigenVector4dToGeometryPoint(location + gradient_vector));
          gradient_rep.scale.x = sdf.GetResolution() * 0.06125;
          gradient_rep.scale.y = sdf.GetResolution() * 0.125;
          gradient_rep.scale.z = 0.0;
          gradient_rep.color = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(1.0, 0.5, 0.0, 1.0);
          markers.markers.push_back(gradient_rep);
        }
        if (smooth_gradient.size() == 3)
        {
          const Eigen::Vector4d gradient_vector(smooth_gradient[0],
                                                smooth_gradient[1],
                                                smooth_gradient[2],
                                                0.0);
          visualization_msgs::Marker gradient_rep;
          // Populate the header
          gradient_rep.header.frame_id = "world";
          // Populate the options
          gradient_rep.ns = "smooth_gradient";
          gradient_rep.id = (int32_t)sdf.HashDataIndex(x_idx, y_idx, z_idx);
          gradient_rep.type = visualization_msgs::Marker::ARROW;
          gradient_rep.action = visualization_msgs::Marker::ADD;
          gradient_rep.lifetime = ros::Duration(0.0);
          gradient_rep.frame_locked = false;
          gradient_rep.pose
              = EigenHelpersConversions::EigenIsometry3dToGeometryPose(Eigen::Isometry3d::Identity());
          gradient_rep.points.push_back(EigenHelpersConversions::EigenVector4dToGeometryPoint(location));
          gradient_rep.points.push_back(EigenHelpersConversions::EigenVector4dToGeometryPoint(location + gradient_vector));
          gradient_rep.scale.x = sdf.GetResolution() * 0.06125;
          gradient_rep.scale.y = sdf.GetResolution() * 0.125;
          gradient_rep.scale.z = 0.0;
          gradient_rep.color = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(0.0, 0.5, 1.0, 1.0);
          markers.markers.push_back(gradient_rep);
        }
        if (autodiff_gradient.size() == 3)
        {
          const Eigen::Vector4d gradient_vector(autodiff_gradient[0],
                                                autodiff_gradient[1],
                                                autodiff_gradient[2],
                                                0.0);
          visualization_msgs::Marker gradient_rep;
          // Populate the header
          gradient_rep.header.frame_id = "world";
          // Populate the options
          gradient_rep.ns = "autodiff_gradient";
          gradient_rep.id = (int32_t)sdf.HashDataIndex(x_idx, y_idx, z_idx);
          gradient_rep.type = visualization_msgs::Marker::ARROW;
          gradient_rep.action = visualization_msgs::Marker::ADD;
          gradient_rep.lifetime = ros::Duration(0.0);
          gradient_rep.frame_locked = false;
          gradient_rep.pose
              = EigenHelpersConversions::EigenIsometry3dToGeometryPose(Eigen::Isometry3d::Identity());
          gradient_rep.points.push_back(EigenHelpersConversions::EigenVector4dToGeometryPoint(location));
          gradient_rep.points.push_back(EigenHelpersConversions::EigenVector4dToGeometryPoint(location + gradient_vector));
          gradient_rep.scale.x = sdf.GetResolution() * 0.06125;
          gradient_rep.scale.y = sdf.GetResolution() * 0.125;
          gradient_rep.scale.z = 0.0;
          gradient_rep.color = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(0.5, 0.0, 1.0, 1.0);
          markers.markers.push_back(gradient_rep);
        }
      }
    }
  }
  display_fn(markers);
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
