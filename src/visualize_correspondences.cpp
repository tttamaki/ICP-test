//
// taken from https://github.com/PointCloudLibrary/pcl/blob/master/doc/tutorials/content/sources/iros2011/src/correspondence_viewer.cpp
//

#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

#include "visualize_correspondences.h"


void
visualize_correspondences (const PointCloudPtr points1, const PointCloudPtr keypoints1,
			   const PointCloudPtr points2, const PointCloudPtr keypoints2,
			   const std::vector<int> &correspondences /*,
			   const std::vector<float> &correspondence_scores, int max_to_display */)
{
  // We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
  // by shifting one to the left and the other to the right. Then we'll draw lines between the corresponding points
  // Create some new point clouds to hold our transformed data
  PointCloudPtr points_left (new PointCloud);
  PointCloudPtr keypoints_left (new PointCloud);
  PointCloudPtr points_right (new PointCloud);
  PointCloudPtr keypoints_right (new PointCloud);
  // Shift the first clouds' points to the left
  //const Eigen::Vector3f translate (0.0, 0.0, 0.3);
  const Eigen::Vector3f translate (0.4, 0.0, 0.0);
  const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
  pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);
  pcl::transformPointCloud (*keypoints1, *keypoints_left, -translate, no_rotation);
  // Shift the second clouds' points to the right
  pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);
  pcl::transformPointCloud (*keypoints2, *keypoints_right, translate, no_rotation);
  // Add the clouds to the visualizer
  pcl::visualization::PCLVisualizer vis;
  vis.addPointCloud (points_left, "points_left");
  vis.addPointCloud (points_right, "points_right");
//   // Compute the weakest correspondence score to display
//   std::vector<float> temp (correspondence_scores);
//   std::sort (temp.begin (), temp.end ());
//   if (max_to_display >= temp.size ())
//     max_to_display = temp.size () - 1;
//   float threshold = temp[max_to_display];
//   std::cout << max_to_display << std::endl;
  // Draw lines between the best corresponding points
  for (size_t i = 0; i < keypoints_left->size (); ++i)
  {
//     if (correspondence_scores[i] > threshold)
//     {
//       continue; // Don't draw weak correspondences
//     }
    if ( correspondences[i] < 0) continue;
    // Get the pair of points
    const PointT & p_left = keypoints_left->points[i];
    const PointT & p_right = keypoints_right->points[correspondences[i]];
    // Generate a random (bright) color
    double r = (rand() % 100);
    double g = (rand() % 100);
    double b = (rand() % 100);
    double max_channel = std::max (r, std::max (g, b));
    r /= max_channel;
    g /= max_channel;
    b /= max_channel;
    // Generate a unique string for each line
    std::stringstream ss ("line");
    ss << i;
    // Draw the line
    vis.addLine (p_left, p_right, r, g, b, ss.str ());
  }
  vis.resetCamera ();
  vis.spin ();
}
void
visualize_correspondences (const PointCloudPtr points1, const PointCloudPtr keypoints1,
			   const PointCloudPtr points2, const PointCloudPtr keypoints2,
			   const pcl::CorrespondencesPtr pCorrespondences
			   /* std::vector<int> &correspondences ,
			   const std::vector<float> &correspondence_scores, int max_to_display */)
{
  // We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
  // by shifting one to the left and the other to the right. Then we'll draw lines between the corresponding points
  // Create some new point clouds to hold our transformed data
  PointCloudPtr points_left (new PointCloud);
  PointCloudPtr keypoints_left (new PointCloud);
  PointCloudPtr points_right (new PointCloud);
  PointCloudPtr keypoints_right (new PointCloud);
  // Shift the first clouds' points to the left
  //const Eigen::Vector3f translate (0.0, 0.0, 0.3);
  const Eigen::Vector3f translate (0.4, 0.0, 0.0);
  const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
  pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);
  pcl::transformPointCloud (*keypoints1, *keypoints_left, -translate, no_rotation);
  // Shift the second clouds' points to the right
  pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);
  pcl::transformPointCloud (*keypoints2, *keypoints_right, translate, no_rotation);
  // Add the clouds to the visualizer
  pcl::visualization::PCLVisualizer vis;
  vis.addPointCloud (points_left, "points_left");
  vis.addPointCloud (points_right, "points_right");
//   // Compute the weakest correspondence score to display
//   std::vector<float> temp (correspondence_scores);
//   std::sort (temp.begin (), temp.end ());
//   if (max_to_display >= temp.size ())
//     max_to_display = temp.size () - 1;
//   float threshold = temp[max_to_display];
//   std::cout << max_to_display << std::endl;
  // Draw lines between the best corresponding points
  for (size_t i = 0; i < pCorrespondences->size (); ++i)
  {
//     if (correspondence_scores[i] > threshold)
//     {
//       continue; // Don't draw weak correspondences
//     }
    // Get the pair of points
    const PointT & p_left = keypoints_left->points[(*pCorrespondences)[i].index_query];
    const PointT & p_right = keypoints_right->points[(*pCorrespondences)[i].index_match];
    // Generate a random (bright) color
    double r = (rand() % 100);
    double g = (rand() % 100);
    double b = (rand() % 100);
    double max_channel = std::max (r, std::max (g, b));
    r /= max_channel;
    g /= max_channel;
    b /= max_channel;
    // Generate a unique string for each line
    std::stringstream ss ("line");
    ss << i;
    // Draw the line
    vis.addLine (p_left, p_right, r, g, b, ss.str ());
  }
  vis.resetCamera ();
  vis.spin ();
}
