#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>

#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>

int main (int argc, char** argv)
{
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);


  if ( pcl::io::loadPLYFile<pcl::PointXYZ>("../bunny/data/bun000.ply", *cloud_source) == -1 )
  {
    PCL_ERROR ("loadPLYFile faild.");
    return (-1);
  }
  boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud_source, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud_source, source_color, "source");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source");
  viewer->initCameraParameters ();

  // orthographic (parallel) projection; same with pressing key 'o'
  viewer->getRenderWindow ()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(1);
  // reset camera; same with pressing key 'r'
  viewer->getRenderWindow ()->GetRenderers()->GetFirstRenderer()->ResetCamera ();


  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  exit(0);
  
  
  
  
  
  for (size_t i = 0; i < cloud_source->points.size (); ++i)
    std::cout << "    " <<
    cloud_source->points[i].x << " " << cloud_source->points[i].y << " " <<
    cloud_source->points[i].z << std::endl;

  *cloud_target = *cloud_source;
  std::cout << "size:" << cloud_target->points.size() << std::endl;
  for (size_t i = 0; i < cloud_source->points.size (); ++i)
    cloud_target->points[i].x = cloud_source->points[i].x + 0.7f;

  std::cout << "Transformed " << cloud_source->points.size () << " data points:" << std::endl;

  for (size_t i = 0; i < cloud_target->points.size (); ++i)
    std::cout << "    " << cloud_target->points[i].x << " " <<
    cloud_target->points[i].y << " " << cloud_target->points[i].z << std::endl;



  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_source);
  icp.setInputTarget(cloud_target);

  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  
  
  
  std::cout << "has converged:" << icp.hasConverged()
	    << " score: " << icp.getFitnessScore() << std::endl;
  
  std::cout << icp.getFinalTransformation() << std::endl;
  
  
  
  return (0);
}

