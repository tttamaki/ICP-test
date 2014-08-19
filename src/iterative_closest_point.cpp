#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>

#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>

int main (int argc, char** argv)
{
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
  if ( pcl::io::loadPLYFile<pcl::PointXYZ>("../bunny/data/bun000.ply", *cloud_source) == -1 )
  {
    PCL_ERROR ("loadPLYFile faild.");
    return (-1);
  }

  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);
  if ( pcl::io::loadPLYFile<pcl::PointXYZ>("../bunny/data/bun045.ply", *cloud_target) == -1 )
  {
    PCL_ERROR ("loadPLYFile faild.");
    return (-1);
  }

  std::vector<int> index;
  pcl::removeNaNFromPointCloud( *cloud_source, *cloud_source, index );
  pcl::removeNaNFromPointCloud( *cloud_target, *cloud_target, index );


  boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud_source, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud_source, source_color, "source");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud_target, 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud_target, target_color, "target");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target");

  viewer->initCameraParameters ();
  // orthographic (parallel) projection; same with pressing key 'o'
  viewer->getRenderWindow ()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(1);
  // reset camera; same with pressing key 'r'
  viewer->getRenderWindow ()->GetRenderers()->GetFirstRenderer()->ResetCamera ();


//   while (!viewer->wasStopped ())
//   {
//     viewer->spinOnce (100);
//     boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//   }

  

  
  
  
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_source);
  icp.setInputTarget(cloud_target);

  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  
  
  
  std::cout << "has converged:" << icp.hasConverged()
	    << " score: " << icp.getFitnessScore() << std::endl;
	    
  Eigen::Matrix4f transformation = icp.getFinalTransformation();
  std::cout << transformation << std::endl;
  
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_trans (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud( *cloud_source, *cloud_source_trans, transformation );

  

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  
  return (0);
}

