#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>



#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>



void addNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals
)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals ( new pcl::PointCloud<pcl::Normal> );

  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree (new pcl::search::KdTree<pcl::PointXYZ>);
  searchTree->setInputCloud ( cloud );

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud ( cloud );
  normalEstimator.setSearchMethod ( searchTree );
  normalEstimator.setKSearch ( 15 );
  normalEstimator.compute ( *normals );

  pcl::concatenateFields( *cloud, *normals, *cloud_with_normals );
}



int main (int argc, char** argv)
{
  
  
  // load source
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source( new pcl::PointCloud<pcl::PointXYZ> );
  if ( pcl::io::loadPLYFile<pcl::PointXYZ>("../bunny/data/bun000.ply", *cloud_source) == -1 )
  {
    PCL_ERROR ("loadPLYFile faild.");
    return (-1);
  }
  
  // load target
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target( new pcl::PointCloud<pcl::PointXYZ> );
  if ( pcl::io::loadPLYFile<pcl::PointXYZ>("../bunny/data/bun045.ply", *cloud_target) == -1 )
  {
    PCL_ERROR ("loadPLYFile faild.");
    return (-1);
  }
  
  
  
  { // remove points with nan
    std::vector<int> index;
    pcl::removeNaNFromPointCloud( *cloud_source, *cloud_source, index );
    pcl::removeNaNFromPointCloud( *cloud_target, *cloud_target, index );
  }
  

  
  
  
  // transformed source ---> target
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_trans ( new pcl::PointCloud<pcl::PointXYZ> );
  cloud_source_trans = cloud_source;
  
  
  boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer ( new pcl::visualization::PCLVisualizer ("3D Viewer") );
  viewer->setBackgroundColor (0, 0, 0);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud_source, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud_source, source_color, "source");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud_target, 255, 255, 255);
  viewer->addPointCloud<pcl::PointXYZ> (cloud_target, target_color, "target");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_trans_color(cloud_source_trans, 255, 0, 255);
  viewer->addPointCloud<pcl::PointXYZ> (cloud_source_trans, source_trans_color, "source trans");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source trans");
  
  
  viewer->initCameraParameters ();
  // orthographic (parallel) projection; same with pressing key 'o'
  viewer->getRenderWindow ()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(1);
  // reset camera; same with pressing key 'r'
  viewer->getRenderWindow ()->GetRenderers()->GetFirstRenderer()->ResetCamera ();
  
  
  
  
  
  
  
    
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_trans_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  addNormal(cloud_source, cloud_source_normals);
  addNormal(cloud_target, cloud_target_normals);
  addNormal(cloud_source_trans, cloud_source_trans_normals);
  
  
  
  
  
  pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
  icp.setMaximumIterations (1);
  icp.setInputSource( cloud_source_trans_normals ); // not cloud_source, but cloud_source_trans!
  icp.setInputTarget( cloud_target_normals );
  
  
  
  while(!viewer->wasStopped ())
  {
    // registration
    icp.align( *cloud_source_trans_normals );
    
    if( icp.hasConverged() )
    {
      pcl::transformPointCloud (*cloud_source, *cloud_source_trans, icp.getFinalTransformation());
      viewer->updatePointCloud(cloud_source_trans, source_trans_color, "source trans");
      std::cout << icp.getFitnessScore() << std::endl;
    }
    else
      std::cout << "Not converged." << std::endl;
    
    viewer->spinOnce ();
  }
  
  
  return(0);
}

