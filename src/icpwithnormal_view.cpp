#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
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

void
loadFile(const char* fileName,
	 pcl::PointCloud<pcl::PointXYZ> &cloud
)
{
  pcl::PolygonMesh mesh;
  
  if ( pcl::io::loadPolygonFile ( fileName, mesh ) == -1 )
  {
    PCL_ERROR ( "loadFile faild." );
    return;
  }
  else
    pcl::fromPCLPointCloud2<pcl::PointXYZ> ( mesh.cloud, cloud );
  
  // remove points having values of nan
  std::vector<int> index;
  pcl::removeNaNFromPointCloud ( cloud, cloud, index );
}

int main (int argc, char** argv)
{
  
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source ( new pcl::PointCloud<pcl::PointXYZ> () );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target ( new pcl::PointCloud<pcl::PointXYZ> () );
  
  {
    // load source
    loadFile ( argv[1], *cloud_source );
    // load target
    loadFile ( argv[2], *cloud_target );
  }
  
  
  // transformed source ---> target
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_trans ( new pcl::PointCloud<pcl::PointXYZ> () );
  cloud_source_trans = cloud_source;
  
  
  boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer ( new pcl::visualization::PCLVisualizer ("3D Viewer") );
  viewer->setBackgroundColor (0, 0, 0);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color ( cloud_source, 0, 255, 0 );
  viewer->addPointCloud<pcl::PointXYZ> (cloud_source, source_color, "source");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color ( cloud_target, 255, 255, 255 );
  viewer->addPointCloud<pcl::PointXYZ> ( cloud_target, target_color, "target");
  viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target" );
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_trans_color ( cloud_source_trans, 255, 0, 255 );
  viewer->addPointCloud<pcl::PointXYZ> ( cloud_source_trans, source_trans_color, "source trans" );
  viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source trans" );
  
  
  // orthographic (parallel) projection; same with pressing key 'o'
  viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection( 1 );
  
  viewer->resetCamera();
  
  
  
  // prepare could with normals
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_normals ( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target_normals ( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_trans_normals ( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );

  addNormal( cloud_source, cloud_source_normals );
  addNormal( cloud_target, cloud_target_normals );
  addNormal( cloud_source_trans, cloud_source_trans_normals );
  
  
  
  
  
  pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr icp ( new pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> () );
  icp->setMaximumIterations ( 1 );
  icp->setInputSource ( cloud_source_trans_normals ); // not cloud_source, but cloud_source_trans!
  icp->setInputTarget ( cloud_target_normals );
  
  
  
  while ( !viewer->wasStopped () )
  {
    // registration
    icp->align ( *cloud_source_trans_normals ); // use cloud with normals for ICP
    
    if ( icp->hasConverged() )
    {
      // use cloud without normals for visualizatoin
      pcl::transformPointCloud ( *cloud_source, *cloud_source_trans, icp->getFinalTransformation() );
      viewer->updatePointCloud ( cloud_source_trans, source_trans_color, "source trans" );
      std::cout << icp->getFitnessScore() << std::endl;
    }
    else
      std::cout << "Not converged." << std::endl;
    
    viewer->spinOnce();
  }
  
  
  return( 0 );
}

