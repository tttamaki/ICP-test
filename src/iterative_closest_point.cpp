#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
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
  
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source( new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target( new pcl::PointCloud<pcl::PointXYZ> );
  
  {  
    pcl::PolygonMesh mesh;
    
    // load source
    if ( pcl::io::loadPolygonFilePLY(argv[1], mesh) == -1 )
    {
      PCL_ERROR ("loadPLYFile faild.");
      return (-1);
    }
    else
      pcl::fromPCLPointCloud2<pcl::PointXYZ>(mesh.cloud, *cloud_source);
    
    
    // load target
    if ( pcl::io::loadPolygonFilePLY(argv[2], mesh) == -1 )
    {
      PCL_ERROR ("loadPLYFile faild.");
      return (-1);
    }
    else
      pcl::fromPCLPointCloud2<pcl::PointXYZ>(mesh.cloud, *cloud_target);
  }

  
  
  { // remove points with nan
    std::vector<int> index;
    pcl::removeNaNFromPointCloud( *cloud_source, *cloud_source, index );
    pcl::removeNaNFromPointCloud( *cloud_target, *cloud_target, index );
  }
  
  
  // transformed source = target
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_trans ( new pcl::PointCloud<pcl::PointXYZ> );


  { // ICP registration
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    
    icp.setInputSource( cloud_source );
    icp.setInputTarget( cloud_target );
    
    // registration
    icp.align( *cloud_source_trans );
    
    
    if( icp.hasConverged() )
    {
      std::cout << "Converged. score =" << icp.getFitnessScore() << std::endl;
      
      // 4x4 transformation matrix
      Eigen::Matrix4f transformation = icp.getFinalTransformation();
      std::cout << transformation << std::endl;
    }
    else
      std::cout << "Not converged." << std::endl;
  }
  
  
  
  { // visualization
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
    
    
//     viewer->initCameraParameters ();
    // orthographic (parallel) projection; same with pressing key 'o'
    viewer->getRenderWindow ()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(1);

    viewer->resetCamera();
    
    viewer->spin ();

    
  }
  
  return(0);
}

