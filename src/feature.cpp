#include <iostream>
#include <algorithm>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/ppf.h>


#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>



// taken from https://github.com/PointCloudLibrary/pcl/blob/master/doc/tutorials/content/sources/iros2011/src/correspondence_viewer.cpp
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ PointT;
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



void addNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	       pcl::PointCloud<pcl::Normal>::Ptr normals,
	       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals
)
{
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
  
  
  
  // prepare could with normals
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  pcl::PointCloud<pcl::Normal>::Ptr source_normals ( new pcl::PointCloud<pcl::Normal> );
  pcl::PointCloud<pcl::Normal>::Ptr target_normals ( new pcl::PointCloud<pcl::Normal> );

  addNormal(cloud_source, source_normals, cloud_source_normals);
  addNormal(cloud_target, target_normals, cloud_target_normals);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_trans ( new pcl::PointCloud<pcl::PointXYZ> );

  
  
  { // registration
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints ( new pcl::PointCloud<pcl::PointXYZI> );
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints ( new pcl::PointCloud<pcl::PointXYZI> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ ( new pcl::PointCloud<pcl::PointXYZ> );
    
    
    { // Harris detector with normal
      std::cout << "detection" << std::endl;
      
      pcl::HarrisKeypoint3D<pcl::PointXYZRGBNormal,pcl::PointXYZI>::Ptr detector ( new pcl::HarrisKeypoint3D<pcl::PointXYZRGBNormal,pcl::PointXYZI> );
      detector->setNonMaxSupression ( true );
      detector->setRadius ( 0.01 );
      detector->setRadiusSearch ( 0.01 );
      detector->setMethod( pcl::HarrisKeypoint3D<pcl::PointXYZRGBNormal,pcl::PointXYZI>::HARRIS );
      
      detector->setInputCloud ( cloud_source_normals );
      detector->setSearchSurface ( cloud_source_normals );
      detector->compute ( *source_keypoints );
      cout << "number of source keypoints found: " << source_keypoints->points.size() << endl;
      source_keypointsXYZ->points.resize( source_keypoints->points.size() );
      pcl::copyPointCloud( *source_keypoints, *source_keypointsXYZ );
      
      detector->setInputCloud ( cloud_target_normals );
      detector->setSearchSurface ( cloud_target_normals );
      detector->compute ( *target_keypoints );
      cout << "number of target keypoints found: " << target_keypoints->points.size() << endl;
      source_keypointsXYZ->points.resize( target_keypoints->points.size() );
      pcl::copyPointCloud( *target_keypoints, *target_keypointsXYZ );
      
    }
    

    
// #define useFPFH

#ifdef useFPFH
#define descriptorType pcl::FPFHSignature33
#else
#define descriptorType pcl::PPFSignature
#endif

    pcl::PointCloud<descriptorType>::Ptr source_features ( new pcl::PointCloud<descriptorType> );
    pcl::PointCloud<descriptorType>::Ptr target_features ( new pcl::PointCloud<descriptorType> );
    
    { // descriptor
      std::cout << "description" << std::endl;
#ifdef useFPFH
      pcl::Feature<pcl::PointXYZ, descriptorType>::Ptr descriptor ( new pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, descriptorType> ); 
#else
      pcl::Feature<pcl::PointXYZ, descriptorType>::Ptr descriptor ( new pcl::PPFEstimation<pcl::PointXYZ, pcl::Normal, descriptorType> ); 
#endif      
      descriptor->setSearchMethod ( pcl::search::Search<pcl::PointXYZ>::Ptr ( new pcl::search::KdTree<pcl::PointXYZ>) );
      descriptor->setRadiusSearch ( 0.5 );
      
      pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, descriptorType>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, descriptorType> > ( descriptor );
      
      descriptor->setSearchSurface ( cloud_source );
      descriptor->setInputCloud ( source_keypointsXYZ );
      feature_from_normals->setInputNormals ( source_normals );
      descriptor->compute ( *source_features );
      
      descriptor->setSearchSurface ( cloud_target );
      descriptor->setInputCloud ( target_keypointsXYZ );
      feature_from_normals->setInputNormals ( target_normals );
      descriptor->compute ( *target_features );
      
    }
    

    
    std::vector<int> correspondences;
    
    { // Find matching with Kd-tree
      std::cout << "matching" << std::endl;

      correspondences.resize ( source_features->size() );
      
      pcl::KdTreeFLANN<descriptorType> search_tree;
      search_tree.setInputCloud ( target_features );
      
      std::vector<int> index(1);
      std::vector<float> L2_distance(1);
      for (int i = 0; i < source_features->size(); ++i)
      {
	if ( isnan(source_features->points[i].f1) ) continue;
	search_tree.nearestKSearch ( *source_features, i, 1, index, L2_distance );
	correspondences[i] = index[0];
      }
      
    }
    visualize_correspondences (cloud_source, source_keypointsXYZ,
			       cloud_target, target_keypointsXYZ,
			       correspondences);

    
    
    { // Estimating transformation
      
      pcl::registration::TransformationEstimationSVD< pcl::PointXYZ, pcl::PointXYZ > est;
      Eigen::Matrix4f transformation;
      
      std::vector<int> source_index(source_features->size());
      for (int i = 0; i < source_features->size(); ++i) source_index[i] = i;
      
      
      est.estimateRigidTransformation ( *source_keypointsXYZ, source_index,
					*target_keypointsXYZ, correspondences, transformation);
      std::cout << transformation << std::endl;
      
      pcl::transformPointCloud (*cloud_source, *cloud_source_trans, transformation);
      
    }
    
    

    
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

