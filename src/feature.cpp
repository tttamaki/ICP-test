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
#include <pcl/features/shot_omp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>


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
  
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_trans ( new pcl::PointCloud<pcl::PointXYZ> () );

  // prepare could with normals
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_trans_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal> () );

  pcl::PointCloud<pcl::Normal>::Ptr source_normals ( new pcl::PointCloud<pcl::Normal> () );
  pcl::PointCloud<pcl::Normal>::Ptr target_normals ( new pcl::PointCloud<pcl::Normal> () );

  addNormal ( cloud_source, source_normals, cloud_source_normals );
  addNormal ( cloud_target, target_normals, cloud_target_normals );
  addNormal ( cloud_source, source_normals, cloud_source_trans_normals ); // dummy at this time
  
  

  
  
  
  float radius;
  {
    Eigen::Vector4f max_pt, min_pt;
    pcl::getMinMax3D ( *cloud_source, min_pt, max_pt );
    radius = (max_pt - min_pt).norm() / 50; // fixed radius (scale) for detector/descriptor
    std::cout << "scale: " << radius << std::endl;
  }
  
  
  
  { // registration
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints ( new pcl::PointCloud<pcl::PointXYZI> () );
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints ( new pcl::PointCloud<pcl::PointXYZI> () );
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ ( new pcl::PointCloud<pcl::PointXYZ> () );
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ ( new pcl::PointCloud<pcl::PointXYZ> () );
    
    
    { // Harris detector with normal
      std::cout << "detection" << std::endl;
      
      pcl::HarrisKeypoint3D<pcl::PointXYZRGBNormal,pcl::PointXYZI>::Ptr detector ( new pcl::HarrisKeypoint3D<pcl::PointXYZRGBNormal,pcl::PointXYZI> () );
      detector->setNonMaxSupression ( true );
      detector->setRadius ( radius );
      detector->setRadiusSearch ( radius );
      detector->setMethod ( pcl::HarrisKeypoint3D<pcl::PointXYZRGBNormal,pcl::PointXYZI>::CURVATURE ); // HARRIS, NOBLE, LOWE, TOMASI, CURVATURE 
      
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
    

    
#define useFPFH

#ifdef useFPFH
#define descriptorType pcl::FPFHSignature33
#else
#define descriptorType pcl::SHOT352
#endif

    pcl::PointCloud<descriptorType>::Ptr source_features ( new pcl::PointCloud<descriptorType> () );
    pcl::PointCloud<descriptorType>::Ptr target_features ( new pcl::PointCloud<descriptorType> () );
    
    { // descriptor
      std::cout << "description" << std::endl;
#ifdef useFPFH
      pcl::Feature<pcl::PointXYZ, descriptorType>::Ptr descriptor ( new pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, descriptorType> () ); 
#else
      pcl::Feature<pcl::PointXYZ, descriptorType>::Ptr descriptor ( new pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, descriptorType> () );
#endif      
      descriptor->setSearchMethod ( pcl::search::Search<pcl::PointXYZ>::Ptr ( new pcl::search::KdTree<pcl::PointXYZ>) );
      descriptor->setRadiusSearch ( radius*10 );
      
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
	correspondences[i] = -1; // -1 means no correspondence
	
	#ifdef useFPFH
	if ( isnan ( source_features->points[i].histogram[0] ) ) continue;
	#else
	if ( isnan ( source_features->points[i].descriptor[0] ) ) continue;
	#endif
	
	search_tree.nearestKSearch ( *source_features, i, 1, index, L2_distance );
	correspondences[i] = index[0];
	
      }
      
    }
    visualize_correspondences (cloud_source, source_keypointsXYZ,
			       cloud_target, target_keypointsXYZ,
			       correspondences);
    
    
    pcl::CorrespondencesPtr pCorrespondences ( new pcl::Correspondences );

    { // Refining matching by filtering out wrong correspondence
      std::cout << "refineing matching" << std::endl;
      
      int nCorrespondence = 0;
      for (int i = 0; i < correspondences.size(); i++)
	if ( correspondences[i] >= 0 ) nCorrespondence++; // do not count "-1" in correspondences

      pCorrespondences->resize ( nCorrespondence );
      for (int i = 0, j = 0; i < correspondences.size(); i++)
      {
	if ( correspondences[i] > 0 )
	{
	  (*pCorrespondences)[j].index_query = i;
	  (*pCorrespondences)[j].index_match = correspondences[i];
	  j++;
	  
	}
      }

      pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> refine;
      refine.setInputSource ( source_keypointsXYZ );
      refine.setInputTarget ( target_keypointsXYZ );
      refine.setInputCorrespondences ( pCorrespondences );
      refine.getCorrespondences ( *pCorrespondences );
    }
    visualize_correspondences (cloud_source, source_keypointsXYZ,
			       cloud_target, target_keypointsXYZ,
			       pCorrespondences);
    

    

    Eigen::Matrix4f transformation;
    
    { // Estimating rigid transformation
      std::cout << "Estimating transformation" << std::endl;

      pcl::registration::TransformationEstimationSVD< pcl::PointXYZ, pcl::PointXYZ > est;
      
      std::vector<int> source_index ( source_features->size() );
      for (int i = 0; i < source_features->size(); ++i) source_index[i] = i;
      
      
      est.estimateRigidTransformation ( *source_keypointsXYZ,
					*target_keypointsXYZ, *pCorrespondences,
					transformation );
      std::cout << transformation << std::endl;
      
      pcl::transformPointCloud ( *cloud_source, *cloud_source_trans, transformation );
      pcl::transformPointCloud ( *cloud_source_normals, *cloud_source_trans_normals, transformation );
      
    }
    
    
  }
  
  
  
  {
    // visualization
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
    
    viewer->spin ();
    
    
    
    
    { // addtional refinement wit ICP
      pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
      
      icp.setInputSource ( cloud_source_trans_normals );
      icp.setInputTarget ( cloud_target_normals );
      
      // registration
      icp.align ( *cloud_source_trans_normals );
      
      Eigen::Matrix4f transformation;
      if ( icp.hasConverged() )
      {
	pcl::transformPointCloud ( *cloud_source_trans, *cloud_source_trans, icp.getFinalTransformation() );
	viewer->updatePointCloud ( cloud_source_trans, source_trans_color, "source trans" );
      }
      else
	std::cout << "Not converged." << std::endl;
      
      viewer->spin ();
    }
  }
  
  
  return(0);
}

