#include <iostream>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/common/centroid.h>
#include <pcl/console/parse.h>
#include <boost/random.hpp>

enum methods {
  SVD,
  DQ,
  LM
};


int main (int argc, char** argv)
{
  
  int method = SVD;
  pcl::console::parse_argument (argc, argv, "-m", method);
  std::cout << "method: ";
  switch (method) {
  case SVD: std::cout << "SVD"; break;
  case DQ:  std::cout << "DQ";  break;
  case LM:  std::cout << "LM";  break;
  default: std::cout << "undefined. ERROR" << std::endl; exit(0);
  }
  std::cout << std::endl;

  bool use_scale = false;
  pcl::console::parse_argument (argc, argv, "-s", use_scale);
  std::cout << "use scale: " << (use_scale ? "true" : "false") << std::endl;
  if (use_scale) {
      std::cout << "forse SVD." << std::endl;
      method = SVD;
  }

  bool use_rand = false;
  pcl::console::parse_argument (argc, argv, "-r", use_rand);
  std::cout << "use random seed: " << (use_rand ? "true" : "false") << std::endl;

  
  
  boost::random::mt19937 gen; // alternative to rand()
  if (use_rand)
    gen.seed( std::time(0) ); // random seed with current time in second
  else
    gen.seed( 0 ); // fixed seed

  boost::random::uniform_real_distribution<float> frand( 1.0, 3.2 ); // random gen between 1.0 and 3.2
  
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source ( new pcl::PointCloud<pcl::PointXYZ> () );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target ( new pcl::PointCloud<pcl::PointXYZ> () );
  
  // create random source point cloud
  for (int i = 0; i < 1000; i++) {
    cloud_source->push_back (pcl::PointXYZ (frand(gen), frand(gen), frand(gen) ));
  }
  

  // create random transformation: R and T
  Eigen::Affine3f transformation_true;
  {
    // random rotation matrix
    Eigen::Vector3f axis;
    axis.setRandom().normalize();
    float angle = frand( gen );
    Eigen::Affine3f R ( Eigen::AngleAxis<float> ( angle, axis ) );
    
    // random translation vector
    Eigen::Translation3f T ( frand(gen), frand(gen), frand(gen) );

    std::cout << "true R" << std::endl << R.matrix() << std::endl
              << "true T" << std::endl << T .vector() << std::endl;

    if ( use_scale )
    {
      float scale = frand( gen );
      R.matrix().topLeftCorner(3,3) *= scale;
      std::cout << "true sR" << std::endl << R.matrix() << std::endl
                << "true scale " << scale << std::endl;
    }
    
    // R and T
    transformation_true = T * R ; // shoul be in this order if you mean (Rx + T).   If R*T, then R(x+t) !

  }
  std::cout << "true transformation" << std::endl << transformation_true.matrix() << std::endl;

  
  // create target point cloud
  pcl::transformPointCloud ( *cloud_source, *cloud_target, transformation_true );
    
  boost::shared_ptr< pcl::registration::TransformationEstimation< pcl::PointXYZ, pcl::PointXYZ > > estPtr;
  if ( use_scale )
    // estimator of R and T along with scale
    estPtr.reset ( new pcl::registration::TransformationEstimationSVDScale < pcl::PointXYZ, pcl::PointXYZ > () );
  else 
    // estimator of R and T
    switch (method) {
    case SVD:
      estPtr.reset ( new pcl::registration::TransformationEstimationSVD < pcl::PointXYZ, pcl::PointXYZ > () );
      break;
    case DQ:
      estPtr.reset ( new pcl::registration::TransformationEstimationDualQuaternion < pcl::PointXYZ, pcl::PointXYZ > () );
      break;
    case LM:
      estPtr.reset ( new pcl::registration::TransformationEstimationLM < pcl::PointXYZ, pcl::PointXYZ > () );
      break;
    }

    
  Eigen::Affine3f transformation_est;
  estPtr->estimateRigidTransformation ( *cloud_source,
                                        *cloud_target,
                                        transformation_est.matrix() );
  
  if ( use_scale ) {
    Eigen::Matrix3f R = transformation_est.matrix().topLeftCorner(3,3);
    std::cout << "estimated scale " << std::sqrt( (R.transpose() * R).trace() / 3.0 ) << std::endl;
  }
  std::cout << "estimated transformation " << std::endl << transformation_est.matrix()  << std::endl;
  
  return ( 0 );
}

