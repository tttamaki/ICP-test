#ifndef VISUALIZE_CORRESPONDENCES_H_
#define VISUALIZE_CORRESPONDENCES_H_

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ PointT;
void
visualize_correspondences (const PointCloudPtr points1, const PointCloudPtr keypoints1,
			   const PointCloudPtr points2, const PointCloudPtr keypoints2,
			   const std::vector<int> &correspondences
			   );
void
visualize_correspondences (const PointCloudPtr points1, const PointCloudPtr keypoints1,
			   const PointCloudPtr points2, const PointCloudPtr keypoints2,
			   const pcl::CorrespondencesPtr pCorrespondences
			   );

#endif
