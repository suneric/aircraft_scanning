#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/prosac.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "pcl_segment.h"

#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>

using namespace asv3d;

PCLSegment::PCLSegment(const WSPointCloudPtr srcCloud)
{
  m_cloud = srcCloud;
}

PCLSegment::~PCLSegment()
{

}

bool PCLSegment::Compute(int& nSegment, std::vector<WSPointCloudPtr>& segments)
{
  return true;
}

WSPointCloudPtr PCLSegment::Segmentation(const WSPointCloudPtr cloud, double threashold, WSPointCloudNormalPtr& normals)
{
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  //pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  //seg.setOptimizeCoefficients(true);
  // seg.setModelType(pcl::SACMODEL_PLANE);
  // seg.setMethodType(pcl::SAC_RANSAC);
  // seg.setDistanceThreshold(0.5);
  // seg.setMaxIterations(5000);
  // seg.setInputCloud(cloud);
  // seg.segment(*inliers, *coefficients);

  // Point normals
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  ne.setKSearch(50);
  ne.compute(*normals);

  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(0.5);
  seg.setDistanceThreshold(0.2);
  seg.setMaxIterations(5000);
  seg.setRadiusLimits(0,4);
  seg.setInputCloud(cloud);
  seg.setInputNormals(normals);
  seg.segment(*inliers, *coefficients);

  WSPointCloudPtr plane(new WSPointCloud());
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*plane);
  return cloud;
}
