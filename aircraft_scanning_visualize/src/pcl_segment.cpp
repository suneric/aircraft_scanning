#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/prosac.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

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

WSPointCloudPtr PCLSegment::PointCloud()
{
  return m_cloud;
}

bool PCLSegment::Compute(std::vector<WSPointCloudPtr>& segments)
{
  if (m_cloud == nullptr)
    return false;
  CylinderSegment(0.3, 4.0, segments);
  // PlaneSegment(0.5,segments);
  RegionGrowingSegment(5.5/180*M_PI,segments);
  // EculideanSegment(0.1,segments);
  return true;
}

bool PCLSegment::CylinderSegment(double threshold, double radius, std::vector<WSPointCloudPtr>& segments)
{
  if (m_cloud == nullptr)
    return false;

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Point normals
  WSPointCloudNormalPtr normals(new WSPointCloudNormal);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  ne.setSearchMethod(tree);
  ne.setInputCloud(m_cloud);
  ne.setKSearch(50);
  ne.compute(*normals);

  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(0.5);
  seg.setDistanceThreshold(threshold);
  seg.setMaxIterations(1000);
  seg.setRadiusLimits(0,radius);
  seg.setInputCloud(m_cloud);
  seg.setInputNormals(normals);
  seg.segment(*inliers, *coefficients);
  // extact cylinder
  WSPointCloudPtr cylinder(new WSPointCloud());
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(m_cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cylinder);

  segments.push_back(cylinder);

  // update rest of the m_cloud
  extract.setNegative(true);
  extract.filter(*m_cloud);

  return true;
}

bool PCLSegment::PlaneSegment(double threshold, std::vector<WSPointCloudPtr>& segments)
{
  if (m_cloud == nullptr)
    return false;

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(threshold);
  seg.setMaxIterations(1000);
  seg.setInputCloud(m_cloud);
  seg.segment(*inliers, *coefficients);
  // extact plane
  WSPointCloudPtr plane(new WSPointCloud());
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(m_cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*plane);

  segments.push_back(plane);

  // update rest of the m_cloud
  extract.setNegative(true);
  extract.filter(*m_cloud);
}

bool PCLSegment::RegionGrowingSegment(double threshold, std::vector<WSPointCloudPtr>& segments)
{
  if (m_cloud == nullptr)
    return false;

  // Point normals
  WSPointCloudNormalPtr normals(new WSPointCloudNormal);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  ne.setSearchMethod(tree);
  ne.setInputCloud(m_cloud);
  ne.setKSearch(50);
  ne.compute(*normals);

  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
  reg.setMinClusterSize(100);
  reg.setMaxClusterSize(10000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(50);
  reg.setInputCloud(m_cloud);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(threshold);
  reg.setCurvatureThreshold(1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract(clusters);
  for (int i = 0; i < clusters.size(); ++i)
  {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    inliers->indices = clusters[i].indices;
    WSPointCloudPtr segment(new WSPointCloud());
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(m_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*segment);
    segments.push_back(segment);
  }

  m_cloud = reg.getColoredCloud();

  return true;
}

bool PCLSegment::EculideanSegment(double tolerance, std::vector<WSPointCloudPtr>& segments)
{
  if (m_cloud == nullptr)
    return false;

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  tree->setInputCloud(m_cloud);

  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<WSPoint> ec;
  ec.setClusterTolerance(tolerance);
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(m_cloud);
  ec.extract(clusters);
  for (int i = 0; i < clusters.size(); ++i)
  {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    inliers->indices = clusters[i].indices;
    WSPointCloudPtr segment(new WSPointCloud());
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(m_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*segment);
    segments.push_back(segment);
  }
  return true;
}

bool PCLSegment::ComputeNormals(WSPointCloudNormalPtr normals)
{
  if (m_cloud == nullptr)
    return false;

  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  ne.setSearchMethod(tree);
  ne.setInputCloud(m_cloud);
  // ne.setKSearch(10);
  ne.setRadiusSearch(0.5);
  ne.compute(*normals);

  return true;
}
