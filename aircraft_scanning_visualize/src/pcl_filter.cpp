#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "pcl_filter.h"

using namespace asv3d;

WSPointCloudPtr PCLFilter::FilterPassThrough(const WSPointCloudPtr cloud, const std::string& field, double limit_min, double limit_max)
{
  if (cloud == nullptr)
    return nullptr;

  WSPointCloudPtr passCloud(new WSPointCloud());
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName(field);
  pass.setFilterLimits(limit_min, limit_max);
  // pass.setFilterLimitsNegative(true);
  pass.filter(*passCloud);
  return passCloud;
}

WSPointCloudPtr PCLFilter::FilterPCLPoint(const WSPointCloudPtr cloud, float leafSize)
{
  if (cloud == nullptr)
    return nullptr;

  WSPointCloudPtr voxelCloud(new WSPointCloud());
  pcl::VoxelGrid<WSPoint> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leafSize, leafSize, leafSize);
  sor.filter(*voxelCloud);
  return voxelCloud;
}

WSPointCloudPtr PCLFilter::FilterPCLPointSOR(const WSPointCloudPtr cloud, int neighbor, float thresh)
{
  if (cloud == nullptr)
    return nullptr;

  WSPointCloudPtr sorCloud(new WSPointCloud());
  pcl::StatisticalOutlierRemoval<WSPoint> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(neighbor);
  sor.setStddevMulThresh(thresh);
  sor.filter(*sorCloud);
  return sorCloud;
}
