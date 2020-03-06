#include "pcl_octree.h"
#include "pcl_filter.h"
#include <math.h>

#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/features/normal_3d.h>

using namespace asv3d;

PCLOctree::PCLOctree(const WSPointCloudPtr& cloud, double resolution)
{
  // define a octree search
  m_cloud = cloud;
  m_os = new OctreeSearch(resolution);
  m_os->setInputCloud(m_cloud);
  m_os->defineBoundingBox();
  m_os->addPointsFromInputCloud();

  // get occupied voxel centroids
  m_centroidCloud = nullptr;
  m_voxelMap.Clear();

  pcl::octree::OctreePointCloud<WSPoint>::AlignedPointTVector centroids;
  int nVoxel = m_os->getOccupiedVoxelCenters(centroids);
  if (nVoxel > 0)
  {
    double voxelSideLen = VoxelSideLength();
    WSPointCloudPtr voxelCloud(new WSPointCloud);
    voxelCloud->points.resize(nVoxel);
    for (int i = 0; i < nVoxel; ++i)
    {
      voxelCloud->points[i] = centroids[i];
      m_voxelMap.AddVoxel(voxelCloud->points[i],voxelSideLen);
    }

    m_centroidCloud = voxelCloud;
  }
}

PCLOctree::~PCLOctree()
{
  delete m_os;
}

PCLOctree::PCLOctree(const PCLOctree& tree)
{
  m_os = tree.m_os;
  m_cloud = tree.m_cloud;
  m_centroidCloud = tree.m_centroidCloud;
  m_voxelMap = tree.m_voxelMap;
}

PCLOctree& PCLOctree::operator=(const PCLOctree& tree)
{
  m_os = tree.m_os;
  m_cloud = tree.m_cloud;
  m_centroidCloud = tree.m_centroidCloud;
  m_voxelMap = tree.m_voxelMap;
  return *this;
}

WSPoint PCLOctree::VoxelCentroid(int index) const
{
  return m_voxelMap.GetVoxelCentroid(index);
}

double PCLOctree::VoxelSideLength() const
{
  return sqrt(m_os->getVoxelSquaredSideLen());
}

int PCLOctree::VoxelCount() const
{
  return static_cast<int>(m_centroidCloud->points.size());
}

int PCLOctree::VoxelIndices(std::vector<int>& indices) const
{
  return m_voxelMap.VoxelIndices(indices);
}

int PCLOctree::VoxelIndex(const WSPoint& centroid) const
{
  return m_voxelMap.VoxelIndex(centroid);
}

WSPointCloudPtr PCLOctree::VoxelCentroidCloud() const
{
  return m_centroidCloud;
}

WSPointCloudNormalPtr PCLOctree::VoxelAverageNormals() const
{
  if (nullptr == m_centroidCloud)
    return nullptr;

  WSPointCloudNormalPtr normals(new WSPointCloudNormal);
  int nVoxel = m_centroidCloud->points.size();
  normals->points.resize(nVoxel);
  for (int i = 0; i < nVoxel; ++i)
  {
    WSPoint point = m_centroidCloud->points[i];
    WSNormal normal;
    EvaluateVoxelNormal(m_cloud, point, normal);
    normals->points[i].normal_x = normal.normal_x;
    normals->points[i].normal_y = normal.normal_y;
    normals->points[i].normal_z = normal.normal_z;
  }

  return normals;
}

// evaluete average voxel normal, return if fliped
bool PCLOctree::EvaluateVoxelNormal(const WSPointCloudPtr cloud, const WSPoint& point, WSNormal& normal) const
{
  bool bFlip = false;
  std::vector<int> voxelIndices;
  if (m_os->voxelSearch(point,voxelIndices))
  {
    // find a all point groued in the voxel and evaluate point normals
    PCLFilter filter;
    WSPointCloudPtr groupPt = filter.ExtractPoints(cloud,voxelIndices);
    WSPointCloudNormalPtr normals(new WSPointCloudNormal);
    pcl::NormalEstimation<WSPoint, WSNormal> ne;
    pcl::search::KdTree<WSPoint>::Ptr tree(new pcl::search::KdTree<WSPoint>());
    ne.setSearchMethod(tree);
    ne.setInputCloud(groupPt);
    ne.setKSearch(30);
    ne.compute(*normals);

    // average normal
    size_t num = normals->points.size();
    double nx=0.0,ny=0.0,nz=0.0;
    for (size_t i = 0; i < num; ++i)
    {
      WSNormal normal = normals->points[i];
      Eigen::Vector3f nmVec(normal.normal_x, normal.normal_y, normal.normal_z);
      Eigen::Vector3f nmUnit = nmVec.normalized();
      nx += nmUnit.x();
      ny += nmUnit.y();
      nz += nmUnit.z();
    }
    nx /= num;
    ny /= num;
    nz /= num;

    // flip normal based on voxel intersection
    Eigen::Vector3f pt(point.x,point.y,point.z);
    Eigen::Vector3f nm(nx,ny,nz);
    std::vector<int> intersectIndices;
    int nIntersectPositive = m_os->getIntersectedVoxelIndices(pt,nm,intersectIndices);
    int nInterSectNegative = m_os->getIntersectedVoxelIndices(pt,-nm,intersectIndices);
    if (nIntersectPositive > 1 && nInterSectNegative == 1)
    {
      nm = -nm;
      bFlip = true;
    }
    normal.normal_x = nm.x();
    normal.normal_y = nm.y();
    normal.normal_z = nm.z();
  }
  else
  {
    std::cout << "no search result" << std::endl;
  }
  return bFlip;
}

int PCLOctree::GetIntersectedVoxelCenters(const Eigen::Vector3f& point,
                                          const Eigen::Vector3f& normal,
                                          int max,
                                          std::vector<WSPoint>& voxelCenters) const
{
  pcl::octree::OctreePointCloud<WSPoint>::AlignedPointTVector intersects;
  int nRes = m_os->getIntersectedVoxelCenters(point, normal, intersects, max);
  voxelCenters.assign(intersects.begin(), intersects.end());
  return nRes;
}

int PCLOctree::BoxSearch(const Eigen::Vector3f& minPt,
              const Eigen::Vector3f& maxPt,
              std::vector<int> indices) const
{
  return m_os->boxSearch(minPt, maxPt, indices);
}

// void CreateViewpointsWithAverageNormal(const WSPointCloudPtr cloud, OctreeSearch os, double distance, std::vector<Eigen::Affine3f>& cameras)
// {
//   // find all voxel centroids and average normal of the voxel
//   PCLOctree octree;
//   WSPointCloudNormalPtr vNormal(new WSPointCloudNormal);
//   WSPointCloudPtr vCloud = octree.EvaluateNormals(cloud, os, vNormal);
//
//   // find all camera poses
//   std::vector<Eigen::Affine3f> cameraPoseVec;
//   for (size_t i = 0; i < vCloud->points.size(); ++i)
//   {
//     Eigen::Vector3f point(vCloud->points[i].x, vCloud->points[i].y, vCloud->points[i].z);
//     Eigen::Vector3f nm(vNormal->points[i].normal_x, vNormal->points[i].normal_y, vNormal->points[i].normal_z);
//     Eigen::Vector3f camera;
//     if (CameraPosition(point, nm, os.getResolution(), distance, camera))
//       cameras.push_back(octree.CameraPose(camera,-nm));
//   }
// }
//
// void CubeFaceCenter(const Eigen::Vector3f& center, double sideLen, std::vector<Eigen::Vector3f>& faceCenters)
// {
//   Eigen::Vector3f v1(center.x()-0.5*sideLen,center.y(),center.z());
//   Eigen::Vector3f v2(center.x()+0.5*sideLen,center.y(),center.z());
//   Eigen::Vector3f v3(center.x(),center.y()-0.5*sideLen,center.z());
//   Eigen::Vector3f v4(center.x(),center.y()+0.5*sideLen,center.z());
//   Eigen::Vector3f v5(center.x(),center.y(),center.z()-0.5*sideLen);
//   Eigen::Vector3f v6(center.x(),center.y(),center.z()+0.5*sideLen);
//   faceCenters.push_back(v1);
//   faceCenters.push_back(v2);
//   faceCenters.push_back(v3);
//   faceCenters.push_back(v4);
//   faceCenters.push_back(v5);
//   faceCenters.push_back(v6);
// }
//
// void CreateViewpointsWithVoxelCube(const WSPointCloudPtr cloud, OctreeSearch os, double distance, std::vector<Eigen::Affine3f>& cameras)
// {
//   PCLOctree octree;
//   double voxelLen = sqrt(os.getVoxelSquaredSideLen());
//   for (size_t i = 0; i < cloud->points.size(); ++i)
//   {
//     Eigen::Vector3f center(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
//     std::vector<Eigen::Vector3f> faceCenters;
//     CubeFaceCenter(center,voxelLen,faceCenters);
//     for (size_t j = 0; j < faceCenters.size(); j++)
//     {
//       Eigen::Vector3f nm = (faceCenters[j]-center).normalized();
//       std::vector<int> intersectIndices;
//       int nInterSectVoxel = os.getIntersectedVoxelIndices(center,nm,intersectIndices);
//       if (nInterSectVoxel <= 1)
//       {
//         Eigen::Vector3f camera;
//         if (CameraPosition(center, nm, os.getResolution(), distance, camera))
//           cameras.push_back(octree.CameraPose(camera,-nm));
//       }
//     }
//   }
// }