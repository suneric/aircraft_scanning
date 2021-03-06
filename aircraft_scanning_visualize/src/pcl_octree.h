#ifndef _ASV3D_PCL_OCTREE_H_
#define _ASV3D_PCL_OCTREE_H_

#include "pcl_viewer.h"
#include <pcl/octree/octree_search.h>

typedef pcl::octree::OctreePointCloudSearch<WSPoint> OctreeSearch;
typedef std::pair<Eigen::Vector3f, std::vector<Eigen::Vector3f> > VoxelNormals;

namespace asv3d {

  struct Voxel
  {
    Voxel(const WSPoint& point, double sideLen, int idx)
    {
      x = point.x;
      y = point.y;
      z = point.z;
      len = sideLen;
      index = idx;
    }

    Voxel(const Voxel& voxel)
    {
      x = voxel.x;
      y = voxel.y;
      z = voxel.z;
      len = voxel.len;
      index = voxel.index;
    }

    Voxel& operator=(const Voxel& voxel)
    {
      x = voxel.x;
      y = voxel.y;
      z = voxel.z;
      len = voxel.len;
      index = voxel.index;
      return *this;
    }

    WSPoint Centroid() const
    {
      WSPoint center;
      center.x = x;
      center.y = y;
      center.z = z;
      return center;
    }

    bool InVoxel(const WSPoint& pt) const
    {
      if (pt.x < x-0.5*len || pt.x > x+0.5*len)
        return false;
      if (pt.y < y-0.5*len || pt.y > y+0.5*len)
        return false;
      if (pt.z < z-0.5*len || pt.z > z+0.5*len)
        return false;
      return true;
    }

    double x;
    double y;
    double z;
    double len;
    int index;
  };

  struct VoxelMap
  {
    std::vector<Voxel> m_voxels;
    std::vector<int> m_indices;

    void Clear() {
      m_voxels.clear();
      m_indices.clear();
    }

    VoxelMap() { Clear(); }

    VoxelMap(const VoxelMap& vMap)
    {
      m_voxels.assign(vMap.m_voxels.begin(),vMap.m_voxels.end());
      m_indices.assign(vMap.m_indices.begin(),vMap.m_indices.end());
    }

    VoxelMap& operator=(const VoxelMap& vMap)
    {
      m_voxels.assign(vMap.m_voxels.begin(),vMap.m_voxels.end());
      m_indices.assign(vMap.m_indices.begin(),vMap.m_indices.end());
      return *this;
    }

    void AddVoxel(const WSPoint& centroid, double sideLen)
    {
      bool bExist = false;
      for (size_t i = 0; i < m_voxels.size(); ++i)
      {
        if (m_voxels[i].InVoxel(centroid))
        {
          bExist = true;
          break;
        }
      }

      if (!bExist)
      {
        int id = m_voxels.size();
        Voxel v(centroid, sideLen, id);
        m_voxels.push_back(v);
        m_indices.push_back(id);
      }
    }

    WSPoint GetVoxelCentroid(int index) const
    {
      Voxel v = m_voxels[index];
      return v.Centroid();
    }

    int VoxelIndices(std::vector<int>& indices) const
    {
      indices.assign(m_indices.begin(), m_indices.end());
      return m_indices.size();
    }

    int VoxelIndex(const WSPoint& centroid) const
    {
      for (size_t i = 0; i < m_voxels.size(); ++i)
      {
        if (m_voxels[i].InVoxel(centroid))
          return m_voxels[i].index;
      }
      return -1;
    }
  };

  // Octree representation of input point cloud
  class PCLOctree {
  public:
    PCLOctree(const WSPointCloudPtr& cloud, double resolution, int minVoxelPts=5);
    PCLOctree(const PCLOctree& tree);
    PCLOctree& operator=(const PCLOctree& tree);
    ~PCLOctree();

    int TreeDepth() const {return m_os->getTreeDepth();}
    double Resolution() const {return m_os->getResolution();}

    WSPointCloudPtr VoxelCentroidCloud() const;
    WSPointCloudNormalPtr VoxelAverageNormals(const Eigen::Vector3f& refNormal,
       const std::vector<double>& bbox,
       WSPointCloudPtr& voxelCentroid) const;

    void FindOutsidePolygons(std::vector<WSPointCloudPtr>& outsidePolygons) const;
    void VoxelOutsideCenters(std::vector<VoxelNormals>& outsideNormals) const;

    // voxel
    WSPoint VoxelCentroid(int index) const;
    double VoxelSideLength() const;
    int VoxelCount() const;
    int VoxelIndices(std::vector<int>& indices) const;
    int VoxelIndex(const WSPoint& centroid) const;

    int IntersectedOccupiedVoxels(const Eigen::Vector3f& origin, const Eigen::Vector3f& end) const;

    // int GetIntersectedVoxelCenters(const Eigen::Vector3f& point,
    //                                const Eigen::Vector3f& normal,
    //                                int max,
    //                                std::vector<WSPoint>& voxelCenters) const;

    int BoxSearch(const Eigen::Vector3f& minPt,
                  const Eigen::Vector3f& maxPt,
                  std::vector<int> indices) const;
  private:
    bool IsOutsideVoxel(const WSPoint& point, const Eigen::Vector3f& refNormal);
    bool EvaluateVoxelNormal(const WSPointCloudPtr cloud, const WSPoint& point, WSNormal& normal, const Eigen::Vector3f& refNormal) const;


  private:
    OctreeSearch* m_os;
    WSPointCloudPtr m_cloud;
    WSPointCloudPtr m_centroidCloud;
    VoxelMap m_voxelMap;
  };
};


#endif //_ASV3D_PCL_OCTREE_H_
