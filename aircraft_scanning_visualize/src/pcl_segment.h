#ifndef _ASV3D_PCL_SEGMENT_H_
#define _ASV3D_PCL_SEGMENT_H_

#include <vector>
#include "pcl_viewer.h"

namespace asv3d {

  class PCLSegment{
    public:
      PCLSegment(const WSPointCloudPtr srcCloud);
      ~PCLSegment();

      WSPointCloudPtr PointCloud();
      bool Compute(std::vector<WSPointCloudPtr>& segments);
      bool ComputeNormals(WSPointCloudNormalPtr normals);

    private:
      bool CylinderSegment(double threshold, double radius, std::vector<WSPointCloudPtr>& segments);
      bool PlaneSegment(double threshold, std::vector<WSPointCloudPtr>& segments);
      bool RegionGrowingSegment(double threashold, std::vector<WSPointCloudPtr>& segments);
      bool EculideanSegment(double tolerance, std::vector<WSPointCloudPtr>& segments);

    private:
      WSPointCloudPtr m_cloud;
  };


};

#endif //_ASV3D_PCL_SEGMENT_H_
