#ifndef _ASV3D_PCL_SEGMENT_H_
#define _ASV3D_PCL_SEGMENT_H_

#include <vector>
#include "pcl_viewer.h"

namespace asv3d {

  class PCLSegment{
    public:
      PCLSegment(const WSPointCloudPtr srcCloud);
      ~PCLSegment();

      bool Compute(int& nSegment, std::vector<WSPointCloudPtr>& segments);

    private:
      WSPointCloudPtr Segmentation(const WSPointCloudPtr cloud, double threashold, WSPointCloudNormalPtr& normals);

    private:
      WSPointCloudPtr m_cloud;
  };


};

#endif //_ASV3D_PCL_SEGMENT_H_
