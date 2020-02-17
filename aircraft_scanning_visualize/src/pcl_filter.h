#ifndef _ASV3D_PCL_FILTER_H_
#define _ASV3D_PCL_FILTER_H_

#include "pcl_viewer.h"

namespace asv3d {

  class PCLFilter {
    public:
      WSPointCloudPtr FilterPCLPoint(const WSPointCloudPtr cloud, float leafSize);
      WSPointCloudPtr FilterPassThrough(const WSPointCloudPtr cloud, const std::string& field, double limit_min, double limit_max);
      WSPointCloudPtr FilterPCLPointSOR(const WSPointCloudPtr cloud, int neighbor, float thresh);
  };

};

#endif //_ASV3D_PCL_FILTER_H_
