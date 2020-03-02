#ifndef _ASV3D_PCL_FILTER_H_
#define _ASV3D_PCL_FILTER_H_

#include "pcl_viewer.h"

namespace asv3d {

  class PCLFilter {
    public:
      WSPointCloudPtr FilterPCLPoint(const WSPointCloudPtr cloud, float leafSize);
      WSPointCloudPtr FilterPassThrough(const WSPointCloudPtr cloud, const std::string& field, double limit_min, double limit_max);
      WSPointCloudPtr FilterPCLPointSOR(const WSPointCloudPtr cloud, int neighbor, float thresh);

      WSPointCloudPtr SamplingSurfaceNormal(const WSPointCloudPtr cloud, unsigned int sample, float ratio, WSPointCloudNormalPtr& normals);
      // create sub sampling point cloud with number of sample
      bool RandomSampling(const WSPointCloudPtr cloud, unsigned int sample, std::vector<int>& indices);
      // create a uniform sampling point cloud with a search radius
      WSPointCloudPtr UniformSampling(const WSPointCloudPtr cloud, double radius);

      WSPointCloudPtr ExtractPoints(const WSPointCloudPtr cloud, const std::vector<int>& indices);
      WSPointCloudNormalPtr ExtractNormals(const WSPointCloudNormalPtr normal, const std::vector<int>& indices);
  };

};

#endif //_ASV3D_PCL_FILTER_H_
