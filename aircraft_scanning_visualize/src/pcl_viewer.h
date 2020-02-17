#ifndef _ASV3D_PCL_VIEWER_H_
#define _ASV3D_PCL_VIEWER_H_

#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PolygonMesh.h>

typedef pcl::PointXYZRGB WSPoint;
typedef pcl::PointCloud<WSPoint> WSPointCloud;
typedef WSPointCloud::Ptr WSPointCloudPtr;

typedef pcl::Normal WSNormal;
typedef pcl::PointCloud<WSNormal> WSPointCloudNormal;
typedef WSPointCloudNormal::Ptr WSPointCloudNormalPtr;

namespace asv3d
{

  class PCLViewer
  {
  public:
    PCLViewer(const std::string& title);
    ~PCLViewer();

    WSPointCloudPtr LoadPointCloud(const std::string& dir);
    bool SavePointCloud(const WSPointCloudPtr cloud, const std::string& dir);
    
    WSPointCloudPtr PointCloud();
    void AddCoordinate(const Eigen::Affine3f& transform, const std::string& name);
    void Update(const WSPointCloudPtr cloud, const WSPointCloudNormalPtr normal = nullptr);
    void UpdateMesh(const pcl::PolygonMesh& mesh);

    bool IsStop() const;
    void SpinOnce(double duration = 10);
    void Spin() const;

  private:
    pcl::visualization::PCLVisualizer* m_viewer;
    WSPointCloudPtr m_ptCloud;
  };

};

#endif //!_ASV3D_PCL_VIEWER_H_
