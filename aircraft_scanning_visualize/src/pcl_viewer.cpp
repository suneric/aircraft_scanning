
#include <vector>
#include "pcl_viewer.h"
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <math.h>

using namespace asv3d;

PCLViewer::PCLViewer(const std::string& title)
{
  m_viewer = new pcl::visualization::PCLVisualizer(title);
  m_viewer->setBackgroundColor(255,255,255);
  m_viewer->addCoordinateSystem(1.0, "axis", 0);
  m_viewer->setSize(1024,748);
  //m_viewer->setCameraFieldOfView(0.8);
}

PCLViewer::~PCLViewer()
{
  m_viewer->close();
  delete m_viewer;
}

bool PCLViewer::IsStop() const
{
  return m_viewer->wasStopped();
}

void PCLViewer::Update(const WSPointCloudPtr cloud)
{
  if (!m_viewer->updatePointCloud(cloud, "cloud"))
  {
    m_viewer->addPointCloud<WSPoint>(cloud, "cloud");
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

    // set camera position
    Eigen::Vector4f centroid;
    Eigen::Vector4f min, max;
    pcl::compute3DCentroid(*cloud, centroid);
    pcl::getMinMax3D<pcl::PointXYZRGB>(*cloud, min, max);
    double dRadius = 0.5*std::sqrt((max[0]-min[0])*(max[0]-min[0])+(max[1]-min[1])*(max[1]-min[1])+(max[2]-min[2])*(max[2]-min[2]));
    m_viewer->setCameraPosition(dRadius, dRadius, 1, centroid[0], centroid[1], centroid[2], 0, 0, 1);
  }
}

void PCLViewer::UpdateMesh(const pcl::PolygonMesh& mesh)
{
  if (!m_viewer->updatePolygonMesh(mesh, "mesh"))
  {
    m_viewer->addPolygonMesh(mesh, "mesh");
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "mesh");
  }
}

void PCLViewer::AddCoordinate(const Eigen::Affine3f& transform, const std::string& name)
{
  m_viewer->removeCoordinateSystem(name);
  m_viewer->addCoordinateSystem(0.5, transform, name, 0);
}

void PCLViewer::SpinOnce(double duration)
{
  m_viewer->spinOnce(duration);
}

void PCLViewer::Spin() const
{
  m_viewer->spin();
}
