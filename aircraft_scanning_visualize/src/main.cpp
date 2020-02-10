#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <boost/filesystem.hpp>
#include <functional>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include "pcl_viewer.h"

using namespace asv3d;
using namespace std;

void PrintHelp();
void ViewPointCloud(const std::string& strFile, double timeDuration);
WSPointCloudPtr FilterPCLPoint(const WSPointCloudPtr cloud, float leafSize);
WSPointCloudPtr FilterPCLPointSOR(const WSPointCloudPtr cloud, int neighbor, float thresh);

void UpdatePointCloud(PCLViewer* viewer, const std::string& dir)
{
  // add a new thread for spin the viewer
  //std::thread t(ViewerSpin, viewer);
  std::vector<std::string> files;
  WSPointCloudPtr cloud(new WSPointCloud());
  while (!viewer->IsStop())
  {
    std::vector<std::string> allfiles;
    boost::filesystem::directory_iterator itr(dir);
    for (; itr != boost::filesystem::directory_iterator(); ++itr)
    {
      if (boost::filesystem::is_regular_file(itr->status()));
        allfiles.push_back(itr->path().string());
    }

    bool bNewCloud = false;
    for (const auto& file : allfiles)
    {
        std::vector<std::string>::iterator end = files.end();
        if (std::find(files.begin(),files.end(),file) == files.end())
        {
            std::cout << "pcl == load point cloud from " << file << std::endl;
            WSPointCloudPtr temp(new WSPointCloud());
            int res = pcl::io::loadPLYFile(file, *temp);
            if(res < 0)
               std::cout << "pcl == failed to load point cloud." << std::endl;

            temp = FilterPCLPointSOR(temp, 50, 1.0);
            temp = FilterPCLPoint(temp, 0.001);
            *cloud += *temp;

            bNewCloud = true;
            files.push_back(file);
        }
     }

     if (bNewCloud)
         viewer->Update(cloud);

     // Sleep for 2 seconds for the ply file
     std::this_thread::sleep_for(std::chrono::seconds(2));
  }
}

int main(int argc, char** argv) try
{
  if (argc != 2)
  {
      PrintHelp();
      return 0;
  }

  std::string dir = std::string(argv[1]);
  PCLViewer viewer("point_cloud");
  std::thread t(UpdatePointCloud, &viewer, dir);
  while(!viewer.IsStop())
  {
      viewer.Spin();
  }
  t.join();
}
catch (const std::exception& e)
{
  std::cout << e.what() << std::endl;
  return -1;
}

void PrintHelp()
{
  std::cout << "This app is used for visualizing the point cloud by loading the '.ply' files in a directory.\n";
  std::cout << "Command line with providing a directory containing the .ply files:\n";
  std::cout << "    cmd [file_directory]\n";
}

WSPointCloudPtr FilterPCLPoint(const WSPointCloudPtr cloud, float leafSize)
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

WSPointCloudPtr FilterPCLPointSOR(const WSPointCloudPtr cloud, int neighbor, float thresh)
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
