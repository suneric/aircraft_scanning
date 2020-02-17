#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <vector>
#include <functional>
#include <boost/filesystem.hpp>
#include <pcl/io/ply_io.h>

#include "pcl_viewer.h"
#include "pcl_segment.h"
#include "pcl_filter.h"

using namespace asv3d;
using namespace std;

std::mutex mtx;

void PrintHelp();
int  ParseArguments(int argc, char** argv, std::string& dir, bool& save, bool& filter);
void UpdatePointCloud(PCLViewer* viewer, const std::string& dir)
{
  // add a new thread for spin the viewer
  //std::thread t(ViewerSpin, viewer);
  std::vector<std::string> files;
  WSPointCloudPtr cloud(new WSPointCloud());
  int i = 0;
  int all = 0;

  while (!viewer->IsStop())
  {
    std::vector<std::string> allfiles;
    boost::filesystem::directory_iterator itr(dir);
    for (; itr != boost::filesystem::directory_iterator(); ++itr)
    {
      if (boost::filesystem::is_regular_file(itr->status()));
        allfiles.push_back(itr->path().string());
    }
    all = allfiles.size();
    bool bNewCloud = false;
    for (const auto& file : allfiles)
    {
        i++;
        std::vector<std::string>::iterator end = files.end();
        if (std::find(files.begin(),files.end(),file) == files.end())
        {
            std::cout << "pcl == load " << i << "/" << all << " point cloud from " << file << std::endl;
            WSPointCloudPtr temp(new WSPointCloud());
            int res = pcl::io::loadPLYFile(file, *temp);
            if(res < 0)
               std::cout << "pcl == failed to load point cloud." << std::endl;

            mtx.lock();
            *cloud += *temp;
            mtx.unlock();

            bNewCloud = true;
            files.push_back(file);
        }
     }

     if (bNewCloud)
     {
       mtx.lock();
       viewer->Update(cloud);
       mtx.unlock();
     }

     // Sleep for 2 seconds for the ply file
     std::this_thread::sleep_for(std::chrono::seconds(2));
  }
}

int main(int argc, char** argv) try
{
  bool bSave = false;
  bool bFilter = false;
  std::string dir = "";
  int task = ParseArguments(argc, argv, dir, bSave, bFilter);
  PCLViewer viewer("point_cloud");
  if (task == 0)
  {
      PrintHelp();
  }
  else if (task == 1 || task == 2)
  {
    std::thread t(UpdatePointCloud, &viewer, dir);
    while(!viewer.IsStop())
        viewer.Spin();
    t.join();
  }
  else if (task == 3)
  {
    WSPointCloudPtr cloud = viewer.LoadPointCloud(dir);
    if (bFilter)
    {
      PCLFilter filter;
      // filter z < 0.02 and z > 10 point
      std::cout << "Filtering: pass through z out of [0.05,20]..." << std::endl;
      cloud = filter.FilterPassThrough(cloud,"z",0.05,20);
      /*
      remove outlier
      The number of neighbors to analyze for each point is set to 50,
      and the standard deviation multiplier to 1. What this means is
      that all points who have a distance larger than 1 standard
      deviation of the mean distance to the query point will be marked
      as outliers and removed.
      */
      // std::cout << "Filtering: outlier with 50 samples in 1 standard deviation..." << std::endl;
      // cloud = filter.FilterPCLPointSOR(cloud,50,1);
      // // downsampling with neighbor distance 0.1
      // std::cout << "Filtering: downsampling with 0.1 meter neighbor distance..." << std::endl;
      // cloud = filter.FilterPCLPoint(cloud,0.1);
    }

    PCLSegment segment(cloud);
    int nSegment = 1;
    std::vector<WSPointCloudPtr> segClouds;
    bool success = segment.Compute(nSegment, segClouds);
    if (!success)
    {
      std::cout << "Segmentation: Failed." << std::endl;
    }
    // else
    // {
    //   // create viewpoints
    // }

    viewer.Update(cloud);
    while(!viewer.IsStop())
        viewer.Spin();

  }
  if (bSave)
    viewer.SavePointCloud(viewer.PointCloud(),dir);
  return task;
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
  std::cout << "    1. view point cloud:   cmd [-view|-v] [file_directory]\n";
  std::cout << "    2. merge pointcloud:   cmd [-merge|-m] [file_directory]\n";
  std::cout << "    3. process pointcloud: cmd [-process|-p] [file_directory] [filter]\n";

}

int ParseArguments(int argc, char** argv, std::string& dir, bool& save, bool& filter)
{
  if (argc < 2)
    return 0;

  std::string task = std::string(argv[1]);
  if (task.compare("-v") == 0 || task.compare("-view") == 0)
  {
    dir = std::string(argv[2]);
    return 1;
  }
  else if (task.compare("-m") == 0 || task.compare("-merge") == 0)
  {
    save = true;
    dir = std::string(argv[2]);
    return 2;
  }
  else if (task.compare("-p") == 0 || task.compare("-process") == 0)
  {
    dir = std::string(argv[2]);
    if (argc > 3)
      filter = std::string(argv[3]).compare("1") == 0;
    if (argc > 4)
      save = std::string(argv[4]).compare("1") == 0;
    return 3;
  }
  else
  {
    return 0;
  }
}
