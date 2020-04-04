#include <iostream>
#include <math.h>
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
#include "pcl_octree.h"
#include "pcl_viewpoint.h"

using namespace asv3d;
using namespace std;

// use static mutex to lock thread when operating the pcl visuliazer
static std::mutex mtx;

void PrintHelp();
int  ParseArguments(int argc, char** argv, std::string& dir, std::string& trajectory, bool& save, bool& filter,double& distance, double& resolution, int& display, int& sampleType);
WSPointCloudPtr FilterPointCloud(const WSPointCloudPtr cloud);
void GenerateViewpoints(PCLViewer* viewer, const WSPointCloudPtr cloud, double distance,double resolution, int display, int sampleType = 0);
void SegmentPointCloud(PCLViewer* viewer, const std::string& dir);

void UpdatePointCloud(PCLViewer* viewer, const std::string& dir, bool bFilter)
{
  // add a new thread for spin the viewer
  //std::thread t(ViewerSpin, viewer);
  std::vector<std::string> files;
  WSPointCloudPtr cloud(new WSPointCloud());

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
        std::vector<std::string>::iterator end = files.end();
        if (std::find(files.begin(),files.end(),file) == files.end())
        {
            std::cout << "pcl == load " << all << " point cloud from " << file << std::endl;
            WSPointCloudPtr temp(new WSPointCloud());
            int res = pcl::io::loadPLYFile(file, *temp);
            if(res < 0)
              std::cout << "pcl == failed to load point cloud." << std::endl;
            if (bFilter)
              temp = FilterPointCloud(temp);

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
       viewer->AddPointCloud(cloud);
       mtx.unlock();
     }

     // Sleep for 2 seconds for the ply file
     std::this_thread::sleep_for(std::chrono::seconds(2));
  }
}

void AddCubes(PCLViewer* viewer,const std::vector<WSPoint>& points, const std::string& nameprefix, double voxelLen, int vp, double r, double g, double b)
{
  for (size_t i = 0; i < points.size(); ++i)
  {
    WSPoint c = points[i];
    std::string name = nameprefix;
    name.append("-").append(std::to_string(i));
    viewer->AddCube(c,0.5*voxelLen,name,r,g,b,vp);
  }
}

void DisplayTrajectory(PCLViewer* viewer, const std::string& dir, const std::string& file, double resolution, int type)
{
  PCLViewPoint viewCreator;
  std::vector<ViewPoint> vps;
  if (!file.empty())
  {
    viewCreator.LoadTrajectory(file,vps);
    std::cout << vps.size() << " viewpoints found in trajectory." << std::endl;
  }


  WSPointCloudPtr cloud = viewer->LoadPointCloud(dir);
  mtx.lock();
  viewer->AddPointCloud(cloud,0);
  mtx.unlock();

  PCLOctree octree(cloud, resolution);

  double voxelLen = octree.VoxelSideLength();
  WSPointCloudPtr vCloud = octree.VoxelCentroidCloud();
  std::vector<WSPoint> centroids;
  for (size_t i = 0; i < vCloud->points.size(); ++i)
    centroids.push_back(vCloud->points[i]);

  std::vector<int> allVoxels;
  int totalVoxel = octree.VoxelIndices(allVoxels);

  std::vector<WSPointCloudPtr> outsidePolygons;
  octree.FindOutsidePolygons(outsidePolygons);

  if (file.empty())
  {
    std::cout << "outsidePolygons count:" << outsidePolygons.size() << std::endl;
    mtx.lock();
    // AddCubes(viewer, centroids, "octree_voxel", voxelLen, 0, 0.0,0.0,1.0);
    for (size_t i = 0; i < outsidePolygons.size(); ++i)
    {
      std::string name("polygon_");
      name.append(std::to_string(i));
      WSPointCloudPtr pts = outsidePolygons[i];
      // std::cout << name << std::endl;
      viewer->AddPolygon(pts,name,0.0,0.0,1.0);
    }
    mtx.unlock();
  }

  WSPoint startPt, endPt;
  std::vector<int> coveredVoxels;
  for (size_t i = 0; i < vps.size(); ++i)
  {
      Eigen::Affine3f camera = viewCreator.ViewPoint2CameraPose(vps[i]);
      if (i == 0) {
        endPt.x = camera.matrix()(0,3);
        endPt.y = camera.matrix()(1,3);
        endPt.z = camera.matrix()(2,3);
        startPt = endPt;
      } else {
        startPt = endPt;
        endPt.x = camera.matrix()(0,3);
        endPt.y = camera.matrix()(1,3);
        endPt.z = camera.matrix()(2,3);
      }

      std::vector<int> indices;
      WSPointCloudPtr voxelCloud = viewCreator.CameraViewVoxels(octree, camera, indices);
      std::vector<WSPoint> vCenters;
      for (size_t j = 0; j < voxelCloud->points.size(); ++j)
      {
        WSPoint vCenter = voxelCloud->points[j];
        int idx = octree.VoxelIndex(vCenter);
        if (std::find(coveredVoxels.begin(), coveredVoxels.end(), idx) == coveredVoxels.end())
        {
          coveredVoxels.push_back(idx);
          vCenters.push_back(vCenter);
        }
      }
      // std::cout << vCenters.size() << " voxels are covered." << std::endl;

      mtx.lock();
      std::string text("viewpoint: ");
      text.append(std::to_string(i+1)).append("/").append(std::to_string(vps.size()));
      double dCoverage = 100.0*(double(coveredVoxels.size()) / double(totalVoxel));
      std::string coverage(", total coverage: ");
      coverage.append(std::to_string(dCoverage)).append(" %");
      text.append(coverage);
      viewer->AddText(text, "text");
      if (type == -1)
      {
        viewer->AddCoordinateSystem(camera,i,0,true);
      }
      else if (type == -2)
      {
        AddCubes(viewer, vCenters, std::to_string(i), voxelLen, 0, 1.0,0.0,0.0);
      }
      else if (type == -3)
      {
        viewer->AddCoordinateSystem(camera,i,0,true);
        viewer->AddLine(startPt,endPt,i,0.0,0.0,0.0);
      }
      else
      {
        viewer->AddCoordinateSystem(camera,i,0,true);
        viewer->AddLine(startPt,endPt,i,0.0,0.0,0.0);
        AddCubes(viewer, vCenters, std::to_string(i), voxelLen, 0, 1.0,0.0,0.0);
      }
      mtx.unlock();

      // Sleep for 2 seconds for the ply file
      std::this_thread::sleep_for(std::chrono::seconds(2));
  }
}

int main(int argc, char** argv) try
{
  bool bSave = false;
  bool bFilter = false;
  std::string dir = "";
  std::string trajectory = "";
  double resolution = 1.0;
  int display = -5;
  double distance = -1;
  int sampleType = 0;
  int task = ParseArguments(argc, argv, dir, trajectory, bSave, bFilter, distance, resolution, display, sampleType);
  PCLViewer viewer("3D Point Cloud Viewer");
  if (task == 2)
  {
    std::thread t(UpdatePointCloud, &viewer, dir, bFilter);
    while (!viewer.IsStop()) {
      mtx.lock();
      viewer.SpinOnce();
      mtx.unlock();
    }
    t.join();
  }
  else if(task == 1)
  {
    std::thread t(DisplayTrajectory, &viewer, dir, trajectory, resolution, sampleType);
    while (!viewer.IsStop()) {
      mtx.lock();
      viewer.SpinOnce();
      mtx.unlock();
    }
    t.join();
  }
  else if (task == 3)
  {
    WSPointCloudPtr cloud = viewer.LoadPointCloud(dir);
    GenerateViewpoints(&viewer, cloud, distance, resolution, display, sampleType);
    while(!viewer.IsStop())
      viewer.Spin();
  }
  else if (task == 4)
  {
    SegmentPointCloud(&viewer, dir);
    while(!viewer.IsStop())
      viewer.Spin();
  }
  else
  {
    PrintHelp();
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

// generate viewpoints with distance to the aircraft and resolution of octree
void GenerateViewpoints(PCLViewer* viewer, const WSPointCloudPtr srcCloud, double distance, double resolution, int display, int sampleType)
{
  if (nullptr == srcCloud)
    return;

  ///////////////////////////////////////////////////////////////////////////
  // viewpoint creator
  PCLViewPoint viewCreator;

  // create camera along the voxel normals
  std::vector<Eigen::Affine3f> cameraPoseVec;
  std::vector<ViewPoint> viewpointVec;
  PCLOctree octree(srcCloud, resolution);
  viewCreator.GenerateCameraPositions(octree, distance, cameraPoseVec, viewpointVec, 1);
  std::cout << cameraPoseVec.size() << " viewpoints generated." << std::endl;

  ////////////////////////////////////////////////////////////////////////////
  // voxel in view frustum culling
  // use a higher resolution to calculate voxel coverage
  PCLOctree octree2(srcCloud,0.5);
  std::vector<int> voxelCoveredVec;
  std::map<int, std::vector<int> > viewVoxelMap;
  for (size_t i = 0; i < cameraPoseVec.size(); ++i)
  {
    std::vector<int> visibleVoxelIndices;
    WSPointCloudPtr viewCloud = viewCreator.CameraViewVoxels(octree2,cameraPoseVec[i],visibleVoxelIndices);
    viewVoxelMap.insert(std::make_pair(static_cast<int>(i), visibleVoxelIndices));
    // calculate covered voxels
    for (size_t j = 0; j < visibleVoxelIndices.size(); ++j)
    {
      int vIndex = visibleVoxelIndices[j];
      if (std::find(voxelCoveredVec.begin(), voxelCoveredVec.end(), vIndex) == voxelCoveredVec.end())
        voxelCoveredVec.push_back(vIndex);
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  // voxel coverage
  std::vector<int> voxelUncoveredVec;
  std::vector<int> allVoxelVec;
  int total = octree2.VoxelIndices(allVoxelVec);
  for (size_t i = 0; i < total; i++)
  {
    if (std::find(voxelCoveredVec.begin(), voxelCoveredVec.end(), allVoxelVec[i]) == voxelCoveredVec.end())
      voxelUncoveredVec.push_back(allVoxelVec[i]);
  }
  double coverage = 100.0*(double(voxelCoveredVec.size())/double(total));
  std::cout << "total voxel: " << total << " voxel covered/uncovered " << voxelCoveredVec.size()
            << "/" << voxelUncoveredVec.size() << " coverage: " << coverage << " %" << std::endl;

  /////////////////////////////////////////////////////////////////////////////
  // save viewpoints
  std::string fileName("/home/yufeng/Temp/viewpoint");
  fileName.append(std::to_string(distance)).append("_").append(std::to_string(resolution)).append(".txt");
  viewCreator.Save2File(fileName, viewpointVec, viewVoxelMap);
  std::cout << "save to " << fileName << std::endl;

  ////////////////////////////////////////////////////////////////////////////
  // display
  int vp = viewer->CreateViewPort(0,0,1,1);
  viewer->AddPointCloud(srcCloud,vp);
  // display voxel cube, camere view and frustum visible voxels
  double voxelLen = octree2.VoxelSideLength();
  if (display == -2)
  {
    WSPointCloudPtr vCloud = octree2.VoxelCentroidCloud();
    std::vector<WSPoint> centroids;
    for (size_t i = 0; i < vCloud->points.size(); ++i)
      centroids.push_back(vCloud->points[i]);
    AddCubes(viewer, centroids, "octree_voxel", voxelLen, vp, 0.0,0.0,1.0);
  }
  else if (display == -1)
  {
    for (size_t i = 0; i < cameraPoseVec.size(); ++i)
      viewer->AddCoordinateSystem(cameraPoseVec[i],i);
  }
  else if (display == -3)
  {
    std::vector<WSPoint> centroids;
    for (size_t i = 0; i < voxelCoveredVec.size(); ++i)
      centroids.push_back(octree2.VoxelCentroid(voxelCoveredVec[i]));
    AddCubes(viewer, centroids, "covered voxel", voxelLen, vp, 0.0,1.0,0.0);
  }
  else if (display == -4)
  {
    std::vector<WSPoint> centroids;
    for (size_t i = 0; i < voxelUncoveredVec.size(); ++i)
      centroids.push_back(octree2.VoxelCentroid(voxelUncoveredVec[i]));
    AddCubes(viewer, centroids, "uncovered voxel", voxelLen, vp, 1.0,0.0,0.0);
  }
  else
  {
    std::string name("view_frustum");
    name.append("_").append(std::to_string(0));
    std::vector<int> visibleVoxelIndices;
    WSPointCloudPtr viewCloud = viewCreator.CameraViewVoxels(octree2,cameraPoseVec[0],visibleVoxelIndices);
    std::vector<WSPoint> centroids;
    for (size_t i = 0; i < viewCloud->points.size(); ++i)
      centroids.push_back(viewCloud->points[i]);
    AddCubes(viewer, centroids, name, voxelLen, vp, 0.0,0.0,1.0);
    viewer->AddCoordinateSystem(cameraPoseVec[0],display);
  }
}

////////////////////////////////////////////////////////////////
WSPointCloudPtr FilterPointCloud(const WSPointCloudPtr srcCloud)
{
  PCLFilter filter;
  WSPointCloudPtr cloud(new WSPointCloud);
  // filter z < 0.02 and z > 10 point
  std::cout << "Filtering: pass through z out of [0.05,20]..." << std::endl;
  cloud = filter.FilterPassThrough(srcCloud,"z",0.05,20);
  /*
  remove outlier
  The number of neighbors to analyze for each point is set to 50,
  and the standard deviation multiplier to 1. What this means is
  that all points who have a distance larger than 1 standard
  deviation of the mean distance to the query point will be marked
  as outliers and removed.
  */
  std::cout << "Filtering: outlier with 50 samples in 1 standard deviation..." << std::endl;
  cloud = filter.FilterPCLPointSOR(cloud,50,1);
  // downsampling with neighbor distance 0.1
  std::cout << "Filtering: downsampling with 0.1 meter neighbor distance..." << std::endl;
  cloud = filter.FilterPCLPoint(cloud,0.1);
  return cloud;
}

void PrintHelp()
{
  std::cout << "This app is used for visualizing the point cloud by loading the '.ply' files in a directory.\n";
  std::cout << "Command line with providing a directory containing the .ply files:\n";
  std::cout << "    1. view point cloud:   cmd [-view|-v] [file_directory] [octree resolution] [display_type] [trajectory]\n";
  std::cout << "    2. merge pointcloud:   cmd [-merge|-m] [file_directory] [filter]\n";
  std::cout << "    3. create viewpoints:  cmd [-trajectory|-t] [file_directory] [viewpoint distance] [octree resolution] [display_type]\n";
  std::cout << "       display_type: '-4': uncovered voxels; '-3': covered voxels; '-2': add octree voxels; '-1': add camera positions;\n";
  std::cout << "    4. segment pointcloud: cmd [-segment|-s] [file_directory]\n";
}

int ParseArguments(int argc, char** argv,
                  std::string& dir,
                  std::string& trajectory,
                  bool& save,
                  bool& filter,
                  double& distance,
                  double& resolution,
                  int& display,
                  int& sampleType)
{
  if (argc < 2)
    return 0;

  std::string task = std::string(argv[1]);
  if (task.compare("-v") == 0 || task.compare("-view") == 0)
  {
    dir = std::string(argv[2]);
    if (argc > 3)
      resolution = std::stod(argv[3]);
    if (argc > 4)
      sampleType = std::stoi(argv[4]);
    if (argc > 5)
      trajectory = std::string(argv[5]);

    return 1;
  }
  else if (task.compare("-m") == 0 || task.compare("-merge") == 0)
  {
    save = true;
    dir = std::string(argv[2]);
    if (argc > 3)
      filter = std::string(argv[3]).compare("1")==0;
    return 2;
  }
  else if (task.compare("-t") == 0 || task.compare("-trajectory") == 0)
  {
    dir = std::string(argv[2]);
    if (argc > 3)
      distance = std::stod(argv[3]);
    if (argc > 4)
      resolution = std::stod(argv[4]);
    if (argc > 5)
      display = std::stoi(argv[5]);
    if (argc > 6)
      sampleType = std::stoi(argv[6]);
    return 3;
  }
  else if (task.compare("-s")==0 || task.compare("-segment") == 0)
  {
    dir = std::string(argv[2]);
    return 4;
  }
  else
  {
    return 0;
  }
}

void SegmentPointCloud(PCLViewer* viewer, const std::string& dir)
{
  WSPointCloudPtr cloud = viewer->LoadPointCloud(dir);
  if (!cloud)
    return;

  std::vector<WSPointCloudPtr> segClouds;
  PCLSegment segment(cloud);
  bool success = segment.Compute(segClouds);
  if (!success)
    return;

  std::cout << segClouds.size() << "segments" << std::endl;
  for (int i = 0; i < segClouds.size(); ++i)
  {
    WSPointCloudPtr segCloud = segClouds[i];
    std::string file = dir+"segment_"+std::to_string(i)+".ply";
    int res = pcl::io::savePLYFileASCII(file, *segCloud);
    if (res >= 0)
      std::cout << "save point cloud" << i << "as" << file << std::endl;
  }

  int vp = viewer->CreateViewPort(0,0,1,1);
  viewer->AddPointCloud(cloud,vp);
}
