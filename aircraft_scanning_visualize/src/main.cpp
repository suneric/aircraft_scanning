#include <iostream>
#include <math.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <vector>
#include <fstream>
#include <sstream>
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

std::mutex mtx;

void PrintHelp();
int  ParseArguments(int argc, char** argv, std::string& dir, bool& save, bool& filter,double& distance, double& resolution, int& display);
WSPointCloudPtr FilterPointCloud(const WSPointCloudPtr cloud);
void GenerateViewpoints(PCLViewer* viewer, const WSPointCloudPtr cloud, double distance,double resolution, int display);

void UpdatePointCloud(PCLViewer* viewer, const std::string& dir, bool bFilter)
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

int main(int argc, char** argv) try
{
  bool bSave = false;
  bool bFilter = false;
  std::string dir = "";
  double resolution = -1.0;
  int display = -5;
  double distance = -1;
  int task = ParseArguments(argc, argv, dir, bSave, bFilter, distance, resolution, display);
  PCLViewer viewer("3D Point Cloud Viewer");
  if (task == 0)
  {
      PrintHelp();
  }
  else if (task == 1 || task == 2)
  {
    std::thread t(UpdatePointCloud, &viewer, dir, bFilter);
    while(!viewer.IsStop())
        viewer.Spin();
    t.join();
  }
  else if (task == 3)
  {
    WSPointCloudPtr cloud = viewer.LoadPointCloud(dir);
    GenerateViewpoints(&viewer, cloud, distance, resolution, display);
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

void AddCubes(PCLViewer* viewer,const std::vector<WSPoint>& points, const std::string& nameprefix, double voxelLen, int vp, double r, double g, double b)
{
  for (size_t i = 0; i < points.size(); ++i)
  {
    WSPoint c = points[i];
    std::string name = nameprefix;
    name.append(std::to_string(i));
    viewer->AddCube(c,0.5*voxelLen,name,r,g,b,vp);
  }
}

// generate viewpoints with distance to the aircraft and resolution of octree
void GenerateViewpoints(PCLViewer* viewer, const WSPointCloudPtr srcCloud, double distance, double resolution, int display)
{
  if (nullptr == srcCloud)
    return;

  // octree utility
  PCLOctree octree(srcCloud, resolution);
  std::cout << "tree depth: " << octree.TreeDepth() << std::endl;

  ///////////////////////////////////////////////////////////////////////////
  // viewpoint creator
  PCLViewPoint viewCreator;
  std::vector<Eigen::Affine3f> cameraPoseVec;
  viewCreator.GenerateCameraPositions(octree, distance, cameraPoseVec);
  std::cout << cameraPoseVec.size() << " viewpoints generated." << std::endl;

  // filter camera pose
  std::vector<ViewPoint> viewpointVec;
  std::vector<Eigen::Affine3f> cameraQualify;
  for (size_t i = 0; i < cameraPoseVec.size(); ++i)
  {
    ViewPoint viewpt;
    if (!viewCreator.FilterViewPoint(octree, cameraPoseVec[i], viewpt))
    {
      viewpointVec.push_back(viewpt);
      cameraQualify.push_back(cameraPoseVec[i]);
    }
  }
  std::cout << cameraQualify.size() << " viewpoints qulified." << std::endl;

  ////////////////////////////////////////////////////////////////////////////
  // voxel in view frustum culling
  std::vector<int> voxelCoveredVec;
  std::map<int, std::vector<int> > viewVoxelMap;
  for (size_t i = 0; i < cameraQualify.size(); ++i)
  {
    std::vector<int> visibleVoxelIndices;
    WSPointCloudPtr viewCloud = viewCreator.CameraViewVoxels(octree,cameraQualify[i],visibleVoxelIndices);
    viewVoxelMap.insert(std::make_pair(static_cast<int>(i), visibleVoxelIndices));
    //std::cout << visibleVoxelIndices.size() << std::endl;
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
  int total = octree.VoxelIndices(allVoxelVec);
  for (size_t i = 0; i < total; i++)
  {
    if (std::find(voxelCoveredVec.begin(), voxelCoveredVec.end(), allVoxelVec[i]) == voxelCoveredVec.end())
      voxelUncoveredVec.push_back(allVoxelVec[i]);
  }
  double coverage = 100.0*(double(voxelCoveredVec.size())/double(total));
  std::cout << "voxel covered/uncovered " << voxelCoveredVec.size() << "/" << voxelUncoveredVec.size() << " coverage: " << coverage << " %" << std::endl;

  /////////////////////////////////////////////////////////////////////////////
  // save viewpoints
  std::string fileName("/home/yufeng/Temp/viewpoint");
  fileName.append(std::to_string(distance)).append("_").append(std::to_string(resolution)).append(".txt");
  std::ofstream tFile(fileName);
  tFile << "voxel count: " << total << "\n";
  for (size_t i = 0; i < viewpointVec.size(); ++i)
  {
    ViewPoint viewpt = viewpointVec[i];
    tFile << "viewpoint" << i << ": quadrotor pose:["
                               << viewpt.quadrotor_pose.pos_x << ","
                               << viewpt.quadrotor_pose.pos_y << ","
                               << viewpt.quadrotor_pose.pos_z << ","
                               << viewpt.quadrotor_pose.ori_x << ","
                               << viewpt.quadrotor_pose.ori_y << ","
                               << viewpt.quadrotor_pose.ori_z << ","
                               << viewpt.quadrotor_pose.ori_w << "] camera angle: "
                               << viewpt.camera_angle << "\n";
    tFile << "view frustum voxels:[ ";
    // std::cout << viewVoxelMap[i].size() << std::endl;
    for (size_t j = 0; j < viewVoxelMap[i].size(); ++j)
    {
      int vIndex = viewVoxelMap[i][j];
      tFile << vIndex << " ";
    }
    tFile << "]\n";
  }
  tFile.close();
  std::cout << "save viewpoints to " << fileName << std::endl;

  ////////////////////////////////////////////////////////////////////////////
  // display
  int vp = viewer->CreateViewPort(0,0,1,1);
  viewer->AddPointCloud(srcCloud,vp);
  // display voxel cube, camere view and frustum visible voxels
  double voxelLen = octree.VoxelSideLength();
  if (display == -2)
  {
    WSPointCloudPtr vCloud = octree.VoxelCentroidCloud();
    std::vector<WSPoint> centroids;
    for (size_t i = 0; i < vCloud->points.size(); ++i)
      centroids.push_back(vCloud->points[i]);
    AddCubes(viewer, centroids, "octree_voxel", voxelLen, vp, 0.0,0.0,1.0);
  }
  else if (display == -1)
  {
    for (size_t i = 0; i < cameraQualify.size(); ++i)
      viewer->AddCoordinateSystem(cameraQualify[i],i);
  }
  else if (display == -3)
  {
    std::vector<WSPoint> centroids;
    for (size_t i = 0; i < voxelCoveredVec.size(); ++i)
      centroids.push_back(octree.VoxelCentroid(voxelCoveredVec[i]));
    AddCubes(viewer, centroids, "covered voxel", voxelLen, vp, 0.0,1.0,0.0);
  }
  else if (display == -4)
  {
    std::vector<WSPoint> centroids;
    for (size_t i = 0; i < voxelUncoveredVec.size(); ++i)
      centroids.push_back(octree.VoxelCentroid(voxelUncoveredVec[i]));
    AddCubes(viewer, centroids, "uncovered voxel", voxelLen, vp, 1.0,0.0,0.0);
  }
  else
  {
    std::string name("view_frustum");
    name.append("_").append(std::to_string(display));
    std::vector<int> visibleVoxelIndices;
    WSPointCloudPtr viewCloud = viewCreator.CameraViewVoxels(octree,cameraQualify[display],visibleVoxelIndices);
    std::vector<WSPoint> centroids;
    for (size_t i = 0; i < viewCloud->points.size(); ++i)
      centroids.push_back(viewCloud->points[i]);
    AddCubes(viewer, centroids, name, voxelLen, vp, 0.0,0.0,1.0);
    viewer->AddCoordinateSystem(cameraQualify[display],display);
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
  std::cout << "    1. view point cloud:   cmd [-view|-v] [file_directory]\n";
  std::cout << "    2. merge pointcloud:   cmd [-merge|-m] [file_directory] [filter]\n";
  std::cout << "    3. create viewpoints:  cmd [-trajectory|-t] [file_directory] [viewpoint distance] [octree resolution] [display_type]\n";
  std::cout << "       display_type: '-4': uncovered voxels; '-3': covered voxels; '-2': add octree voxels; '-1': add camera positions;\n";
}

int ParseArguments(int argc, char** argv, std::string& dir, bool& save, bool& filter, double& distance, double& resolution, int& display)
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
    return 3;
  }
  else
  {
    return 0;
  }
}

// PCLFilter filter;
// WSPointCloudNormalPtr normals(new WSPointCloudNormal);
// WSPointCloudPtr nCloud = filter.SamplingSurfaceNormal(cloud,100,0.001,normals);
// viewer->AddPointCloud(nCloud, vp);
// viewer->AddNormals(nCloud,normals,1,1,vp);

// PCLFilter filter;
// WSPointCloudPtr sample = filter.UniformSampling(cloud,0.5);
// WSPointCloudNormalPtr normal(new WSPointCloudNormal);
// PCLSegment segment(cloud);
// if (segment.ComputeNormals(normal))
// {
//   // WSPointCloudPtr sCloud = filter.ExtractPoints(cloud,inliers);
//   // WSPointCloudNormalPtr sNormal = filter.ExtractNormals(normal, inliers);
//   // sNormal = filter.FlipNormals(cloud, normal,inliers);
//   int vp = viewer->CreateViewPort(0,0,1,1);
//   viewer->AddPointCloud(cloud,vp);
//   viewer->AddNormals(cloud, normal, 1, 1, vp);
// }

// WSPointCloudNormalPtr normal(new WSPointCloudNormal);
// PCLSegment segment(cloud);
// if (segment.ComputeNormals(normal))
// {
//   std::vector<int> inliers;
//   PCLFilter filter;
//   if (filter.RandomSampling(cloud,2000,inliers))
//   {
//     WSPointCloudPtr sCloud = filter.ExtractPoints(cloud,inliers);
//     WSPointCloudNormalPtr sNormal = filter.ExtractNormals(normal, inliers);
//     sNormal = filter.FlipNormals(cloud, normal,inliers);
//     int vp = viewer->CreateViewPort(0,0,1,1);
//     viewer->AddPointCloud(cloud,vp);
//     viewer->AddNormals(sCloud, sNormal, 1, 0.5, vp);
//   }
// }

// WSPointCloudNormalPtr normal(new WSPointCloudNormal);
// PCLSegment segment(cloud);
// if (segment.ComputeNormals(normal))
// {
//   int vp = viewer.CreateViewPort(0,0,1,1);
//   viewer.AddPointCloud(cloud,vp);
//   viewer.AddNormals(cloud, normal, 1, 1, vp);
// }

// WSPointCloudNormalPtr normal(new WSPointCloudNormal);
// PCLSegment segment(cloud);
// // if (segment.ComputeNormals(normal))
// // {
// //   int vp = viewer.CreateViewPort(0,0,1,1);
// //   viewer.AddPointCloud(cloud,vp);
// //   viewer.AddNormals(cloud, normal, 50, 1, vp);
// // }
//
// // PCLSegment segment(cloud);
// std::vector<WSPointCloudPtr> segClouds;
// bool success = segment.Compute(segClouds);
// if (success)
// {
//   std::cout << segClouds.size() << "segments" << std::endl;
//   // int vp = viewer.CreateViewPort(0,0,1,1);
//   // viewer.AddPointCloud(segClouds[0],vp);
//   // int nSeg = segClouds.size();
//   // std::cout << "Segmentation: " << nSeg << " segments were generated." << std::endl;
//   // int vp0 = viewer.CreateViewPort(0,0,0.5,0.5);
//   // viewer.AddPointCloud(segClouds[0], vp0);
//   // int vp1 = viewer.CreateViewPort(0.5,0,1,0.5);
//   // viewer.AddPointCloud(segClouds[1],vp1);
//   // int vp2 = viewer.CreateViewPort(0,0.5,0.5,1);
//   // viewer.AddPointCloud(segClouds[2],vp2);
//   // int vp3 = viewer.CreateViewPort(0.5,0.5,1,1);
//   // viewer.AddPointCloud(segClouds[3],vp3);
//
//   int vp0 = viewer.CreateViewPort(0,0,0.5,0.5);
//   viewer.AddPointCloud(segClouds[0], vp0);
//   int vp1 = viewer.CreateViewPort(0.5,0,1,0.5);
//   viewer.AddPointCloud(segment.PointCloud(),vp1);
// }
