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
int  ParseArguments(int argc, char** argv,
  std::string& dir,
  std::string& trajectory,
  bool& save,
  bool& filter,
  double& distance,
  double& resolution,
  int& display,
  int& sampleType,
  std::vector<double>& limitBox,
  string& configFile);

WSPointCloudPtr FilterPointCloud(const WSPointCloudPtr cloud,
  double resolution);

void GenerateViewpoints(PCLViewer* viewer,
  const WSPointCloudPtr cloud,
  double distance,
  double resolution,
  int display,
  const string& configFile,
  int sampleType = 0);

void UpdatePointCloud(PCLViewer* viewer,
  const std::string& dir,
  bool bFilter,
  double resolution);

void DisplayTrajectory(PCLViewer* viewer,
  const std::string& dir,
  const std::string& file,
  double resolution,
  int type);

void SegmentPointCloud(PCLViewer* viewer,
  const std::string& dir,
  const std::vector<double>& limitBox,
  bool bFilter);

void GenerateScanningPath(PCLViewer* viewer,
  const WSPointCloudPtr cloud,
  double distance,
  double resolution
);

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
  std::vector<double> limitBox;
  string configFile = "";
  int task = ParseArguments(argc,argv,dir,
    trajectory,
    bSave,
    bFilter,
    distance,
    resolution,
    display,
    sampleType,
    limitBox,
    configFile);

  PCLViewer viewer("3D Point Cloud Viewer");
  if(task == 1)
  {
    std::thread t(DisplayTrajectory, &viewer, dir, trajectory, resolution, sampleType);
    while (!viewer.IsStop()) {
      mtx.lock();
      viewer.SpinOnce();
      mtx.unlock();
    }
    t.join();
  }
  else if (task == 2)
  {
    std::thread t(UpdatePointCloud, &viewer, dir, bFilter,resolution);
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
    GenerateViewpoints(&viewer, cloud, distance, resolution, display, configFile, sampleType);
    while(!viewer.IsStop())
    {
      mtx.lock();
      viewer.SpinOnce();
      mtx.unlock();
    }
  }
  else if (task == 4)
  {
    SegmentPointCloud(&viewer, dir, limitBox, bFilter);
    while(!viewer.IsStop())
    {
      mtx.lock();
      viewer.SpinOnce();
      mtx.unlock();
    }
  }
  else if (task == 5)
  {
    WSPointCloudPtr cloud = viewer.LoadPointCloud(dir);
    GenerateScanningPath(&viewer,cloud,distance,resolution);
    while(!viewer.IsStop())
    {
      mtx.lock();
      viewer.SpinOnce();
      mtx.unlock();
    }
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

void UpdatePointCloud(PCLViewer* viewer, const std::string& dir, bool bFilter,double resolution)
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
              temp = FilterPointCloud(temp,resolution);

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

  PCLOctree octree(cloud,0.5*resolution,3);

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
      //double dCoverage = 100.0*(double(coveredVoxels.size()) / double(totalVoxel));
      //std::string coverage(", total coverage: ");
      //coverage.append(std::to_string(dCoverage)).append(" %");
      //text.append(coverage);
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
      std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

// generate viewpoints with distance to the aircraft and resolution of octree
void GenerateViewpoints(PCLViewer* viewer,
  const WSPointCloudPtr srcCloud,
  double distance,
  double resolution,
  int display,
  const string& configFile,
  int sampleType)
{
  if (nullptr == srcCloud)
    return;

  // load configuration
  typedef std::vector<double> BBox;
  std::vector<std::pair<Eigen::Vector3f,BBox> > segments;
  std::ifstream config(configFile);
  std::string line;
  std::cout << "read config " << segments.size() << std::endl;
  while (std::getline(config, line))
  {
    std::cout << line << std::endl;
    std::stringstream linestream(line);
    std::string nx,ny,nz,xmin,xmax,ymin,ymax,zmin,zmax;
    linestream >> nx >> ny >> nz >> xmin >> xmax >> ymin >> ymax >> zmin >> zmax;
    BBox box;
    box.push_back(std::stod(xmin));
    box.push_back(std::stod(xmax));
    box.push_back(std::stod(ymin));
    box.push_back(std::stod(ymax));
    box.push_back(std::stod(zmin));
    box.push_back(std::stod(zmax));
    Eigen::Vector3f refNormal(std::stod(nx),std::stod(ny),std::stod(nz));
    segments.push_back(std::make_pair(refNormal,box));
  }
  config.close();

  ///////////////////////////////////////////////////////////////////////////
  // viewpoint creator
  PCLViewPoint viewCreator;

  // create camera along the voxel normals
  std::vector<ViewPoint> vps;
  std::vector<Eigen::Affine3f> cameras;
  PCLOctree octree(srcCloud, resolution,3);
  for (int i = 0; i < segments.size(); ++i)
  {
    Eigen::Vector3f refNormal = segments[i].first;
    BBox bbox = segments[i].second;
    viewCreator.GenerateCameraPositions(octree,distance,refNormal,bbox,cameras,vps);
  }


  ////////////////////////////////////////////////////////////////////////////
  // voxel in view frustum culling
  // use a higher resolution to calculate voxel coverage
  PCLOctree octree2(srcCloud,0.5*resolution,3);
  // filter viewpoints
  std::vector<ViewPoint> viewpointVec;
  std::vector<Eigen::Affine3f> cameraPoseVec;
  for (size_t i = 0; i < cameras.size(); ++i)
  {
    ViewPoint vp = vps[i];
    double qr_x = vp.quadrotor_pose.pos_x;
    double qr_y = vp.quadrotor_pose.pos_y;
    double qr_z = vp.quadrotor_pose.pos_z;
    // //height limitation
    if (qr_x < 3.0 && qr_x >-3.0)
    {
      if (qr_z > 3 && qr_z < 8)
      {
        if (qr_y > -26)
          continue;
      }
    }

    if(viewCreator.FilterViewPoint(octree2,cameras[i],vp))
      continue;

    viewpointVec.push_back(vps[i]);
    cameraPoseVec.push_back(cameras[i]);
  }

  std::vector<int> voxelCoveredVec;
  std::map<int, std::vector<int> > viewVoxelMap;
  for (size_t i = 0; i < cameraPoseVec.size(); ++i)
  {
    std::vector<int> visibleVoxels;
    WSPointCloudPtr viewCloud = viewCreator.CameraViewVoxels(octree2,cameraPoseVec[i],visibleVoxels);
    viewVoxelMap.insert(std::make_pair(static_cast<int>(i), visibleVoxels));
    for (size_t j = 0; j < visibleVoxels.size(); ++j)
    { // calculate covered voxels
      int vIndex = visibleVoxels[j];
      if (std::find(voxelCoveredVec.begin(),voxelCoveredVec.end(),vIndex) == voxelCoveredVec.end())
        voxelCoveredVec.push_back(vIndex);
    }
  }
  std::cout << cameraPoseVec.size() << " filtered camere positions / " << vps.size() << " generated." << std::endl;


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

void SegmentPointCloud(PCLViewer* viewer, const std::string& dir, const std::vector<double>& bbox, bool bFilter)
{
  WSPointCloudPtr cloud = viewer->LoadPointCloud(dir);
  if (!cloud)
    return;

  if (bbox.empty())
  {
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
  else
  {
    std::cout << "filter point cloud in bounding box" << bbox[0] << " "<< bbox[1]<< " "
    << bbox[2]<< " " << bbox[3] << " "<< bbox[4] << " "<< bbox[5]<< std::endl;
    PCLFilter filter;
    cloud = filter.FilterPCLPointInBBox(cloud,bbox,bFilter);
    int vp = viewer->CreateViewPort(0,0,1,1);
    viewer->AddPointCloud(cloud,vp);
  }
}

// generate continous scanning path with input Cloud
// 1. slice the cloud with plane
// 2. find all point on the plane
// 3. offset the point
// 4. conncet all the point form a polyline
void GenerateScanningPath(PCLViewer* viewer, const WSPointCloudPtr cloud, double distance, double resolution)
{
  // the point cloud bounding box is
  // x: -5 to -5
  // y: -23 to -13
  // z : 4.5 to 8
  int vp = viewer->CreateViewPort(0,0,1,1);
  viewer->AddPointCloud(cloud, vp);

  std::vector<double> bbox;
  bbox.push_back(0);
  bbox.push_back(5);
  bbox.push_back(-23);
  bbox.push_back(-13);
  bbox.push_back(4.6);
  bbox.push_back(8);
  PCLFilter filter;
  WSPointCloudPtr newCloud = filter.FilterPCLPointInBBox(cloud,bbox,0);
  // WSPointCloudPtr newCloud = filter.FilterPCLPoint(newCloud,0.001);
  double z = 4.6;
  double x = 0;
  double y = -23.0+0.05;
  Eigen::Vector3f normal(0,1,0);
  int n = 1;
  std::vector<WSPoint> ftPts;
  while (y < -13)
  {
    Eigen::Vector3f root(x,y,z);
    WSPointCloudPtr pts = filter.SlicePoints(newCloud, root, normal);
    pts = filter.SortPointsInZ(pts,0.3,4.5,8);
    int numberPt = pts->points.size();
    if (n%2 == 0)
    {
      for (int i = numberPt-1; i >= 0; i--)
        ftPts.push_back(pts->points[i]);
    }
    else
    {
      for (int i = 0; i < numberPt; ++i)
        ftPts.push_back(pts->points[i]);
    }
    y = y + resolution;
    n++;
  }

  // offset
  std::vector<WSPoint> offsetPts;
  for (size_t i = 0; i < ftPts.size(); ++i)
  {
    WSPoint pt = ftPts[i];
    WSNormal nm = filter.PointNormal(pt,cloud,0.3,Eigen::Vector3f(1,0,1));

    WSPoint offset;
    offset.x = pt.x+nm.normal_x*distance;
    offset.y = pt.y+nm.normal_y*distance;
    offset.z = pt.z+nm.normal_z*distance;
    offsetPts.push_back(offset);
  }

  // add polyline path
  for (size_t i = 0; i < offsetPts.size(); ++i)
  {
    if (i > 1)
      viewer->AddLine(offsetPts[i-1],offsetPts[i],i,1.0,0.0,0.0,vp);
  }
}

////////////////////////////////////////////////////////////////
WSPointCloudPtr FilterPointCloud(const WSPointCloudPtr srcCloud, double resolution)
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
  std::cout << "Filtering: downsampling with " << resolution << " meter neighbor distance..." << std::endl;
  cloud = filter.FilterPCLPoint(cloud,resolution);
  return cloud;
}

void PrintHelp()
{
  std::cout << "This app is used for visualizing the point cloud by loading the '.ply' files in a directory.\n";
  std::cout << "Command line with providing a directory containing the .ply files:\n";
  std::cout << "    1. view point cloud:   cmd [-view|-v] [file_directory] [octree resolution] [display_type] [trajectory]\n";
  std::cout << "    2. merge pointcloud:   cmd [-merge|-m] [file_directory] [filter] [downsample resolution]\n";
  std::cout << "    3. create viewpoints:  cmd [-trajectory|-t] [file_directory] [viewpoint distance] [octree resolution] [display_type] [config]\n";
  std::cout << "       display_type: '-4': uncovered voxels; '-3': covered voxels; '-2': add octree voxels; '-1': add camera positions;\n";
  std::cout << "    4. segment pointcloud: cmd [-segment|-s] [file_directory] [xmin] [xmax] [ymin] [ymax] [zmin] [zmax]\n";
  std::cout << "    5. create continuous trajectory: cmd [-c] [file_directory] [offset distance] [margin]\n";
}

int ParseArguments(int argc, char** argv,
                  std::string& dir,
                  std::string& trajectory,
                  bool& save,
                  bool& filter,
                  double& distance,
                  double& resolution,
                  int& display,
                  int& sampleType,
                  std::vector<double>& limitBox,
                  string& configFile
                )
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
    if (argc > 4)
      resolution = std::stod(argv[4]);
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
      configFile = std::string(argv[6]);
    return 3;
  }
  else if (task.compare("-s")==0 || task.compare("-segment") == 0)
  {
    save = true;
    dir = std::string(argv[2]);
    if (argc > 8)
    {
      limitBox.push_back(std::stod(argv[3]));
      limitBox.push_back(std::stod(argv[4]));
      limitBox.push_back(std::stod(argv[5]));
      limitBox.push_back(std::stod(argv[6]));
      limitBox.push_back(std::stod(argv[7]));
      limitBox.push_back(std::stod(argv[8]));
    }
    if (argc > 9)
      filter = std::string(argv[9]).compare("1")==0;

    return 4;
  }
  else if (task.compare("-c")==0)
  {
    dir = std::string(argv[2]);
    if (argc > 3)
      distance = std::stod(argv[3]);
    if (argc > 4)
      resolution = std::stod(argv[4]);
    return 5;
  }
  else
  {
    return 0;
  }
}
