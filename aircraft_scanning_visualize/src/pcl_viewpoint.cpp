#include "pcl_viewpoint.h"
#include "pcl_filter.h"
#include <math.h>
#include <fstream>
#include <sstream>

#include <pcl/filters/frustum_culling.h>
#include <pcl/features/normal_3d.h>

using namespace asv3d;

WSPointCloudPtr PCLViewPoint::CameraViewVoxels(const PCLOctree& tree, const Eigen::Affine3f& camera, std::vector<int>& voxelIndices)
{
  WSPointCloudPtr viewCloud(new WSPointCloud);
  Eigen::Matrix4f camMatrix = camera.matrix();
  Eigen::Vector3f cameraPt(camMatrix(0,3),camMatrix(1,3),camMatrix(2,3));
  if(FrustumCulling(tree.VoxelCentroidCloud(),camera.matrix(),68,42,0.1,10.0,viewCloud))
  {
    std::vector<WSPoint> visibleVoxels;
    for (int i = 0; i < viewCloud->points.size(); ++i)
    {
      WSPoint centroid = viewCloud->points[i];
      Eigen::Vector3f endPt(centroid.x, centroid.y, centroid.z);
      std::vector<WSPoint> intesects;
      // get outer side voxels, with max intersected voxel number is 1
      int nRes = tree.GetIntersectedVoxelCenters(cameraPt, endPt-cameraPt, 1, intesects);
      if (nRes == 1)
        visibleVoxels.push_back(intesects[0]);
    }
    // voxel index
    std::vector<WSPoint> visibleCentroids;
    for (size_t i = 0; i < visibleVoxels.size(); ++i)
    {
      int voxelIndex = tree.VoxelIndex(visibleVoxels[i]);
      if (std::find(voxelIndices.begin(), voxelIndices.end(),voxelIndex) == voxelIndices.end())
      {
        voxelIndices.push_back(voxelIndex);
        visibleCentroids.push_back(visibleVoxels[i]);
      }
    }

    WSPointCloudPtr visibleVoxelCloud(new WSPointCloud);
    visibleVoxelCloud->points.resize(visibleCentroids.size());
    visibleVoxelCloud->points.assign(visibleCentroids.begin(), visibleCentroids.end());
    return visibleVoxelCloud;
  }
  return nullptr;
}

void PCLViewPoint::GenerateCameraPositions(const PCLOctree& tree,double distance,std::vector<Eigen::Affine3f>& cameras, std::vector<ViewPoint>& vps, int type)
{
  if (type == 0)
    CameraPositionWithVoxelAverageNormal(tree, distance, cameras, vps);
  else
    CameraPositionWithVoxelCube(tree, distance, cameras, vps);
}

void PCLViewPoint::CameraPositionWithVoxelCube(const PCLOctree& tree, double distance, std::vector<Eigen::Affine3f>& cameras, std::vector<ViewPoint>& vps)
{
  std::vector<VoxelNormals> voCenters;
  tree.VoxelOutsideCenters(voCenters);
  std::cout << "voxel outside " << voCenters.size() << std::endl;
  double minDist = tree.Resolution();
  double maxDist = distance;
  for (int i = 0; i < voCenters.size(); ++i)
  {
    Eigen::Vector3f center = voCenters[i].first;
    std::vector<Eigen::Vector3f> ops = voCenters[i].second;
    for (size_t j = 0; j < ops.size(); ++j)
    {
      Eigen::Vector3f nm = ops[j]-center;
      Eigen::Affine3f camera;
      ViewPoint vp;
      if (CameraPosition(tree, center, nm, minDist, maxDist, camera, vp))
      {
        cameras.push_back(camera);
        vps.push_back(vp);
      }
    }

    // extra normals
    if (ops.size() > 1)
    {
      double nx=0.0,ny=0.0,nz=0.0;
      int num = ops.size();
      for (size_t j = 0; j < num; ++j)
      {
        Eigen::Vector3f nm = (ops[j]-center).normalized();
        nx += nm.x();
        ny += nm.y();
        nz += nm.z();
      }
      nx /= num;
      ny /= num;
      nz /= num;

      Eigen::Vector3f anm(nx,ny,nz);
      Eigen::Affine3f camera;
      ViewPoint vp;
      if (CameraPosition(tree, center, anm, minDist, maxDist, camera, vp))
      {
        cameras.push_back(camera);
        vps.push_back(vp);
      }
    }
  }
}

void PCLViewPoint::CameraPositionWithVoxelAverageNormal(const PCLOctree& tree, double distance, std::vector<Eigen::Affine3f>& cameras, std::vector<ViewPoint>& vps)
{
  WSPointCloudPtr vCloud = tree.VoxelCentroidCloud();
  WSPointCloudNormalPtr vNormal = tree.VoxelAverageNormals();
  // find all camera poses
  for (size_t i = 0; i < vCloud->points.size(); ++i)
  {
    Eigen::Vector3f point(vCloud->points[i].x, vCloud->points[i].y, vCloud->points[i].z);
    Eigen::Vector3f nm(vNormal->points[i].normal_x, vNormal->points[i].normal_y, vNormal->points[i].normal_z);
    Eigen::Affine3f camera;
    ViewPoint vp;
    if (CameraPosition(tree,point, nm, tree.Resolution(), distance, camera,vp))
    {
      cameras.push_back(camera);
      vps.push_back(vp);
    }
  }
}

bool PCLViewPoint::CameraPosition(const PCLOctree& tree,
                    const Eigen::Vector3f& target,
                    const Eigen::Vector3f& normal,
                    double distMin, double distMax,
                    Eigen::Affine3f& camera,
                    ViewPoint& vp)
{
  bool bOk = true;
  double distance = distMax;
  Eigen::Vector3f nm = normal.normalized();
  Eigen::Vector3f pt = target + distance*nm;
  camera = CameraPose(pt,-nm);
  while (FilterViewPoint(tree,camera,vp))
  {
    distance = distance - 0.1;
    if (distance > distMin)
    {
      pt = target + distance*nm;
      camera = CameraPose(pt,-nm);
    }
    else
    {
      bOk = false;
      break;
    }
  }
  return bOk;
}

bool PCLViewPoint::FilterViewPoint(const PCLOctree& tree, const Eigen::Affine3f& camera, ViewPoint& vp)
{
  vp = CreateViewPoint(camera);

  double qr_x = vp.quadrotor_pose.pos_x;
  double qr_y = vp.quadrotor_pose.pos_y;
  double qr_z = vp.quadrotor_pose.pos_z;
  // height limitation
  if (qr_z < 0.31)
  {
    return true;
  }

  // bounding box collsion check
  Eigen::Vector3f minPt(qr_x-0.5, qr_y-0.5, qr_z-0.35);
  Eigen::Vector3f maxPt(qr_x+0.5, qr_y+0.5, qr_z+0.05);
  std::vector<int> intersets;
  if (tree.BoxSearch(minPt, maxPt, intersets) > 0)
  {
    // std::cout << intersets.size() << " filtered by collsion."<< std::endl;
    return true;
  }

  // no point in view frustum
  WSPointCloudPtr viewCloud(new WSPointCloud);
  WSPointCloudPtr vCloud = tree.VoxelCentroidCloud();
  FrustumCulling(vCloud,camera.matrix(),68,42,0.1,10.0,viewCloud);
  if (viewCloud->points.size() == 0)
  {
    // std::cout << " filtered by null view in frustum."<< std::endl;
    return true;
  }

  return false;
}

Eigen::Affine3f PCLViewPoint::ViewPoint2CameraPose(const ViewPoint& vp)
{
  Eigen::Vector3f t;
  Eigen::Quaternionf q;
  t.x() = vp.quadrotor_pose.pos_x;
  t.y() = vp.quadrotor_pose.pos_y;
  t.z() = vp.quadrotor_pose.pos_z;
  q.x() = vp.quadrotor_pose.ori_x;
  q.y() = vp.quadrotor_pose.ori_y;
  q.z() = vp.quadrotor_pose.ori_z;
  q.w() = vp.quadrotor_pose.ori_w;
  Eigen::Matrix3f rot = q.normalized().toRotationMatrix();

  Eigen::Matrix4f quadrotor_pose;
  quadrotor_pose.setIdentity();
  quadrotor_pose.block<3,3>(0,0)=rot;
  quadrotor_pose.block<3,1>(0,3)=t;

  double angle = vp.camera_angle;
  Eigen::Matrix4f quad2Cam = Quadrotor2Camera(angle);
  Eigen::Matrix4f camera_pose = quadrotor_pose*quad2Cam;
  Eigen::Affine3f res;
  res.matrix() = camera_pose;

  return res;
}

ViewPoint PCLViewPoint::CreateViewPoint(const Eigen::Affine3f& camera)
{
  Eigen::Matrix4f mat = camera.matrix();
  Eigen::Matrix3f affine = mat.topLeftCorner(3,3);
  Eigen::Quaternionf orientation(affine);
  double yaw =0,pitch=0,roll=0;
  Quaternion2EularAngles(orientation,yaw,pitch,roll);
  // std::cout << "yaw: " << yaw <<", pitch: " << pitch << ", roll: " << roll << std::endl;
  Eigen::Matrix4f quad2Cam = Quadrotor2Camera(pitch);
  Eigen::Matrix4f quadPoseMatrix = mat*quad2Cam.inverse();
  Cartesion quadPose;
  Matrix2Cartesion(quadPoseMatrix, quadPose);
  ViewPoint vp;
  vp.quadrotor_pose = quadPose;
  vp.camera_angle = pitch;
  return vp;
}

Eigen::Matrix4f PCLViewPoint::Quadrotor2Camera(double camera_angle)
{
  Eigen::Matrix4f quadBase;
  quadBase(0,0) = 1;
  quadBase(0,1) = 0;
  quadBase(0,2) = 0;
  quadBase(0,3) = 0.42;
  quadBase(1,0) = 0;
  quadBase(1,1) = 1;
  quadBase(1,2) = 0;
  quadBase(1,3) = 0;
  quadBase(2,0) = 0;
  quadBase(2,1) = 0;
  quadBase(2,2) = 1;
  quadBase(2,3) = 0;
  quadBase(3,0) = 0;
  quadBase(3,1) = 0;
  quadBase(3,2) = 0;
  quadBase(3,3) = 1;
  Eigen::Matrix4f cameraJoint;
  cameraJoint(0,0) = cos(camera_angle);
  cameraJoint(0,1) = 0;
  cameraJoint(0,2) = sin(camera_angle);
  cameraJoint(0,3) = 0.0358;
  cameraJoint(1,0) = 0;
  cameraJoint(1,1) = 1;
  cameraJoint(1,2) = 0;
  cameraJoint(1,3) = 0;
  cameraJoint(2,0) = -sin(camera_angle);
  cameraJoint(2,1) = 0;
  cameraJoint(2,2) = cos(camera_angle);
  cameraJoint(2,3) = 0;
  cameraJoint(3,0) = 0;
  cameraJoint(3,1) = 0;
  cameraJoint(3,2) = 0;
  cameraJoint(3,3) = 1;
  Eigen::Matrix4f quad2Camera = quadBase*cameraJoint;
  return quad2Camera;
}

Eigen::Affine3f PCLViewPoint::CameraPose(const Eigen::Vector3f& center, const Eigen::Vector3f& normal)
{
  //std::cout << "(" << center.x() <<","<< center.y()<<"," << center.z()<<")" << std::endl;
  Eigen::Vector3f nm = normal.normalized();
  //std::cout << "(" << nm.x() <<","<< nm.y()<<"," << nm.z()<<")" << std::endl;
  // nm = R*[0,0,1]'
  // c = 0.0, Cc = 1, Sc = 0
  // CcSbCa+ScSb = SbCa = nm.x()
  // ScSbCa-CcSb = -Sb = nm.y()
  // CbCa = nm.z()
  double c = 0.0; // yaw
  double a = asin(-(nm.y()));  // roll  angle should be in [-pi/2, pi/2]
  double b = atan2(nm.x(),nm.z()); // pitch
  //std::cout << "y:"<< a <<"p:"<< b <<"r:"<< c << std::endl;
  Eigen::Affine3f t = Eigen::Affine3f(Eigen::Translation3f(center.x(),center.y(),center.z()));
  Eigen::Affine3f rx = Eigen::Affine3f(Eigen::AngleAxisf(a,Eigen::Vector3f::UnitX()));
  Eigen::Affine3f ry = Eigen::Affine3f(Eigen::AngleAxisf(b,Eigen::Vector3f::UnitY()));
  Eigen::Affine3f rz = Eigen::Affine3f(Eigen::AngleAxisf(c,Eigen::Vector3f::UnitZ()));
  Eigen::Affine3f rot = rz*ry*rx;
  Eigen::Affine3f pose = t*rot;

  return pose;
}

bool PCLViewPoint::FrustumCulling(const WSPointCloudPtr cloud,
                               const Eigen::Matrix4f& cpose,
                               float hfov, float vfov,
                               float ndist, float fdist,
                               WSPointCloudPtr viewCloud)
{
  if (cloud == nullptr)
    return false;

  Eigen::Matrix4f poseNew = CameraPoseTransform(cpose);
  // frustum culling
  pcl::FrustumCulling<WSPoint> fc;
  fc.setInputCloud(cloud);
  fc.setCameraPose(poseNew);
  fc.setHorizontalFOV(hfov); // degree
  fc.setVerticalFOV(vfov); // degree
  fc.setNearPlaneDistance(ndist);
  fc.setFarPlaneDistance(fdist);
  fc.filter(*viewCloud);
  return true;
}

Eigen::Matrix4f PCLViewPoint::CameraPoseTransform(const Eigen::Matrix4f& mat)
{
  Eigen::Matrix4f cam2rot;
  cam2rot(0,0) = 0;
  cam2rot(0,1) = 0;
  cam2rot(0,2) = 1;
  cam2rot(0,3) = 0;
  cam2rot(1,0) = 0;
  cam2rot(1,1) = -1;
  cam2rot(1,2) = 0;
  cam2rot(1,3) = 0;
  cam2rot(2,0) = 1;
  cam2rot(2,1) = 0;
  cam2rot(2,2) = 0;
  cam2rot(2,3) = 0;
  cam2rot(3,0) = 0;
  cam2rot(3,1) = 0;
  cam2rot(3,2) = 0;
  cam2rot(3,3) = 1;
  return mat*cam2rot;
}

void PCLViewPoint::Matrix2Cartesion(const Eigen::Matrix4f& mat, Cartesion& quadPose)
{
  Eigen::Matrix3f affine = mat.topLeftCorner(3,3);
  Eigen::Quaternionf q(affine);
  quadPose.pos_x = mat(0,3);
  quadPose.pos_y = mat(1,3);
  quadPose.pos_z = mat(2,3);
  quadPose.ori_w = q.w();
  quadPose.ori_x = q.x();
  quadPose.ori_y = q.y();
  quadPose.ori_z = q.z();
}

void PCLViewPoint::Quaternion2EularAngles(const Eigen::Quaternionf& q, double& yaw, double& pitch, double& roll)
{
  double sr_cp = 2*(q.w()*q.x()+q.y()*q.z());
  double cr_cp = 1-2*(q.x()*q.x()+q.y()*q.y());
  roll = atan2(sr_cp,cr_cp);
  double sp = 2*(q.w()*q.y()-q.z()*q.x());
  pitch = asin(sp);
  if (abs(sp) >= 1)
    pitch = copysign(0.5*M_PI, sp);
  double sy_cp = 2*(q.w()*q.z()+q.x()*q.y());
  double cy_cp = 1-2*(q.y()*q.y()+q.z()*q.z());
  yaw = atan2(sy_cp, cy_cp);
}


void PCLViewPoint::Save2File(const std::string& output,
                             std::vector<ViewPoint>& vps,
                             std::map<int, std::vector<int> >& voxelMap)
{
  std::ofstream tFile(output);
  for (size_t i = 0; i < vps.size(); ++i)
  {
    ViewPoint viewpt = vps[i];
    // each line: viewpoint_idx px py pz ox oy oz ow angle voxel_indices ... \n
    tFile << i << " " << viewpt.quadrotor_pose.pos_x << " "
                      << viewpt.quadrotor_pose.pos_y << " "
                      << viewpt.quadrotor_pose.pos_z << " "
                      << viewpt.quadrotor_pose.ori_x << " "
                      << viewpt.quadrotor_pose.ori_y << " "
                      << viewpt.quadrotor_pose.ori_z << " "
                      << viewpt.quadrotor_pose.ori_w << " "
                      << viewpt.camera_angle << " ";
    // std::cout << viewVoxelMap[i].size() << std::endl;
    for (size_t j = 0; j < voxelMap[i].size(); ++j)
    {
      int vIndex = voxelMap[i][j];
      tFile << vIndex;
      if (j == voxelMap[i].size()-1)
        tFile << "\n";
      else
        tFile << " ";
    }
  }
  tFile.close();
}

void PCLViewPoint::LoadTrajectory(const std::string& input, std::vector<ViewPoint>& vps)
{
  std::ifstream tFile(input);
  std::string line;
  while (std::getline(tFile, line))
  {
    std::stringstream linestream(line);
    std::string index,px,py,pz,ox,oy,oz,ow,angle;
    linestream >> index >> px >> py >> pz >> ox >> oy >> oz >> ow >> angle;
    ViewPoint vp;
    vp.quadrotor_pose.pos_x = std::stod(px);
    vp.quadrotor_pose.pos_y = std::stod(py);
    vp.quadrotor_pose.pos_z = std::stod(pz);
    vp.quadrotor_pose.ori_x = std::stod(ox);
    vp.quadrotor_pose.ori_y = std::stod(oy);
    vp.quadrotor_pose.ori_z = std::stod(oz);
    vp.quadrotor_pose.ori_w = std::stod(ow);
    vp.camera_angle = std::stod(angle);
    vps.push_back(vp);
  }
  tFile.close();
}
