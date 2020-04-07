#include "pcl_viewpoint.h"
#include "pcl_filter.h"
#include <math.h>
#include <fstream>
#include <sstream>

#include <pcl/filters/frustum_culling.h>
#include <pcl/features/normal_3d.h>

using namespace asv3d;

void PCLViewPoint::GenerateCameraPositions(
  const PCLOctree& tree,
  double distance,
  const Eigen::Vector3f& refNormal,
  std::vector<Eigen::Affine3f>& cameras,
  std::vector<ViewPoint>& vps,
  int type)
{
  if (type == 0)
    CameraPositionWithVoxelAverageNormal(tree, distance,refNormal,cameras,vps);
  // else
  //   CameraPositionWithVoxelCube(tree, distance, cameras);
}

void PCLViewPoint::CameraPositionWithVoxelAverageNormal(
  const PCLOctree& tree,
  double distance,
  const Eigen::Vector3f& refNormal,
  std::vector<Eigen::Affine3f>& cameras,
  std::vector<ViewPoint>& vps)
{
  WSPointCloudPtr vCloud = tree.VoxelCentroidCloud();
  WSPointCloudNormalPtr vNormal = tree.VoxelAverageNormals(refNormal);
  for (size_t i = 0; i < vCloud->points.size(); ++i)
  {
    Eigen::Vector3f point(vCloud->points[i].x, vCloud->points[i].y, vCloud->points[i].z);
    Eigen::Vector3f nm(vNormal->points[i].normal_x, vNormal->points[i].normal_y, vNormal->points[i].normal_z);
    Eigen::Affine3f camera;
    ViewPoint vp;
    if (CameraPosition(tree,point,nm,distance,camera,vp))
    {
      cameras.push_back(camera);
      vps.push_back(vp);
    }
  }
}

bool PCLViewPoint::CameraPosition(const PCLOctree& tree,
                    const Eigen::Vector3f& target,
                    const Eigen::Vector3f& normal,
                    double distance,
                    Eigen::Affine3f& camera,
                    ViewPoint& vp)
{
  bool bOk = true;
  Eigen::Vector3f nm = normal.normalized();
  Eigen::Vector3f pt = target + distance*nm;
  while (pt.z() < 0.5)
  {
    distance = 0.9*distance;
    pt = target + distance*nm;
  }
  double alpha = 0.0, beta = 0.0; // yaw and pitch
  camera = CameraPose(pt,-nm, alpha, beta);
  vp = CreateViewPoint(camera,alpha,beta);
  return bOk;
}

ViewPoint PCLViewPoint::CreateViewPoint(const Eigen::Affine3f& camera,double alpha,double beta)
{
  Eigen::Matrix4f mat = camera.matrix();
  Eigen::Vector3f center(mat(0,3),mat(1,3),mat(2,3));
  Eigen::Matrix4f quadrotor;
  quadrotor(0,0) = cos(alpha);
  quadrotor(0,1) = -sin(alpha);
  quadrotor(0,2) = 0;
  quadrotor(0,3) = center.x()-0.4558*cos(alpha);
  quadrotor(1,0) = sin(alpha);
  quadrotor(1,1) = cos(alpha);
  quadrotor(1,2) = 0;
  quadrotor(1,3) = center.y()-0.4558*sin(alpha);
  quadrotor(2,0) = 0;
  quadrotor(2,1) = 0;
  quadrotor(2,2) = 1;
  quadrotor(2,3) = center.z();
  quadrotor(3,0) = 0;
  quadrotor(3,1) = 0;
  quadrotor(3,2) = 0;
  quadrotor(3,3) = 1;
  Cartesion quadPose;
  Matrix2Cartesion(quadrotor, quadPose);
  ViewPoint vp;
  vp.quadrotor_pose = quadPose;
  vp.camera_angle = beta;
  return vp;
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

// give a position of camera and the direction of camera facing
// calculate the orientation of camera
// given the condition that rolling of camera is 0 as uav is hovering
Eigen::Affine3f PCLViewPoint::CameraPose(const Eigen::Vector3f& center,
                                         const Eigen::Vector3f& normal,
                                         double& alpha,double& beta)
{
  Eigen::Vector3f nm = normal.normalized();
  beta = acos(nm.z())-M_PI/2;
  alpha = 0.0;
  if (nm.x() != 0)
    alpha = atan2(nm.y(),nm.x());

  Eigen::Matrix4f mat;
  mat(0,0) = sin(alpha);
  mat(0,1) = cos(alpha)*cos(beta+M_PI/2);
  mat(0,2) = cos(alpha)*sin(beta+M_PI/2);
  mat(0,3) = center.x();
  mat(1,0) = -cos(alpha);
  mat(1,1) = sin(alpha)*cos(beta+M_PI/2);
  mat(1,2) = sin(alpha)*sin(beta+M_PI/2);
  mat(1,3) = center.y();
  mat(2,0) = 0;
  mat(2,1) = -sin(beta+M_PI/2);
  mat(2,2) = cos(beta+M_PI/2);
  mat(2,3) = center.z();
  mat(3,0) = 0;
  mat(3,1) = 0;
  mat(3,2) = 0;
  mat(3,3) = 1;
  Eigen::Affine3f res;
  res.matrix() = mat;
  return res;
}

WSPointCloudPtr PCLViewPoint::CameraViewVoxels(const PCLOctree& tree, const Eigen::Affine3f& camera, std::vector<int>& voxelIndices)
{
  Eigen::Matrix4f camMatrix = camera.matrix();
  Eigen::Vector3f cameraPt(camMatrix(0,3),camMatrix(1,3),camMatrix(2,3));
  // get voxels in camera frustum culling
  WSPointCloudPtr viewCloud(new WSPointCloud);
  if(FrustumCulling(tree.VoxelCentroidCloud(),camera.matrix(),68,42,0.1,5.0,viewCloud))
  {
    std::vector<WSPoint> visibleVoxels;
    for (int i = 0; i < viewCloud->points.size(); ++i)
    {
      WSPoint c = viewCloud->points[i];
      if (IsVisibleVoxel(tree,cameraPt,Eigen::Vector3f(c.x,c.y,c.z)))
        visibleVoxels.push_back(c);
    }

    // voxel index
    std::vector<WSPoint> visibleCentroids;
    for (size_t i = 0; i < visibleVoxels.size(); ++i)
    {
      int voxelIndex = tree.VoxelIndex(visibleVoxels[i]);
      if (std::find(voxelIndices.begin(),voxelIndices.end(),voxelIndex) == voxelIndices.end())
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
  else
  {
    std::cout << "no voxels in camera view" << std::endl;
    return nullptr;
  }
}

// check if voxel is blocked by other voxel in the view with checking all of its side
bool PCLViewPoint::IsVisibleVoxel(const PCLOctree& tree, const Eigen::Vector3f& camera, const Eigen::Vector3f& c)
{
  int visibleSide = 0;
  double halfVLen = 0.5*tree.VoxelSideLength();
  std::vector<Eigen::Vector3f> checkPoints;
  checkPoints.push_back(Eigen::Vector3f(c.x()-halfVLen,c.y(),c.z()));
  checkPoints.push_back(Eigen::Vector3f(c.x()+halfVLen,c.y(),c.z()));
  checkPoints.push_back(Eigen::Vector3f(c.x(),c.y()-halfVLen,c.z()));
  checkPoints.push_back(Eigen::Vector3f(c.x(),c.y()+halfVLen,c.z()));
  checkPoints.push_back(Eigen::Vector3f(c.x(),c.y(),c.z()-halfVLen));
  checkPoints.push_back(Eigen::Vector3f(c.x(),c.y(),c.z()+halfVLen));
  for (int i = 0; i < checkPoints.size(); ++i)
  {
    std::vector<WSPoint> intesects;
    int nRes = tree.IntersectedOccupiedVoxels(camera, c);
    //std::cout << "intersects " << nRes << std::endl;
    if (nRes <= 1) // consider the two voxel share the same edge
      visibleSide++;
  }
  //std::cout << "visible side " << visibleSide << std::endl;
  return visibleSide > 0;
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

bool PCLViewPoint::FilterViewPoint(const PCLOctree& tree, const Eigen::Affine3f& camera, const ViewPoint& vp)
{
  double qr_x = vp.quadrotor_pose.pos_x;
  double qr_y = vp.quadrotor_pose.pos_y;
  double qr_z = vp.quadrotor_pose.pos_z;

  // bounding box collsion check
  Eigen::Vector3f minPt(qr_x-0.5, qr_y-0.5, qr_z-0.35);
  Eigen::Vector3f maxPt(qr_x+0.5, qr_y+0.5, qr_z+0.05);
  std::vector<int> intersets;
  if (tree.BoxSearch(minPt, maxPt, intersets) > 0)
    return true;

  // no point in view frustum
  std::vector<int> voxelIndices;
  CameraViewVoxels(tree,camera,voxelIndices);
  if (voxelIndices.empty())
    return true;

  return false;
}

// This assume a coordinate system where X is forward, Y is up and Z is right.
// to convert from the traditional camera coordinate syste (X is right, Y is down, Z forward)
// [0,0,1,0
//  0,-1,0,0
//  1,0,0,0
//  0,0,0,1]
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

// void PCLViewPoint::CameraPositionWithVoxelCube(const PCLOctree& tree, double distance, std::vector<Eigen::Affine3f>& cameras)
// {
//   std::vector<VoxelNormals> voCenters;
//   tree.VoxelOutsideCenters(voCenters);
//   std::cout << "voxel outside " << voCenters.size() << std::endl;
//   for (int i = 0; i < voCenters.size(); ++i)
//   {
//     Eigen::Vector3f center = voCenters[i].first;
//     std::vector<Eigen::Vector3f> ops = voCenters[i].second;
//     for (size_t j = 0; j < ops.size(); ++j)
//     {
//       Eigen::Vector3f nm = ops[j]-center;
//       Eigen::Affine3f camera;
//       ViewPoint vp;
//       if (CameraPosition(tree, center, nm, distance, camera))
//       {
//         cameras.push_back(camera);
//       }
//     }
//
//     // extra normals
//     if (ops.size() > 1)
//     {
//       double nx=0.0,ny=0.0,nz=0.0;
//       int num = ops.size();
//       Eigen::Vector3f nm(0,0,0);
//       for (size_t j = 0; j < num; ++j)
//         nm += Eigen::Vector3f(ops[j]-center).normalized();
//       Eigen::Vector3f anm = nm/num;
//       Eigen::Affine3f camera;
//       if (CameraPosition(tree, center, anm, distance, camera))
//         cameras.push_back(camera);
//     }
//   }
// }

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
  cameraJoint(0,0) = cos(M_PI/2+camera_angle);
  cameraJoint(0,1) = 0;
  cameraJoint(0,2) = sin(M_PI/2+camera_angle);
  cameraJoint(0,3) = 0.0358;
  cameraJoint(1,0) = 0;
  cameraJoint(1,1) = 1;
  cameraJoint(1,2) = 0;
  cameraJoint(1,3) = 0;
  cameraJoint(2,0) = -sin(M_PI/2+camera_angle);
  cameraJoint(2,1) = 0;
  cameraJoint(2,2) = cos(M_PI/2+camera_angle);
  cameraJoint(2,3) = 0;
  cameraJoint(3,0) = 0;
  cameraJoint(3,1) = 0;
  cameraJoint(3,2) = 0;
  cameraJoint(3,3) = 1;
  Eigen::Matrix4f rotate;
  rotate(0,0) = cos(-M_PI/2);
  rotate(0,1) = -sin(-M_PI/2);
  rotate(0,2) = 0;
  rotate(0,3) = 0;
  rotate(1,0) = sin(-M_PI/2);
  rotate(1,1) = cos(-M_PI/2);
  rotate(1,2) = 0;
  rotate(1,3) = 0;
  rotate(2,0) = 0;
  rotate(2,1) = 0;
  rotate(2,2) = 1;
  rotate(2,3) = 0;
  rotate(3,0) = 0;
  rotate(3,1) = 0;
  rotate(3,2) = 0;
  rotate(3,3) = 1;
  Eigen::Matrix4f quad2Camera = quadBase*cameraJoint*rotate;
  return quad2Camera;
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

// void PCLViewPoint::Quaternion2EularAngles(const Eigen::Quaternionf& q, double& yaw, double& pitch, double& roll)
// {
//   double sr_cp = 2*(q.w()*q.x()+q.y()*q.z());
//   double cr_cp = 1-2*(q.x()*q.x()+q.y()*q.y());
//   roll = atan2(sr_cp,cr_cp);
//   double sp = 2*(q.w()*q.y()-q.z()*q.x());
//   pitch = asin(sp);
//   if (abs(sp) >= 1)
//     pitch = copysign(0.5*M_PI, sp);
//   double sy_cp = 2*(q.w()*q.z()+q.x()*q.y());
//   double cy_cp = 1-2*(q.y()*q.y()+q.z()*q.z());
//   yaw = atan2(sy_cp, cy_cp);
// }
