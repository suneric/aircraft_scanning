#include "pcl_viewpoint.h"
#include "pcl_filter.h"
#include <math.h>

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
      int nRes = tree.GetIntersectedVoxelCenters(cameraPt, endPt-cameraPt, 1, intesects);
      if (nRes == 1 && intesects.size() == 1)
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
    visibleVoxelCloud->points.resize(visibleVoxels.size());
    visibleVoxelCloud->points.assign(visibleCentroids.begin(), visibleCentroids.end());
    return visibleVoxelCloud;
  }
  return nullptr;
}

void PCLViewPoint::GenerateCameraPositions(const PCLOctree& tree,double distance,std::vector<Eigen::Affine3f>& cameras)
{
  WSPointCloudPtr vCloud = tree.VoxelCentroidCloud();
  WSPointCloudNormalPtr vNormal = tree.VoxelAverageNormals();
  // find all camera poses
  for (size_t i = 0; i < vCloud->points.size(); ++i)
  {
    Eigen::Vector3f point(vCloud->points[i].x, vCloud->points[i].y, vCloud->points[i].z);
    Eigen::Vector3f nm(vNormal->points[i].normal_x, vNormal->points[i].normal_y, vNormal->points[i].normal_z);
    Eigen::Vector3f camera;
    if (CameraPosition(point, nm, tree.Resolution(), distance, camera))
      cameras.push_back(CameraPose(camera,-nm));
  }
}

bool PCLViewPoint::CameraPosition(const Eigen::Vector3f& target, const Eigen::Vector3f& normal, double distMin, double distMax, Eigen::Vector3f& camera)
{
  bool bOk = true;
  double distance = distMax;
  Eigen::Vector3f nm = normal;
  Eigen::Vector3f unitNormal = nm.normalized();
  camera = target + distance*unitNormal;
  while (camera.z() < 0.35)
  {
    distance = distance - 0.1;
    if (distance > distMin)
      camera = target + distance*unitNormal;
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

ViewPoint PCLViewPoint::CreateViewPoint(const Eigen::Affine3f& camera)
{
  Eigen::Matrix4f mat = camera.matrix();
  Eigen::Matrix3f affine = mat.topLeftCorner(3,3);
  Eigen::Quaternionf orientation(affine);
  double yaw =0,pitch=0,roll=0;
  Quaternion2EularAngles(orientation,yaw,pitch,roll);
  // std::cout << "yaw: " << yaw <<", pitch: " << pitch << ", roll: " << roll << std::endl;
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
  cameraJoint(0,0) = cos(pitch);
  cameraJoint(0,1) = 0;
  cameraJoint(0,2) = sin(pitch);
  cameraJoint(0,3) = 0.0358;
  cameraJoint(1,0) = 0;
  cameraJoint(1,1) = 1;
  cameraJoint(1,2) = 0;
  cameraJoint(1,3) = 0;
  cameraJoint(2,0) = -sin(pitch);
  cameraJoint(2,1) = 0;
  cameraJoint(2,2) = cos(pitch);
  cameraJoint(2,3) = 0;
  cameraJoint(3,0) = 0;
  cameraJoint(3,1) = 0;
  cameraJoint(3,2) = 0;
  cameraJoint(3,3) = 1;

  Eigen::Matrix4f quad2Camera = quadBase*cameraJoint;
  Eigen::Matrix4f quadPoseMatrix = mat*quad2Camera.inverse();
  Cartesion quadPose;
  Matrix2Cartesion(quadPoseMatrix, quadPose);

  ViewPoint vp;
  vp.quadrotor_pose = quadPose;
  vp.camera_angle = pitch;
  return vp;
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
  double qw = 0.5*sqrt(1+mat(0,0)+mat(1,1)+mat(2,2));
  double qx = (mat(2,1)-mat(1,2))/(4*qw);
  double qy = (mat(0,2)-mat(2,0))/(4*qw);
  double qz = (mat(1,0)-mat(0,1))/(4*qw);
  quadPose.pos_x = mat(0,3);
  quadPose.pos_y = mat(1,3);
  quadPose.pos_z = mat(2,3);
  quadPose.ori_w = qw;
  quadPose.ori_x = qx;
  quadPose.ori_y = qy;
  quadPose.ori_z = qz;
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
