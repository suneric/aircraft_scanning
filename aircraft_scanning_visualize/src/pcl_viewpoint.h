#ifndef _ASV3D_PCL_VIEWPOINT_H_
#define _ASV3D_PCL_VIEWPOINT_H_

#include "pcl_octree.h"

namespace asv3d
{
  struct Cartesion
  {
    Cartesion(){pos_x=pos_y=pos_z=ori_x=ori_y=ori_z=ori_w=0;}
    Cartesion(const Cartesion& p)
    {
      pos_x = p.pos_x;
      pos_y = p.pos_y;
      pos_z = p.pos_z;
      ori_x = p.ori_x;
      ori_y = p.ori_y;
      ori_z = p.ori_z;
      ori_w = p.ori_w;
    }
    Cartesion& operator=(const Cartesion& p)
    {
      pos_x = p.pos_x;
      pos_y = p.pos_y;
      pos_z = p.pos_z;
      ori_x = p.ori_x;
      ori_y = p.ori_y;
      ori_z = p.ori_z;
      ori_w = p.ori_w;
      return *this;
    }
    double pos_x;
    double pos_y;
    double pos_z;
    double ori_x;
    double ori_y;
    double ori_z;
    double ori_w;
  };

  struct ViewPoint {
    ViewPoint(){quadrotor_pose = Cartesion();camera_angle=0.0;}
    ViewPoint(const ViewPoint& vp)
    {
      quadrotor_pose = vp.quadrotor_pose;
      camera_angle = vp.camera_angle;
    }
    ViewPoint& operator=(const ViewPoint& vp)
    {
      quadrotor_pose = vp.quadrotor_pose;
      camera_angle = vp.camera_angle;
      return *this;
    }
    Cartesion quadrotor_pose;
    double camera_angle; // camera joint angle
  };

  class PCLViewPoint{
  public:
    void GenerateCameraPositions(const PCLOctree& tree, double distance, std::vector<Eigen::Affine3f>& cameras, std::vector<ViewPoint>& vps, int type = 0);
    WSPointCloudPtr CameraViewVoxels(const PCLOctree& tree, const Eigen::Affine3f& camera, std::vector<int>& voxelInices);
    void Save2File(const std::string& output,
                   std::vector<ViewPoint>& vps,
                   std::map<int, std::vector<int> >& voxelMap);
    void LoadTrajectory(const std::string& input, std::vector<ViewPoint>& vps);
    Eigen::Affine3f ViewPoint2CameraPose(const ViewPoint& vp);


  private:
    void CameraPositionWithVoxelAverageNormal(const PCLOctree& tree, double distance, std::vector<Eigen::Affine3f>& cameras, std::vector<ViewPoint>& vps);
    void CameraPositionWithVoxelCube(const PCLOctree& tree, double distance, std::vector<Eigen::Affine3f>& cameras, std::vector<ViewPoint>& vps);
    bool CameraPosition(const PCLOctree& tree,
                        const Eigen::Vector3f& target,
                        const Eigen::Vector3f& normal,
                        double distMin, double distMax,
                        Eigen::Affine3f& camera,
                        ViewPoint& viewpoint);
    bool FilterViewPoint(const PCLOctree& tree, const Eigen::Affine3f& camera, ViewPoint& viewpoint);

    ViewPoint CreateViewPoint(const Eigen::Affine3f& camera);
    Eigen::Matrix4f Quadrotor2Camera(double camera_angle);

    Eigen::Affine3f CameraPose(const Eigen::Vector3f& center, const Eigen::Vector3f& normal);
    bool FrustumCulling(const WSPointCloudPtr cloud,
                        const Eigen::Matrix4f& camera,
                        float hfov, float vfov,
                        float ndist, float fdist,
                        WSPointCloudPtr viewPoints);
    // convert traditional x right, y down and z forward matrix to xforward,y up, z right.
    Eigen::Matrix4f CameraPoseTransform(const Eigen::Matrix4f& mat);

    void Matrix2Cartesion(const Eigen::Matrix4f& mat, Cartesion& vp);
    void Quaternion2EularAngles(const Eigen::Quaternionf& q, double& yaw, double& pitch, double& roll);
  };
};

#endif //_ASV3D_PCL_VIEWPOINT_H_
