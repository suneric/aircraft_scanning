//=================================================================================================
// Copyright (c) 2012-2016, Institute of Flight Systems and Automatic Control,
// Technische Universität Darmstadt.
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of hector_quadrotor nor the names of its contributors
//       may be used to endorse or promote products derived from this software
//       without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <hector_uav_msgs/YawrateCommand.h>
#include <hector_uav_msgs/ThrustCommand.h>
#include <hector_uav_msgs/AttitudeCommand.h>
#include <hector_uav_msgs/TakeoffAction.h>
#include <hector_uav_msgs/LandingAction.h>
#include <hector_uav_msgs/PoseAction.h>
#include <hector_quadrotor_interface/limiters.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <actionlib/client/simple_action_client.h>

namespace hector_quadrotor
{

class Teleop
{
private:
  typedef actionlib::SimpleActionClient<hector_uav_msgs::LandingAction> LandingClient;
  typedef actionlib::SimpleActionClient<hector_uav_msgs::TakeoffAction> TakeoffClient;
  typedef actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> PoseClient;

  ros::NodeHandle node_handle_;
  ros::Subscriber joy_subscriber_;
  ros::Publisher velocity_publisher_, attitude_publisher_, yawrate_publisher_, thrust_publisher_, camjoint_publisher_;
  ros::ServiceClient motor_enable_service_;
  boost::shared_ptr<LandingClient> landing_client_;
  boost::shared_ptr<TakeoffClient> takeoff_client_;
  boost::shared_ptr<PoseClient> pose_client_;

  geometry_msgs::PoseStamped pose_;
  double yaw_;
  double cam_angle_;

  struct Axis
  {
    Axis()
      : axis(0), factor(0.0), offset(0.0)
    {}

    int axis;
    double factor;
    double offset;
  };

  struct Button
  {
    Button()
      : button(0)
    {}

    int button;
  };

  struct
  {
    Axis x;
    Axis y;
    Axis z;
    Axis thrust;
    Axis yaw;
  } axes_;

  struct
  {
    Button slow;
    Button go;
    Button stop;
    Button interrupt;
    Button camup;
    Button camdown;
  } buttons_;

  double slow_factor_;
  std::string base_link_frame_, base_stabilized_frame_, world_frame_;

public:
  Teleop()
  {
    ros::NodeHandle private_nh("~");

    private_nh.param<int>("x_axis", axes_.x.axis, 5);
    private_nh.param<int>("y_axis", axes_.y.axis, 4);
    private_nh.param<int>("z_axis", axes_.z.axis, 2);
    private_nh.param<int>("thrust_axis", axes_.thrust.axis, 2);
    private_nh.param<int>("yaw_axis", axes_.yaw.axis, 1);
    private_nh.param<double>("yaw_velocity_max", axes_.yaw.factor, 90.0);

    private_nh.param<int>("slow_button", buttons_.slow.button, 1);
    private_nh.param<int>("go_button", buttons_.go.button, 4);
    private_nh.param<int>("stop_button", buttons_.stop.button, 2);
    private_nh.param<int>("interrupt_button", buttons_.interrupt.button, 3);
    private_nh.param<double>("slow_factor", slow_factor_, 0.2);

    private_nh.param<int>("camup_button", buttons_.camup.button, 6);
    private_nh.param<int>("camdown_button", buttons_.camdown.button,5);

    private_nh.param<double>("x_velocity_max", axes_.x.factor, 2.0);
    private_nh.param<double>("y_velocity_max", axes_.y.factor, 2.0);
    private_nh.param<double>("z_velocity_max", axes_.z.factor, 2.0);

    joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 1,
                                                               boost::bind(&Teleop::joyPoseCallback, this, _1));

    ros::NodeHandle robot_nh;
    robot_nh.param<std::string>("world_frame", world_frame_, "world");
    robot_nh.param<std::string>("base_stabilized_frame", base_stabilized_frame_, "base_stabilized");
    robot_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");

    pose_.pose.position.x = 0.0;
    pose_.pose.position.y = -30.0;
    pose_.pose.position.z = 0.3;
    pose_.pose.orientation.x = 0;
    pose_.pose.orientation.y = 0;
    pose_.pose.orientation.z = 0;
    pose_.pose.orientation.w = 1;
    cam_angle_ = 0.0;

    motor_enable_service_ = robot_nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
    takeoff_client_ = boost::shared_ptr<TakeoffClient>(new TakeoffClient(robot_nh, "action/takeoff"));
    landing_client_ = boost::shared_ptr<LandingClient>(new LandingClient(robot_nh, "action/landing"));
    pose_client_ = boost::shared_ptr<PoseClient>(new PoseClient(robot_nh, "action/pose"));

    std::string camjointCmd = "uav_scanning/camera_pose_controller/command";
    camjoint_publisher_ = robot_nh.advertise<std_msgs::Float64>(camjointCmd, 1);
  }

  ~Teleop()
  {
    stop();
  }

  void joyPoseCallback(const sensor_msgs::JoyConstPtr &joy)
  {
    ros::Time now = ros::Time::now();
    double dt = 0.0;
    if (!pose_.header.stamp.isZero()) {
      dt = std::max(0.0, std::min(1.0, (now - pose_.header.stamp).toSec()));
    }

    if (getButton(joy, buttons_.go))
    {
      pose_.header.stamp = now;
      pose_.header.frame_id = world_frame_;
      pose_.pose.position.x += (cos(yaw_) * getAxis(joy, axes_.x) - sin(yaw_) * getAxis(joy, axes_.y)) * dt;
      pose_.pose.position.y += (cos(yaw_) * getAxis(joy, axes_.y) + sin(yaw_) * getAxis(joy, axes_.x)) * dt;
      pose_.pose.position.z += getAxis(joy, axes_.z) * dt;
      yaw_ += getAxis(joy, axes_.yaw) * M_PI/180.0 * dt;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw_);
      pose_.pose.orientation = tf2::toMsg(q);

      hector_uav_msgs::PoseGoal goal;
      goal.target_pose = pose_;
      pose_client_->sendGoal(goal);
    }
    if (getButton(joy, buttons_.interrupt))
    {
      pose_client_->cancelGoalsAtAndBeforeTime(ros::Time::now());
    }
    if (getButton(joy, buttons_.stop))
    {
      landing_client_->sendGoalAndWait(hector_uav_msgs::LandingGoal(), ros::Duration(10.0), ros::Duration(10.0));
    }
    if (getButton(joy, buttons_.camup))
    {
      cam_angle_ = changeCamJoint(true);
    }
    if (getButton(joy, buttons_.camdown))
    {
      cam_angle_ = changeCamJoint(false);
    }
  }

  double getAxis(const sensor_msgs::JoyConstPtr &joy, const Axis &axis)
  {
    if (axis.axis == 0 || std::abs(axis.axis) > joy->axes.size())
    {
      ROS_ERROR_STREAM("Axis " << axis.axis << " out of range, joy has " << joy->axes.size() << " axes");
      return 0;
    }

    double output = std::abs(axis.axis) / axis.axis * joy->axes[std::abs(axis.axis) - 1] * axis.factor + axis.offset;
    return output;
  }

  bool getButton(const sensor_msgs::JoyConstPtr &joy, const Button &button)
  {
    if (button.button <= 0 || button.button > joy->buttons.size())
    {
      ROS_ERROR_STREAM("Button " << button.button << " out of range, joy has " << joy->buttons.size() << " buttons");
      return false;
    }
    return joy->buttons[button.button - 1] > 0;
  }

  bool enableMotors(bool enable)
  {
    if (!motor_enable_service_.waitForExistence(ros::Duration(5.0)))
    {
      ROS_WARN("Motor enable service not found");
      return false;
    }

    hector_uav_msgs::EnableMotors srv;
    srv.request.enable = enable;
    return motor_enable_service_.call(srv);
  }

  double changeCamJoint(bool up)
  {
    double newangle = cam_angle_;
    double rate = 0.01;
    if (up)
    {
      if (newangle < 1.5708)
      {
        newangle += rate;
      }
      else
      {
        newangle = 1.57;
      }
    }
    else
    {
      if (newangle > -1.5708)
      {
        newangle -= rate;
      }
      else
      {
        newangle = -1.5708;
      }
    }
    std_msgs::Float64 msg;
    msg.data = newangle;
    camjoint_publisher_.publish(msg);
    return newangle;
  }

  void stop()
  {
    if (velocity_publisher_.getNumSubscribers() > 0)
    {
      velocity_publisher_.publish(geometry_msgs::TwistStamped());
    }
    if (attitude_publisher_.getNumSubscribers() > 0)
    {
      attitude_publisher_.publish(hector_uav_msgs::AttitudeCommand());
    }
    if (thrust_publisher_.getNumSubscribers() > 0)
    {
      thrust_publisher_.publish(hector_uav_msgs::ThrustCommand());
    }
    if (yawrate_publisher_.getNumSubscribers() > 0)
    {
      yawrate_publisher_.publish(hector_uav_msgs::YawrateCommand());
    }
    if (camjoint_publisher_.getNumSubscribers() > 0)
    {
      camjoint_publisher_.publish(std_msgs::Float64());
    }
  }
};

} // namespace hector_quadrotor

int main(int argc, char **argv)
{
  ros::init(argc, argv, "quadrotor_teleop");

  hector_quadrotor::Teleop teleop;
  ros::spin();

  return 0;
}
