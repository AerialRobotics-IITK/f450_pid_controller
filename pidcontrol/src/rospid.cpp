#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

using namespace std;
using namespace Eigen;

double vx1 = 1, vy1 = 1; //defined for setpoint velocity from controller

Matrix<float, 3, 3> R;
Matrix<float, 3, 3> R_inv;
Matrix<float, 3, 1> VelMat;

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  double x, y, z, w;

  x = msg->orientation.x;
  y = msg->orientation.y;
  z = msg->orientation.z;
  w = msg->orientation.w;
  Quaternionf quat;
  quat = Eigen::Quaternionf(x, y, z, w);
  R = quat.toRotationMatrix();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_pid_control");
  ros::NodeHandle nh;

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 100, imuCallback);

  ros::Publisher cmd_att_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 100);

  geometry_msgs::PoseStamped att_sp;
  geometry_msgs::Quaternion quaternion;
  geometry_msgs::TransformStamped transformStamped;

  ros::Rate rate(20.0);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  float kp_x = 1, kp_y = 1, v_kp_x = 1, v_ki_x = 0, v_kd_x = 0, v_kp_y = 1, v_ki_y = 0, v_kd_y = 0, vx = 0, vy = 0, x_des = 0, y_des = 0;
  double dt, current_time, last_time, mav_x = 0, mav_y = 0;
  double error_x, error_y, iTerm_x, iTerm_y, dTerm_x, dTerm_y, last_input_x, last_input_y, ang_x, ang_y;

  Quaternionf quat;

  while (ros::ok())
  {
    nh.getParam("/aruco_pid_control/x_des", x_des);
    nh.getParam("/aruco_pid_control/y_des", y_des);
    nh.getParam("/aruco_pid_control/kp_x", kp_x);
    nh.getParam("/aruco_pid_control/kp_y", kp_y);
    nh.getParam("/aruco_pid_control/v_kp_x", v_kp_x);
    nh.getParam("/aruco_pid_control/v_ki_x", v_ki_x);
    nh.getParam("/aruco_pid_control/v_kd_x", v_kd_x);
    nh.getParam("/aruco_pid_control/v_kp_y", v_kp_y);
    nh.getParam("/aruco_pid_control/v_ki_y", v_ki_y);
    nh.getParam("/aruco_pid_control/v_kd_y", v_kd_y);

    try
    {
      transformStamped = tfBuffer.lookupTransform("world", "camera", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    mav_x = transformStamped.transform.translation.x;
    mav_y = transformStamped.transform.translation.y;

    quaternion = transformStamped.transform.rotation;

    quat = Eigen::Quaternionf(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    R = quat.toRotationMatrix();
    R_inv = R.inverse();

    VelMat(0, 0) = kp_x * (mav_x - x_des); //set point is set to (0,0) in
    VelMat(1, 0) = kp_y * (mav_y - y_des); //world frame
    VelMat(2, 0) = 0;

    VelMat = R_inv * VelMat;

    vx = VelMat(0, 0); // ToDO:change from camera frame
    vy = VelMat(1, 0); // to quadcopter frame(axis)

    current_time = ros::Time::now().toSec();
    dt = current_time - last_time;

    if (dt != 0)
    {
      error_x = (vx - vx1);
      iTerm_x += error_x * dt;
      dTerm_x = (error_x - last_input_x) / dt;
      last_input_x = error_x;

      ang_x = v_kp_x * error_x + v_ki_x * iTerm_x - v_kd_x * dTerm_x;

      error_y = (vy - vy1);
      iTerm_y += error_y * dt;
      dTerm_y = (error_y - last_input_y) / dt;
      last_input_y = error_y;

      ang_y = v_kp_y * error_y + v_ki_y * iTerm_y - v_kd_y * dTerm_y;
    }
    last_time = current_time;

    att_sp.pose.position.x = ang_x;
    att_sp.pose.position.y = ang_y;

    cmd_att_pub.publish(att_sp);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}