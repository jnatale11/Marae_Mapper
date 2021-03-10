// ROS Imu Node 

#include <stdio.h>
#include <iostream>
#include <string>
#include "marae_bno055.c"
#include <rosbag/bag.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

using namespace std;

void set_orientation(geometry_msgs::Quaternion* orientation, struct marae_data_t imu_data) {
    orientation->x = imu_data.quaternion.x;
    orientation->y = imu_data.quaternion.y;
    orientation->z = imu_data.quaternion.z;
    orientation->w = imu_data.quaternion.w;
    //printf("quaternion x: %f, y: %f, z: %f, w: %f\n", orientation->x, orientation->y, orientation->z, orientation->w);
    return;
}

void set_ang_vel(geometry_msgs::Vector3* ang_vel, struct marae_data_t imu_data) {
    ang_vel->x = imu_data.ang_vel.x;
    ang_vel->y = imu_data.ang_vel.y;
    ang_vel->z = imu_data.ang_vel.z;
    //printf("ang vel (rps) x: %f, y: %f, z: %f\n", ang_vel->x, ang_vel->y, ang_vel->z);
    return;
}

void set_lin_accel(geometry_msgs::Vector3* lin_accel, struct marae_data_t imu_data) {
    lin_accel->x = imu_data.lin_accel.x;
    lin_accel->y = imu_data.lin_accel.y;
    lin_accel->z = imu_data.lin_accel.z;
    printf("lin accel x: %f, y: %f, z: %f\n", lin_accel->x, lin_accel->y, lin_accel->z);
    return;
}

int main(int argc, char *argv[])
{
  printf("Hi Juicy Motion Sensor...\n");
  string bag_suffix = argv[1];
  int tot_frames = atoi(argv[2]);
  string bag_fname = "marae_mono_inertial_" + bag_suffix + ".bag";
  int i, func_val;
  struct marae_data_t imu_data = {};
  rosbag::Bag bag;
  sensor_msgs::Imu imu_msg;
  geometry_msgs::Quaternion orientation;
  geometry_msgs::Vector3 angular_velocity;
  geometry_msgs::Vector3 linear_acceleration;
  //TODO may want to fill in cov matrix in future for better accuracy
  boost::array<double, 9> cov_matrix_zero = {0,0,0,0,0,0,0,0,0};

  bag.open(bag_fname, rosbag::bagmode::Write);
  
  func_val = init_and_calib_bno055();
  printf("Init and Calib returned %d\n", func_val);
 
  ros::Time::init();

  for (i=0; i< tot_frames; i++) {
      //TODO may want to verify at some point the quality of this data - as in - is the config properly tuned for this freq?
      imu_data = get_bno055_data();
      if (imu_data.status != 0) {
        printf("Imu Data Issue: %d\n", imu_data.status);
      }
      
      imu_msg.header.stamp = ros::Time::now();
      //NOTE: this is a different thing than other 'frames' used 
      imu_msg.header.frame_id = "pi_base_link";

      set_orientation(&orientation, imu_data);
      imu_msg.orientation = orientation;
      imu_msg.orientation_covariance = cov_matrix_zero;

      set_ang_vel(&angular_velocity, imu_data);
      imu_msg.angular_velocity = angular_velocity;
      imu_msg.angular_velocity_covariance = cov_matrix_zero;

      set_lin_accel(&linear_acceleration, imu_data);
      imu_msg.linear_acceleration = linear_acceleration;
      imu_msg.linear_acceleration_covariance = cov_matrix_zero;
      
      // build message event and write to bag
      boost::shared_ptr<const sensor_msgs::Imu> msg_ptr(new sensor_msgs::Imu(imu_msg));
      ros::MessageEvent<sensor_msgs::Imu> message(msg_ptr, ros::Time::now());
      bag.write("imu", message);
      //sleep(1);
  }

  func_val = close_bno055();
  printf("BNO055 close func returned %d\n", func_val);
  bag.close();
  return 0;
}
