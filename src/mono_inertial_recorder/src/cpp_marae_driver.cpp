// ROS Imu Node 

#include <iostream>
#include <chrono>
#include <ctime>
//#include "bno055.h"
//#include "bno055.c"
#include "marae_bno055.c"
#include <rosbag/bag.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

void set_orientation(geometry_msgs::Quaternion* orientation, struct marae_data_t imu_data) {
    orientation->x = imu_data.quaternion.x;
    orientation->y = imu_data.quaternion.y;
    orientation->z = imu_data.quaternion.z;
    orientation->w = imu_data.quaternion.w;
    return;
}

void set_ang_vel(geometry_msgs::Vector3* ang_vel, struct marae_data_t imu_data) {
    ang_vel->x = imu_data.ang_vel.x;
    ang_vel->y = imu_data.ang_vel.y;
    ang_vel->z = imu_data.ang_vel.z;
    return;
}

void set_lin_accel(geometry_msgs::Vector3* lin_accel, struct marae_data_t imu_data) {
    lin_accel->x = imu_data.lin_accel.x;
    lin_accel->y = imu_data.lin_accel.y;
    lin_accel->z = imu_data.lin_accel.z;
    return;

}

int main(void)
{
  printf("Test...\n");
  int i, func_val;
  struct marae_data_t imu_data = {};
  auto start = std::chrono::system_clock::now();
  rosbag::Bag bag;
  sensor_msgs::Imu imu_msg;
  geometry_msgs::Quaternion orientation;
  geometry_msgs::Vector3 angular_velocity;
  geometry_msgs::Vector3 linear_acceleration;
  //TODO may want to fill in cov matrix in future for better accuracy
  boost::array<double, 9> cov_matrix_zero = {0,0,0,0,0,0,0,0,0};

  // TODO add in param to cpp to set bag name
  bag.open("marae_mono_inertial.bag", rosbag::bagmode::Write);
  
  func_val = init_and_calib_bno055();
  printf("Init and Calib returned %d\n", func_val);
  
  for (i=0; i< 2000; i++) {
      //TODO may want to verify at some point the quality of this data - as in - is the config properly tuned for this freq?
      imu_data = get_bno055_data();
      //auto end = std::chrono::system_clock::now();
      //std::chrono::duration<double> elapsed_time = end - start;
      //printf("%d %f\n", i, elapsed_time.count());
      imu_msg.header.stamp = ros::Time::now(); 
      //printf("%d %f\n", i, ros::Time::now());
      
      set_orientation(&orientation, imu_data);
      imu_msg.orientation = orientation;
      imu_msg.orientation_covariance = cov_matrix_zero;

      set_ang_vel(&angular_velocity, imu_data);
      imu_msg.angular_velocity = angular_velocity;
      imu_msg.angular_velocity_covariance = cov_matrix_zero;

      set_lin_accel(&linear_acceleration, imu_data);
      imu_msg.linear_acceleration = linear_acceleration;
      imu_msg.linear_acceleration_covariance = cov_matrix_zero;
      
      printf("Data %f %f %f\n", imu_msg.linear_acceleration.x, imu_msg.angular_velocity.y, imu_msg.orientation.w);
      // build message event and write to bag
      //boost::shared_ptr<const sensor_msgs::Imu> msg_ptr(new sensor_msgs::Imu(imu_msg));
      //ros::MessageEvent<sensor_msgs::Imu> message(msg_ptr, ros::Time::now());
      
      //bag.write("imu", message);
      //printf("BNO055 func call returned %d at %f\n", func_val, seconds);
  }

  func_val = close_bno055();
  printf("BNO055 close func returned %d\n", func_val);
  bag.close();
  return 0;
}
