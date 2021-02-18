// ROS Stereo Imagery Node 

#include <iostream>
#include <fstream>
#include <rosbag/bag.h>
#include <ros/time.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cstring>
#include <string>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace std;
using namespace cv;

// get batch of 50 cam0 and 50 cam1 frames from stereo video file
// returns whether or not there are more frames left in the video
bool extract_frames(VideoCapture cap, vector<cv::Mat>& frames_raw_0, vector<cv::Mat>& frames_raw_1, cv::Mat mask0, cv::Mat mask1) {
  try {
    printf("Extracting image frames...\n");
    //printf("Num of frames is %d\n", cap.get(CV_CAP_PROP_FRAME_COUNT));
    //printf("Frame rate: %f, current frame: %f, current time: %f\n", cap.get(CV_CAP_PROP_FPS), cap.get(CV_CAP_PROP_POS_FRAMES), cap.get(CV_CAP_PROP_POS_MSEC));
    // iterate over batch of frames, adding to frames vector
    for (int c = 0; c < 50; c++) {
      cv::Mat stereo_frame = cv::Mat::zeros(cv::Size(1408, 700), CV_8UC3);
      cv::Mat frame_0 = cv::Mat::zeros(cv::Size(700, 700), CV_8UC3);
      cv::Mat frame_1 = cv::Mat::zeros(cv::Size(700, 700), CV_8UC3);
      cap.read(stereo_frame);
      printf("Frame %d Read. ", c);
      if (stereo_frame.empty()) {
          return false;
      }
      printf("stereo_frame.size: (%d, %d) and channels: %d \n", stereo_frame.size().width, stereo_frame.size().height, stereo_frame.channels());
      //now, split the stereo frame
      printf("frame_0.size: (%d, %d), mask0.size: (%d, %d)\n", frame_0.size().width, frame_0.size().height, mask0.size().width, mask0.size().height);
      stereo_frame.copyTo(frame_0, mask0);
      stereo_frame.copyTo(frame_1, mask1);

      printf("frame_0.size: (%d, %d)\n", frame_0.size().width, frame_0.size().height);
      //printf("Frame sizeof(): %d, and frame channels: %d\n", sizeof(frame), frame.channels());
      //printf("Frame elemSize(): %d, and frame elemSize1(): %d, and depth(): %d\n", frame.elemSize(), frame.elemSize1(), frame.depth());
      //printf("Cols: %d, Rows: %d, Type: %d\n", frame.cols, frame.rows, frame.type());
      
      frames_raw_0.push_back(frame_0);
      frames_raw_1.push_back(frame_1);
      // also write both to their respective folders as images to use in calibration
      cv::imwrite(format("./left/img%d.jpg", c), frame_0);
      cv::imwrite(format("./right/img%d.jpg", c), frame_1);
      cv::imwrite("./left/stereo.jpg", stereo_frame);
    }
    cv::imwrite("mask0.jpg", mask0);
    cv::imwrite("mask1.jpg", mask1);
  } catch (cv::Exception& e) {
    cerr << e.msg << endl;
    exit(1);
  }
  
  return true;
}

// get timestamps from input file
// input file format will be: 
// <start_utc_time_ms>
// <some meaningless text from raspivid>
// <ts1>
// <ts2> ...
void extract_timestamps(const string& ts_file_path, vector<double>& timestamps) {
  printf("Extracting timestamps...\n");
  double utc_start_ts;
  double ts;
  string line;
  ifstream ts_file;
  
  ts_file.open(ts_file_path);
  if(ts_file.is_open()) {
    getline(ts_file, line);
    utc_start_ts = std::stod(line);
    printf("Start time of video is: %f\n", utc_start_ts);
    //now skip over the comment line
    getline(ts_file, line);
    while (getline(ts_file, line)) {
      //read image offset times (which are in ms)
      ts = std::stod(line) / 1000;
      timestamps.push_back(utc_start_ts + ts);
    }
  }
  ts_file.close();
}


int main(int argc, char *argv[])
{
  printf("Starting Stereo Image Synthesizer....\n");
  string bag_suffix = argv[1];
  string vid_fpath = argv[2];
  string ts_fpath = argv[3];
  string motion_bag_fname = "marae_mono_inertial_motion.bag";
  string bag_fname = "marae_stereo_inertial_" + bag_suffix + ".bag";
  int i, func_val;
  rosbag::Bag motion_bag, stereo_inertial_bag;
  vector<double> timestamps;
  vector<cv::Mat> frames_raw_0, frames_raw_1;
  vector<sensor_msgs::Imu::ConstPtr> imu_data;

  //opening final stereo-inertial bag
  stereo_inertial_bag.open(bag_fname, rosbag::bagmode::Write);

  // open motion bag and save motion data to the final stereo_inertial bag
  /*
  motion_bag.open(motion_bag_fname, rosbag::bagmode::Read);
  vector<string> motion_topics;
  motion_topics.push_back(string("imu"));
  rosbag::View view(motion_bag, rosbag::TopicQuery(motion_topics));
  foreach(rosbag::MessageInstance const m, view) {
    ros::MessageEvent<sensor_msgs::Imu> message(m.instantiate<sensor_msgs::Imu>(), m.getTime());
    mono_inertial_bag.write("imu", message);
  }
  motion_bag.close();
  */

  // Now incorporate imagery data
  // populate timestamp vector of imagery
  extract_timestamps(ts_fpath, timestamps);
  // generate masks for stereo imagery processing
  // mask0 will be used to get frames from cam0 (left)
  // mask1 will be used to get frames from cam1 (right)
  cv::Mat mask0 = cv::Mat::zeros(cv::Size(1408, 700), CV_8UC3);//cv::Mat(700, 1400, CV_8UC1, 0);
  cv::Mat mask1 = cv::Mat::zeros(cv::Size(1408, 700), CV_8UC3);//cv::Mat(mask0, true); 
  cv::Vec3b filled_pixel(1, 1, 1);
  // mask0 will be first 700 columns
  // then will have a one column gap and mask1 will be the next 700 columns
  for(int col=0; col<1402; col++) {
    for(int row=0; row<700; row++) {
        if (col < 700) {
	    mask0.at<cv::Vec3b>(row, col) = filled_pixel;
	} else if (col > 700 && col < 1401) {
	    mask1.at<cv::Vec3b>(row, col) = filled_pixel;
        }
    }
  } 
  printf("Stereo Masks Generated.\n");

  //open video file
  try {
    VideoCapture cap(vid_fpath);
    if (!cap.isOpened()) //check if success
      CV_Error(CV_StsError, "Can not open video file");
  
    // populate frames in batches and synthesize with timestamps
    bool hasMoreFrames = true;
    i = 0;
    while (hasMoreFrames) {
      hasMoreFrames = extract_frames(cap, frames_raw_0, frames_raw_1, mask0, mask1);
      //iterate over batch of raw frames (from cam 0 and cam 1) 
      for (int j = 0; j < frames_raw_0.size(); j++) {
        const sensor_msgs::ImagePtr msg_ptr_0 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frames_raw_0.at(j)).toImageMsg();
	const sensor_msgs::ImagePtr msg_ptr_1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frames_raw_1.at(j)).toImageMsg();
        ros::MessageEvent<sensor_msgs::Image> message_0(msg_ptr_0, ros::Time(timestamps.at(i)));
        ros::MessageEvent<sensor_msgs::Image> message_1(msg_ptr_1, ros::Time(timestamps.at(i)));
        stereo_inertial_bag.write("camera/left/image_raw", message_0);
	stereo_inertial_bag.write("camera/right/image_raw", message_1);
        i++;
      }
      frames_raw_0.clear();
      frames_raw_1.clear();
    }
  } catch (cv::Exception& e) {
    cerr << e.msg << endl;
    exit(1);
  }

  stereo_inertial_bag.close();
  return 0;
}
