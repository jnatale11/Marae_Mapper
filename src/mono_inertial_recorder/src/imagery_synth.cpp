// ROS Imagery Node 

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
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace std;
using namespace cv;

// get batch of 50 frames from video file
// returns whether or not there are more frames left in the video
bool extract_frames(VideoCapture cap, vector<cv::Mat>& frames) {
  try {
    //open video file
    //VideoCapture cap(vid_file_path);
    //if (!cap.isOpened()) //check if success
    //  CV_Error(CV_StsError, "Can not open video file");
    
    //printf("Num of frames is %d\n", cap.get(CV_CAP_PROP_FRAME_COUNT));
    //printf("Frame rate: %f, current frame: %f, current time: %f\n", cap.get(CV_CAP_PROP_FPS), cap.get(CV_CAP_PROP_POS_FRAMES), cap.get(CV_CAP_PROP_POS_MSEC));
    // iterate over batch of frames, adding to frames vector
    for (int c = 0; c < 50; c++) {
      cv::Mat frame;    
      cap.read(frame);
      printf("Frame %d Read. ", c);
      if (frame.empty()) {
          return false;
      }
      //printf("Frame sizeof(): %d, and frame channels: %d\n", sizeof(frame), frame.channels());
      //printf("Frame elemSize(): %d, and frame elemSize1(): %d, and depth(): %d\n", frame.elemSize(), frame.elemSize1(), frame.depth());
      //printf("Cols: %d, Rows: %d, Type: %d\n", frame.cols, frame.rows, frame.type());
      frames.push_back(frame);
      c++;
    }
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
  printf("Starting ros image population...\n");
  string bag_suffix = argv[1];
  string vid_fpath = argv[2];
  string ts_fpath = argv[3];
  string motion_bag_fname = "marae_mono_inertial_motion.bag";
  string bag_fname = "marae_mono_inertial_" + bag_suffix + ".bag";
  int i, func_val;
  rosbag::Bag motion_bag, mono_inertial_bag;
  vector<double> timestamps;
  vector<cv::Mat> frames_raw;
  vector<sensor_msgs::Imu::ConstPtr> imu_data;

  //opening final data bag
  mono_inertial_bag.open(bag_fname, rosbag::bagmode::Write);

  // open motion bag and save motion data to the final mono_inertial bag
  motion_bag.open(motion_bag_fname, rosbag::bagmode::Read);
  vector<string> motion_topics;
  motion_topics.push_back(string("imu"));
  rosbag::View view(motion_bag, rosbag::TopicQuery(motion_topics));
  foreach(rosbag::MessageInstance const m, view) {
    ros::MessageEvent<sensor_msgs::Imu> message(m.instantiate<sensor_msgs::Imu>(), m.getTime());
    mono_inertial_bag.write("imu", message);
  }
  motion_bag.close();

  // incorporate imagery data
  // populate timestamp vector of imagery
  extract_timestamps(ts_fpath, timestamps);
  //open video file
  try {
    VideoCapture cap(vid_fpath);
    if (!cap.isOpened()) //check if success
      CV_Error(CV_StsError, "Can not open video file");
  
    // populate frames in batches and synthesize with timestamps
    bool hasMoreFrames = true;
    i = 0;
    while (hasMoreFrames) {
      hasMoreFrames = extract_frames(cap, frames_raw);
      //iterate over frames_raw
      for (int j = 0; j < frames_raw.size(); j++) {
        const sensor_msgs::ImagePtr msg_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frames_raw.at(j)).toImageMsg();
        ros::MessageEvent<sensor_msgs::Image> message(msg_ptr, ros::Time(timestamps.at(i)));
        mono_inertial_bag.write("camera/image_raw", message);
        i++;
      }
      frames_raw.clear();
    }
  } catch (cv::Exception& e) {
    cerr << e.msg << endl;
    exit(1);
  }

  mono_inertial_bag.close();
  return 0;
}
