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

// get images from video in batches, since we're limited to 1GB RAM
void extract_frames(const string& vid_file_path, vector<cv::Mat>& frames) {
  printf("Extracting image frames...\n");
  int c = 1;
  try {
    //open video file
    VideoCapture cap(vid_file_path);
    if (!cap.isOpened()) //check if success
      CV_Error(CV_StsError, "Can not open video file");
    
    //printf("Num of frames is %d\n", cap.get(CV_CAP_PROP_FRAME_COUNT));
    //printf("Frame rate: %f, current frame: %f, current time: %f\n", cap.get(CV_CAP_PROP_FPS), cap.get(CV_CAP_PROP_POS_FRAMES), cap.get(CV_CAP_PROP_POS_MSEC));
    // iterate over all frames, adding to frames vector
    for (;;) {
      cv::Mat frame;    
      cap.read(frame);
      printf("Frame %d Read. ", c);
      c++;
      if (frame.empty()) {
        break;
      }
      //printf("Frame sizeof(): %d, and frame channels: %d\n", sizeof(frame), frame.channels());
      //printf("Frame elemSize(): %d, and frame elemSize1(): %d, and depth(): %d\n", frame.elemSize(), frame.elemSize1(), frame.depth());
      //printf("Cols: %d, Rows: %d, Type: %d\n", frame.cols, frame.rows, frame.type());
      frames.push_back(frame);
    }
  } catch (cv::Exception& e) {
    cerr << e.msg << endl;
    exit(1);
  }
  printf("\n");
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
  sensor_msgs::Image image_msg;
  vector<cv::Mat> frames;
  vector<double> timestamps;
  vector<sensor_msgs::Imu::ConstPtr> imu_data;

  // populate frames and timestamps vectors
  extract_frames(vid_fpath, frames);
  extract_timestamps(ts_fpath, timestamps);
  if (frames.size() == timestamps.size()) {
    printf("Imagery data aligns with %d frames\n", frames.size());
  }

  // check and see if frames are different
  for(i=1; i < frames.size(); i++) {
    bool eq = cv::countNonZero(frames.at(i)!=frames.at(i-1)) == 0;
    if (eq) {
	printf("frames were equal\n");
    } else {
	printf("frames were not equal\n");
    }
  }

  //opening final data bag
  mono_inertial_bag.open(bag_fname, rosbag::bagmode::Write);

  // open motion bag and save motion data to the final bag
  motion_bag.open(motion_bag_fname, rosbag::bagmode::Read);
  vector<string> motion_topics;
  motion_topics.push_back(string("imu"));
  rosbag::View view(motion_bag, rosbag::TopicQuery(motion_topics));
  foreach(rosbag::MessageInstance const m, view) {
    ros::MessageEvent<sensor_msgs::Imu> message(m.instantiate<sensor_msgs::Imu>(), m.getTime());
    mono_inertial_bag.write("imu", message);
  }
  motion_bag.close();

  for (i=0; i < frames.size(); i++) {
    //image_msg.data = frames.at(i);
    //image_msg.height = 640;
    //image_msg.width = 640;
    //image_msg.encoding = "bgr8";
    //boost::shared_ptr<const sensor_msgs::Image> msg_ptr(new sensor_msgs::Image(image_msg));
    // seems to be the case that all frames objects are the same, since using 100 also looks like i
    const sensor_msgs::ImagePtr msg_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frames.at(i)).toImageMsg();
    ros::MessageEvent<sensor_msgs::Image> message(msg_ptr, ros::Time(timestamps.at(i)));
    mono_inertial_bag.write("camera/image_raw", message);
  }
  
  mono_inertial_bag.close();
  return 0;
}
