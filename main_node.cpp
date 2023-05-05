#include <string>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <CwCapture.h>

class StereoDevice {
public:
  StereoDevice(int format_idx, 
               int64_t usec_sync_threshold=10000, 
               uint32_t resized_w=0U, 
               uint32_t resized_h=0U)
    : usec_sync_threshold_(usec_sync_threshold),
      w_(0U), h_(0U),
      resized_w_(resized_w),
      resized_h_(resized_h),
      sync_try_cnt_(0U)
  {
    for (unsigned i = 0; i < 2; i++)
    {
      cam_[i] = (CaptureDevice *) std::malloc(sizeof(CaptureDevice));
      
      if (cam_[i] == NULL) 
        exit(-1);

      if (initialize(cam_[i], i) < 0)
        exit(-1);

      if (set_option(cam_[i], format_idx) < 0)
        exit(-1);

      if (alloc_buffer(cam_[i]) < 0)
        exit(-1);
    }
    assert(cam_[0]->width == cam_[1]->width);
    assert(cam_[0]->height == cam_[1]->height);
    assert(cam_[0]->pixel_format == cam_[1]->pixel_format);
    assert(cam_[0]->f_idx == cam_[1]->f_idx);
    assert(cam_[0]->img_bytes == cam_[1]->img_bytes);
    w_ = cam_[0]->width;
    h_ = cam_[0]->height;
    if (resized_w_ == 0U || resized_h_ == 0U) {
      resized_w_ = w_;
      resized_h_ = h_;
    }
    img_[0] = (uint8_t *) std::malloc(sizeof(int8_t) * w_ * h_ * 3);
    img_[1] = (uint8_t *) std::malloc(sizeof(int8_t) * w_ * h_ * 3);
  }

  ~StereoDevice() 
  {
    for (unsigned i = 0; i < 2; i++) {
      free_buffer(cam_[i]);
      free(cam_[i]);
      free(img_[i]);
    }
  }

  void on() 
  {
    uint8_t flag = 0;
    for (unsigned i = 0; i < 2; i++)
    {
      if (stream_on(cam_[i]) < 0)
        break;
      else 
        flag |= (1 << i);
    }
    stream_state = (flag == 0b11) ? StreamState::START : StreamState::PUASE;
  }

  int sync_capture() 
  {

    capture(cam_[0]);
    usec_[0] = static_cast<int64_t>(cam_[0]->data.usec);
    capture(cam_[1]);
    usec_[1] = static_cast<int64_t>(cam_[1]->data.usec);

    int cnt = 0;
    int64_t diff = abs(usec_[0] - usec_[1]);
    while (diff >= usec_sync_threshold_
           && ++cnt <= 10)
    {
      if (cnt > 8) {
        ROS_INFO("Retry %d", cnt);
        ROS_INFO("dif: %ld", diff);
      }

      if (usec_[0] <= usec_[1])
      {
        capture(cam_[0]);
        usec_[0] = static_cast<int64_t>(cam_[0]->data.usec);
      }  
      else 
      {
        capture(cam_[1]);
        usec_[1] = static_cast<int64_t>(cam_[1]->data.usec);
      }

      diff = abs(usec_[0] - usec_[1]);
    }

    if (cnt > 10)
      return -1;
    else
      return 0;
  }

  cv::Mat get_img(int cam_index) {
    convert(cam_[cam_index], img_[cam_index]);
    cv::Mat img = cv::Mat(h_, w_, CV_8UC3, img_[cam_index]);
    // cv::rotate(img, img, cv::ROTATE_180);
    cv::flip(img, img, 0);
    return img;
  }

  void off() 
  {
    uint8_t flag = 0;
    for (unsigned i = 0; i < 2; i++)
    {
      if (stream_off(cam_[i]) < 0)
        break;
      else 
        flag |= (1 << i);
    }
    stream_state = (flag == 0b11) ? StreamState::STOP : StreamState::PUASE;
  }

  int64_t usec() {
    return (usec_[0] > usec_[1]) ? usec_[0] : usec_[1];
  }

private:
  CaptureDevice * cam_[2];
  uint8_t * img_[2];
  int64_t usec_[2];
  int64_t  usec_sync_threshold_;
  uint32_t  w_;
  uint32_t  h_;
  uint32_t  resized_w_;
  uint32_t  resized_h_;
  uint32_t  sync_try_cnt_;
  enum class StreamState{
    PUASE,
    START,
    STOP
  } stream_state;
};

  /* Set Global Base Time (cwlee) */
  // {
  //   auto now = std::chrono::high_resolution_clock::now();
  //   auto epoch = now.time_since_epoch();
  //   uint64_t value = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
  //   std::cout << "time: " << value << std::endl;
  // }

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "stereo_img_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  image_transport::Publisher pub0 = it.advertise("/cam0", 1);
  image_transport::Publisher pub1 = it.advertise("/cam1", 1);

  int format_option = 0;
  StereoDevice stereo_camera(format_option);

  stereo_camera.on();

  // using std::chrono::system_clock;
  // auto start = system_clock::now();
  // auto prev = start;
  // auto curr = start;
  ros::Rate rate(500);
  double fps = 10.0;
  double interval = 1.0 / fps;
  auto  u_interval =  static_cast<int64_t>(interval * 1000000);
  uint32_t count = 0;

  int64_t prev = 0;
  while (ros::ok())
  {
    if (stereo_camera.sync_capture() < 0) {
      ROS_INFO("Sync failed!");
      return -1;
    }
    else
    {
      int64_t curr = (stereo_camera.usec());
      if (prev == 0 || curr - prev > u_interval)
      {
        prev = curr;
        // ROS_INFO("Sync Success!");

        auto now = std::chrono::high_resolution_clock::now();
        auto epoch = now.time_since_epoch();
        uint64_t nano_now = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();

        std_msgs::Header header;
        header.stamp.fromNSec(nano_now);
        header.seq = count;
        sensor_msgs::ImagePtr msg0 = cv_bridge::CvImage(header, "bgr8", stereo_camera.get_img(1)).toImageMsg();
        sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(header, "bgr8", stereo_camera.get_img(0)).toImageMsg();
        pub0.publish(msg0);
        pub1.publish(msg1);
        
      }
    }
  }

  return 0;
}

/* Version 1

  while (ros::ok())
  {
    curr = system_clock::now();
    auto dur = std::chrono::duration<double>(curr - prev);
    if (dur.count() >= interval) {
      if (stereo_camera.sync_capture() < 0) {
        ROS_INFO("Sync failed!");
        return -1;
      }
      else 
      {
        auto dur2 = std::chrono::duration<double>(curr - start);
        ROS_INFO("%f: %f Sync Success!", dur2.count(), dur.count());
        count++;
        std_msgs::Header header;
        header.stamp.fromSec(dur2.count());
        header.seq = count;
        sensor_msgs::ImagePtr msg0 = cv_bridge::CvImage(header, "rgb8", stereo_camera.get_img(0)).toImageMsg();
        sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(header, "rgb8", stereo_camera.get_img(1)).toImageMsg();
        pub0.publish(msg0);
        pub1.publish(msg1);
        prev = curr;
      }
    }
    rate.sleep();
  }
  */

 /*
    if (abs(usec_[0] - usec_[1]) < usec_sync_threshold_)
    {
      ROS_INFO("sync success");
      sync_try_cnt_ = 0;
      return 0;
    }
    else 
    { // When sync is over threshold, try until 5 times.
      if (sync_try_cnt_++ < 5) 
      {
        ROS_INFO("sync retry");
        ros::Rate rate(500);
        rate.sleep();
        return sync_capture();
      }
      else 
        return -1;
    }
*/