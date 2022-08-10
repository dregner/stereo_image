#ifndef ONBOARDSDK_STEREO_FRAME_H
#define ONBOARDSDK_STEREO_FRAME_H

#include <memory>
#include <opencv2/opencv.hpp>
#include "frame.hpp"
#include "camera_param.hpp"
#include <opencv2/ximgproc/disparity_filter.hpp>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "visualization_msgs/MarkerArray.h"
#include "ros/ros.h"


namespace M210_STEREO
{

class StereoFrame
{
public:
  typedef std::shared_ptr<StereoFrame> Ptr;

  StereoFrame(CameraParam::Ptr left_cam, CameraParam::Ptr right_cam,
              int num_disp = 32, int block_size = 5);
  ~StereoFrame();

public:
  static StereoFrame::Ptr createStereoFrame(CameraParam::Ptr left_cam,
                                            CameraParam::Ptr right_cam);

  void readStereoImgs(const sensor_msgs::ImageConstPtr &img_left, const sensor_msgs::ImageConstPtr &img_right);

  void rectifyImgs();

  void computeDisparityMap();

  void filterDisparityMap();


  inline cv::Mat getRectLeftImg() { return this->rectified_img_left_; }

  inline cv::Mat getRectRightImg() { return this->rectified_img_right_; }

  inline cv::Mat getDisparityMap() { return this->disparity_map_8u_; }


#ifdef USE_OPEN_CV_CONTRIB
  inline cv::Mat getFilteredDispMap() { return this->filtered_disparity_map_8u_; }
#endif

protected:
  bool initStereoParam();

protected:
  //! Image frames
  Frame::Ptr frame_left_ptr_;
  Frame::Ptr frame_right_ptr_;

  //! Camera related
  CameraParam::Ptr camera_left_ptr_;
  CameraParam::Ptr camera_right_ptr_;

  //! Stereo camera parameters
  cv::Mat param_rect_left_;
  cv::Mat param_rect_right_;
  cv::Mat param_proj_left_;
  cv::Mat param_proj_right_;

  cv::Mat rectified_mapping_[2][2];

  //! Rectified images
  cv::Mat rectified_img_left_;
  cv::Mat rectified_img_right_;

  //! Block matching related
  int num_disp_;
  int block_size_;
  cv::Ptr<cv::StereoBM> block_matcher_;
  cv::Mat disparity_map_8u_;
  cv::Mat raw_disparity_map_;

  cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter_;
  cv::Ptr<cv::StereoMatcher> right_matcher_;

  cv::Mat raw_right_disparity_map_;
  cv::Mat filtered_disparity_map_;
  cv::Mat filtered_disparity_map_8u_;



};

} // namespace M210_STEREO

#endif //ONBOARDSDK_STEREO_FRAME_H
