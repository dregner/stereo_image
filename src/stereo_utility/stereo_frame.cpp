#include <sensor_msgs/PointCloud2.h>
#include "stereo_utility/stereo_frame.hpp"

using namespace M210_STEREO;
using namespace cv;

StereoFrame::StereoFrame(CameraParam::Ptr left_cam,
                         CameraParam::Ptr right_cam,
                         int num_disp, int block_size)
  : camera_left_ptr_(left_cam)
  , camera_right_ptr_(right_cam)
  , num_disp_(num_disp)
  , block_size_(block_size)
  , raw_disparity_map_(Mat(VGA_HEIGHT, VGA_WIDTH, CV_16SC1))

{
  if(!this->initStereoParam())
  {
    ROS_ERROR("Failed to init stereo parameters\n");
  }

  Mat m210_vga_stereo_left  = Mat(VGA_HEIGHT,VGA_WIDTH, CV_8U);
  Mat m210_vga_stereo_right = Mat(VGA_HEIGHT,VGA_WIDTH, CV_8U);

  frame_left_ptr_   = Frame::createFrame(0, 0, m210_vga_stereo_left);
  frame_right_ptr_  = Frame::createFrame(0, 0, m210_vga_stereo_right);
}

StereoFrame::~StereoFrame()
{

}

bool
StereoFrame::initStereoParam()
{
  param_rect_left_ =  Config::get<Mat>("leftRectificationMatrix");
  param_rect_right_ = Config::get<Mat>("rightRectificationMatrix");
  param_proj_left_ =  Config::get<Mat>("leftProjectionMatrix");
  param_proj_right_ = Config::get<Mat>("rightProjectionMatrix");


  initUndistortRectifyMap(camera_left_ptr_->getIntrinsic(),
                          camera_left_ptr_->getDistortion(),
                          param_rect_left_,
                          param_proj_left_,
                          Size(VGA_WIDTH, VGA_HEIGHT), CV_32F,
                          rectified_mapping_[0][0], rectified_mapping_[0][1]);
  initUndistortRectifyMap(camera_right_ptr_->getIntrinsic(),
                          camera_right_ptr_->getDistortion(),
                          param_rect_right_,
                          param_proj_right_,
                          Size(VGA_WIDTH, VGA_HEIGHT), CV_32F,
                          rectified_mapping_[1][0], rectified_mapping_[1][1]);

  block_matcher_ = StereoBM::create(num_disp_, block_size_);


  wls_filter_ = ximgproc::createDisparityWLSFilter(block_matcher_); // left_matcher
  wls_filter_->setLambda(8000.0);
  wls_filter_->setSigmaColor(1.5);

  right_matcher_ = ximgproc::createRightMatcher(block_matcher_);


  return true;
}

StereoFrame::Ptr
StereoFrame::createStereoFrame(CameraParam::Ptr left_cam, CameraParam::Ptr right_cam)
{
  return std::make_shared<StereoFrame>(left_cam, right_cam);
}

void
StereoFrame::readStereoImgs(const sensor_msgs::ImageConstPtr &img_left, const sensor_msgs::ImageConstPtr &img_right)
{
  memcpy(frame_left_ptr_->raw_image.data,
         (char*)(&img_left->data[0]), sizeof(char)*VGA_HEIGHT*VGA_WIDTH);
  memcpy(frame_right_ptr_->raw_image.data,
         (char*)(&img_right->data[0]), sizeof(char)*VGA_HEIGHT*VGA_WIDTH);

  frame_left_ptr_ ->id  = img_left->header.seq;
  frame_right_ptr_->id  = img_right->header.seq;
  frame_left_ptr_-> time_stamp  = img_left->header.stamp.nsec;
  frame_right_ptr_->time_stamp  = img_right->header.stamp.nsec;
}

void
StereoFrame::rectifyImgs()
{

  remap(frame_left_ptr_->getImg(), rectified_img_left_,
            rectified_mapping_[0][0], rectified_mapping_[0][1], INTER_LINEAR);
  remap(frame_right_ptr_->getImg(), rectified_img_right_,
            rectified_mapping_[1][0], rectified_mapping_[1][1], INTER_LINEAR);
}

void
StereoFrame::computeDisparityMap()
{


  // CPU implementation of stereoBM outputs short int, i.e. CV_16S
  block_matcher_->compute(rectified_img_left_, rectified_img_right_, raw_disparity_map_);

  raw_disparity_map_.convertTo(disparity_map_8u_, CV_8UC1, 0.0625);

}

void
StereoFrame::filterDisparityMap()
{
  right_matcher_->compute(rectified_img_right_, rectified_img_left_, raw_right_disparity_map_);

  // Only takes CV_16S type cv::Mat
  wls_filter_->filter(raw_disparity_map_,
                      rectified_img_left_,
                      filtered_disparity_map_,
                      raw_right_disparity_map_);

  filtered_disparity_map_.convertTo(filtered_disparity_map_8u_, CV_8UC1, 0.0625);

}