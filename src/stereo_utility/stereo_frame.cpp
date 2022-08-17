#include <sensor_msgs/PointCloud2.h>
#include "stereo_utility/stereo_frame.hpp"

//using namespace M210_STEREO;

M210_STEREO::StereoFrame::StereoFrame(CameraParam::Ptr left_cam,
                                      CameraParam::Ptr right_cam,
                                      int num_disp, int block_size)
        : camera_left_ptr_(left_cam), camera_right_ptr_(right_cam), num_disp_(num_disp), block_size_(block_size),
          raw_disparity_map_(cv::Mat(VGA_HEIGHT, VGA_WIDTH, CV_16SC1)) {
    if (!this->initStereoParam()) {
        ROS_ERROR("Failed to init stereo parameters\n");
    }

    cv::Mat m210_vga_stereo_left = cv::Mat(VGA_HEIGHT, VGA_WIDTH, CV_8U);
    cv::Mat m210_vga_stereo_right = cv::Mat(VGA_HEIGHT, VGA_WIDTH, CV_8U);

    frame_left_ptr_ = Frame::createFrame(0, 0, m210_vga_stereo_left);
    frame_right_ptr_ = Frame::createFrame(0, 0, m210_vga_stereo_right);
}

M210_STEREO::StereoFrame::~StereoFrame() {

}

bool
M210_STEREO::StereoFrame::initStereoParam() {
    param_rect_left_ = Config::get<cv::Mat>("leftRectificationMatrix");
    param_rect_right_ = Config::get<cv::Mat>("rightRectificationMatrix");
    param_proj_left_ = Config::get<cv::Mat>("leftProjectionMatrix");
    param_proj_right_ = Config::get<cv::Mat>("rightProjectionMatrix");


    initUndistortRectifyMap(camera_left_ptr_->getIntrinsic(),
                            camera_left_ptr_->getDistortion(),
                            param_rect_left_,
                            param_proj_left_,
                            cv::Size(VGA_WIDTH, VGA_HEIGHT), CV_32F,
                            rectified_mapping_[0][0], rectified_mapping_[0][1]);
    initUndistortRectifyMap(camera_right_ptr_->getIntrinsic(),
                            camera_right_ptr_->getDistortion(),
                            param_rect_right_,
                            param_proj_right_,
                            cv::Size(VGA_WIDTH, VGA_HEIGHT), CV_32F,
                            rectified_mapping_[1][0], rectified_mapping_[1][1]);

    block_matcher_ = cv::StereoBM::create(num_disp_, block_size_);
//    block_matcher_->setPreFilterCap(1);
//    block_matcher_->setUniquenessRatio(100);
//    block_matcher_->setSpeckleRange(0);


    wls_filter_ = cv::ximgproc::createDisparityWLSFilter(block_matcher_); // left_matcher
    wls_filter_->setLambda(8000.0);
    wls_filter_->setSigmaColor(1.5);

    right_matcher_ = cv::ximgproc::createRightMatcher(block_matcher_);


    return true;
}

M210_STEREO::StereoFrame::Ptr
M210_STEREO::StereoFrame::createStereoFrame(CameraParam::Ptr left_cam, CameraParam::Ptr right_cam) {
    return std::make_shared<StereoFrame>(left_cam, right_cam);
}

void M210_STEREO::StereoFrame::readStereoImgs(const sensor_msgs::ImageConstPtr &img_left,
                                              const sensor_msgs::ImageConstPtr &img_right) {
    memcpy(frame_left_ptr_->raw_image.data,
           (char *) (&img_left->data[0]), sizeof(char) * VGA_HEIGHT * VGA_WIDTH);
    memcpy(frame_right_ptr_->raw_image.data,
           (char *) (&img_right->data[0]), sizeof(char) * VGA_HEIGHT * VGA_WIDTH);

    frame_left_ptr_->id = img_left->header.seq;
    frame_right_ptr_->id = img_right->header.seq;
    frame_left_ptr_->time_stamp = img_left->header.stamp.nsec;
    frame_right_ptr_->time_stamp = img_right->header.stamp.nsec;
}

void
M210_STEREO::StereoFrame::rectifyImgs() {

    cv::remap(frame_left_ptr_->getImg(), rectified_img_left_,
              rectified_mapping_[0][0], rectified_mapping_[0][1], cv::INTER_LINEAR);
    cv::remap(frame_right_ptr_->getImg(), rectified_img_right_,
              rectified_mapping_[1][0], rectified_mapping_[1][1], cv::INTER_LINEAR);
}

void M210_STEREO::StereoFrame::computeDisparityMap() {

    //! CPU implementation of stereoBM outputs short int, i.e. CV_16S
    block_matcher_->compute(rectified_img_left_, rectified_img_right_, raw_disparity_map_);

    raw_disparity_map_.convertTo(disparity_map_8u_, CV_8UC1, 0.06); //! 0.0625

    // convert it from CV_8U to CV_16U for unified
    // calculation in filterDisparityMap() & unprojectPtCloud()
//    raw_disparity_map_.convertTo(raw_disparity_map_, CV_16S, 8);
}

void M210_STEREO::StereoFrame::filterDisparityMap() {
    right_matcher_->compute(rectified_img_right_, rectified_img_left_, raw_right_disparity_map_);

    // Only takes CV_16S type cv::Mat
    wls_filter_->filter(raw_disparity_map_,
                        rectified_img_left_,
                        filtered_disparity_map_,
                        raw_right_disparity_map_);

    filtered_disparity_map_.convertTo(filtered_disparity_map_8u_, CV_8UC1, 0.06);

}
