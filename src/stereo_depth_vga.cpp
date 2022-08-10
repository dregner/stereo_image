#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/stereo.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>



/// Variavel para leitura GPS RTK


double RadToDeg = 180 / M_PI;

// initialize values for StereoSGBM parameters
int numDisparities = 1;
int blockSize = 5;
int preFilterType = 1;
int preFilterSize = 1;
int preFilterCap = 31;
int minDisparity = 0;
int textureThreshold = 10;
int uniquenessRatio = 15;
int speckleRange = 0;
int speckleWindowSize = 0;
int disp12MaxDiff = -1;
int dispType = CV_16S;


cv::Mat imgR_gray, imgL_gray;
cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();
cv::Mat disp, disparity;
// Defining callback functions for the trackbars to update parameter values

static void on_trackbar1( int, void* )
{
    stereo->setNumDisparities(numDisparities*16);
    numDisparities = numDisparities*16;
}

static void on_trackbar2( int, void* )
{
    stereo->setBlockSize(blockSize*2+5);
    blockSize = blockSize*2+5;
}

static void on_trackbar3( int, void* )
{
    stereo->setPreFilterType(preFilterType);
}

static void on_trackbar4( int, void* )
{
    stereo->setPreFilterSize(preFilterSize*2+5);
    preFilterSize = preFilterSize*2+5;
}

static void on_trackbar5( int, void* )
{
    stereo->setPreFilterCap(preFilterCap);
}

static void on_trackbar6( int, void* )
{
    stereo->setTextureThreshold(textureThreshold);
}

static void on_trackbar7( int, void* )
{
    stereo->setUniquenessRatio(uniquenessRatio);
}

static void on_trackbar8( int, void* )
{
    stereo->setSpeckleRange(speckleRange);
}

static void on_trackbar9( int, void* )
{
    stereo->setSpeckleWindowSize(speckleWindowSize*2);
    speckleWindowSize = speckleWindowSize*2;
}

static void on_trackbar10( int, void* )
{
    stereo->setDisp12MaxDiff(disp12MaxDiff);
}

static void on_trackbar11( int, void* )
{
    stereo->setMinDisparity(minDisparity);
}

void callback(const sensor_msgs::ImageConstPtr &image_R,
              const sensor_msgs::ImageConstPtr &image_L) {

    cv_bridge::CvImagePtr cv_ptr_R;
    cv_bridge::CvImagePtr cv_ptr_L;
    // Creating a named window to be linked to the trackbars
    cv::namedWindow("disparity",cv::WINDOW_NORMAL);
    cv::resizeWindow("disparity",600,600);

    // Creating trackbars to dynamically update the StereoBM parameters
    cv::createTrackbar("numDisparities", "disparity", &numDisparities, 18, on_trackbar1);
    cv::createTrackbar("blockSize", "disparity", &blockSize, 50, on_trackbar2);
    cv::createTrackbar("preFilterType", "disparity", &preFilterType, 1, on_trackbar3);
    cv::createTrackbar("preFilterSize", "disparity", &preFilterSize, 25, on_trackbar4);
    cv::createTrackbar("preFilterCap", "disparity", &preFilterCap, 62, on_trackbar5);
    cv::createTrackbar("textureThreshold", "disparity", &textureThreshold, 100, on_trackbar6);
    cv::createTrackbar("uniquenessRatio", "disparity", &uniquenessRatio, 100, on_trackbar7);
    cv::createTrackbar("speckleRange", "disparity", &speckleRange, 100, on_trackbar8);
    cv::createTrackbar("speckleWindowSize", "disparity", &speckleWindowSize, 25, on_trackbar9);
    cv::createTrackbar("disp12MaxDiff", "disparity", &disp12MaxDiff, 25, on_trackbar10);
    cv::createTrackbar("minDisparity", "disparity", &minDisparity, 25, on_trackbar11);

    try {
        cv_ptr_R = cv_bridge::toCvCopy(image_R, sensor_msgs::image_encodings::BGR8);
        cv_ptr_L = cv_bridge::toCvCopy(image_L, sensor_msgs::image_encodings::BGR8);
        cv::cvtColor(cv_ptr_L->image, imgL_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(cv_ptr_R->image, imgR_gray, cv::COLOR_BGR2GRAY);
        stereo->compute(imgL_gray, imgR_gray, disp);
        disp.convertTo(disparity, CV_32F, 1.0);
        disparity = (disparity/16.0f - (float) minDisparity / ((float) numDisparities));
    }


    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow("Result", disparity);
    cvWaitKey(1);

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "stereo_thread");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> image_sub_R(nh, "/stereo_depth_perception/rectified_vga_front_right_image", 1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub_L(nh, "/stereo_depth_perception/rectified_vga_front_left_image", 1);


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), image_sub_R, image_sub_L);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::spin();
    return 0;
}
