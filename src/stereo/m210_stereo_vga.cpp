#include "m210_stereo_vga.h"

using namespace M210_STEREO;

// for visualization purpose
bool is_disp_filterd;
int count = 1;
dji_osdk_ros::StereoVGASubscription subscription;
ros::Publisher rect_img_left_publisher;
ros::Publisher rect_img_right_publisher;
ros::Publisher left_disparity_publisher;


int main(int argc, char **argv) {
    ros::init(argc, argv, "m210_stereo_perception");
    ros::NodeHandle nh;

    std::string yaml_file_path = "/home/vant3d/catkin_ws/src/stereo_image/config/tb_matlab_m210_stereo_calib.yaml";
    Config::setParamFile(yaml_file_path);

    //! Instantiate some relevant objects
    CameraParam::Ptr camera_left_ptr;
    CameraParam::Ptr camera_right_ptr;
    StereoFrame::Ptr stereo_frame_ptr;

    message_filters::Subscriber <sensor_msgs::Image> img_left_sub;
    message_filters::Subscriber <sensor_msgs::Image> img_right_sub;
    message_filters::TimeSynchronizer <sensor_msgs::Image, sensor_msgs::Image> *topic_synchronizer;


    //! Setup stereo frame
    camera_left_ptr = CameraParam::createCameraParam(CameraParam::FRONT_LEFT);
    camera_right_ptr = CameraParam::createCameraParam(CameraParam::FRONT_RIGHT);

    stereo_frame_ptr = StereoFrame::createStereoFrame(camera_left_ptr, camera_right_ptr);


    //! Setup ros related stuff
    rect_img_left_publisher =
            nh.advertise<sensor_msgs::Image>("/stereo_depth_perception/rectified_vga_front_left_image", 10);
    rect_img_right_publisher =
            nh.advertise<sensor_msgs::Image>("/stereo_depth_perception/rectified_vga_front_right_image", 10);
    left_disparity_publisher =
            nh.advertise<sensor_msgs::Image>("/stereo_depth_perception/disparity_front_left_image", 10);


    img_left_sub.subscribe(nh, "/dji_osdk_ros/stereo_vga_front_left_images", 1);
    img_right_sub.subscribe(nh, "/dji_osdk_ros/stereo_vga_front_right_images", 1);

    topic_synchronizer = new message_filters::TimeSynchronizer
            <sensor_msgs::Image, sensor_msgs::Image>(img_left_sub, img_right_sub, 10);


    //! For signal handling, e.g. if user terminate the program with Ctrl+C
    //! this program will unsubscribe the image stream if it's subscribed

    topic_synchronizer->registerCallback(boost::bind(&displayStereoFilteredDisparityCallback,
                                                     _1, _2, stereo_frame_ptr));


    ros::spin();
}


void displayStereoFilteredDisparityCallback(const sensor_msgs::ImageConstPtr &img_left,
                                            const sensor_msgs::ImageConstPtr &img_right,
                                            StereoFrame::Ptr stereo_frame_ptr) {
    //! Read raw images
    stereo_frame_ptr->readStereoImgs(img_left, img_right);

    //! Rectify images
    timer rectify_start = std::chrono::high_resolution_clock::now();
    stereo_frame_ptr->rectifyImgs();
    timer rectify_end = std::chrono::high_resolution_clock::now();

    //! Compute disparity
    timer disp_start = std::chrono::high_resolution_clock::now();
    stereo_frame_ptr->computeDisparityMap();
    timer disp_end = std::chrono::high_resolution_clock::now();

    //! Filter disparity map
    timer filter_start = std::chrono::high_resolution_clock::now();
    stereo_frame_ptr->filterDisparityMap();
    is_disp_filterd = true;
    timer filter_end = std::chrono::high_resolution_clock::now();

    visualizeRectImgHelper(stereo_frame_ptr);

    visualizeDisparityMapHelper(stereo_frame_ptr);

    sensor_msgs::Image rect_left_img = *img_left;
    sensor_msgs::Image rect_right_img = *img_right;
    sensor_msgs::Image disparity_map = *img_left;
    memcpy((char *) (&rect_left_img.data[0]),
           stereo_frame_ptr->getRectLeftImg().data,
           img_left->height * img_left->width);
    memcpy((char *) (&rect_right_img.data[0]),
           stereo_frame_ptr->getRectRightImg().data,
           img_right->height * img_right->width);
    memcpy((char *) (&disparity_map.data[0]),
           stereo_frame_ptr->getFilteredDispMap().data,
           img_left->height * img_left->width);

    rect_img_left_publisher.publish(rect_left_img);
    rect_img_right_publisher.publish(rect_right_img);
    left_disparity_publisher.publish(disparity_map);


    cv::waitKey(1);

    duration rectify_time_diff = rectify_end - rectify_start;
    duration disp_time_diff = disp_end - disp_start;
    duration filter_diff = filter_end - filter_start;
    ROS_INFO("This stereo frame takes %.2f ms to rectify, %.2f ms to compute disparity, "
             "%.2f ms to filter",
             rectify_time_diff.count() * 1000.0,
             disp_time_diff.count() * 1000.0,
             filter_diff.count() * 1000.0);


}


void
visualizeRectImgHelper(StereoFrame::Ptr stereo_frame_ptr) {
    cv::Mat img_to_show;

    cv::hconcat(stereo_frame_ptr->getRectLeftImg(),
                stereo_frame_ptr->getRectRightImg(),
                img_to_show);

    cv::resize(img_to_show, img_to_show,
               cv::Size(M210_STEREO::VGA_WIDTH * 2, M210_STEREO::VGA_HEIGHT),
               (0, 0), (0, 0), cv::INTER_LINEAR);

    // draw epipolar lines to visualize rectification
    for (int j = 0; j < img_to_show.rows; j += 24) {
        line(img_to_show, cv::Point(0, j),
             cv::Point(img_to_show.cols, j),
             cv::Scalar(255, 0, 0, 255), 1, 8);
    }
    cv::imshow("Rectified Stereo Imgs with epipolar lines", img_to_show);

    std::stringstream writeD;
    writeD <<  "Disparity" << count << ".png";
    cv::imwrite(writeD.str(), stereo_frame_ptr->getDisparityMap());
    count++;
}

cv::Mat average_disparity(cv::Mat &image, int x, int y, int h, int w) {
    std::vector<cv::Mat> channels;
    cv::Rect crop(x, y, h, w);
    cv::Mat img_crop = image(crop);
    cv::split(img_crop, channels);
    cv::Scalar m = mean(channels[0]);
    printf("%f\n", m[0]);
    cv::putText(image, "Mean Disparity: ", cvPoint(20, 20), CV_FONT_HERSHEY_SIMPLEX, 1, cvScalar(255, 0, 0), 2);
    return image;
}

void
visualizeDisparityMapHelper(StereoFrame::Ptr stereo_frame_ptr) {
    cv::Mat mean, stddev;
    cv::Mat raw_disp_map;

    if (is_disp_filterd) {
        raw_disp_map = stereo_frame_ptr->getFilteredDispMap().clone();
    } else {
        raw_disp_map = stereo_frame_ptr->getDisparityMap().clone();
    }

    double min_val, max_val;
    cv::minMaxLoc(raw_disp_map, &min_val, &max_val, NULL, NULL);

    cv::Mat scaled_disp_map;
    raw_disp_map.convertTo(scaled_disp_map, CV_8U, 255 / (max_val - min_val), -min_val / (max_val - min_val));
//    scaled_disp_map = (raw_disp_map / (float) 16.0 - (float) stereo_frame_ptr->getMinDisparity()) /
//                      ((float) stereo_frame_ptr->getNumDisparities());


    cv::imshow("Scaled disparity map", scaled_disp_map);
}




