#include <darknet_disparity.h>

DarknetDisparity::DarknetDisparity() {
    DarknetDisparity::subscribing(nh);
}

DarknetDisparity::~DarknetDisparity() {}

void DarknetDisparity::subscribing(ros::NodeHandle &nh) {
    std::string darknet_topic, image_topic, object_count_topic;

    nh.param("/darknet_distance/darknet_topic", darknet_topic, std::string("/darknet_ros/bounding_boxes"));
    nh.param("/darknet_distance/darknet_objet", object_count_topic, std::string("/darknet_ros/found_object"));
    nh.param("/darknet_distance/disparity_topic", image_topic, std::string("/stereo_depth_perception/disparity_front_left_image"));
    nh.param("/darknet_distance/object_track", object_to_track, std::string("simulacro"));


    darknet_bb_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>(darknet_topic, 1, &DarknetDisparity::darknet_cb,
                                                                   this);
    m210_disparity_sub = nh.subscribe<sensor_msgs::Image>(image_topic, 1, &DarknetDisparity::disparity_cb, this);

    darknet_obj_sub = nh.subscribe<darknet_ros_msgs::ObjectCount>(object_count_topic, 1, &DarknetDisparity::darknet_obj_count_cb, this);


}
void DarknetDisparity::darknet_obj_count_cb(const darknet_ros_msgs::ObjectCount::ConstPtr &obj_msg) {
    found_obj_darknet = obj_msg->count;
}
void DarknetDisparity::darknet_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bb_msg){
//    ROS_INFO("%s", bb_msg->bounding_boxes[n_detected_obj].Class.c_str());
    if(bb_msg->bounding_boxes[n_detected_obj].Class == object_to_track && cv_disp != NULL){
        top_left.x =  bb_msg->bounding_boxes[n_detected_obj].xmin;
        top_left.y = bb_msg->bounding_boxes[n_detected_obj].ymin;
        bot_right.x =  bb_msg->bounding_boxes[n_detected_obj].xmax;
        bot_right.y = bb_msg->bounding_boxes[n_detected_obj].ymax;
//        ROS_INFO("Got position of %s at (%d, %d) x (%d, %d)", bb_msg->bounding_boxes[n_detected_obj].Class.c_str() ,
//                                                            top_left.x, top_left.y, bot_right.x, bot_right.y);
        float distance  = calculate_distance();
        ROS_INFO("Aproximate distance: %f m", distance);

    }
    else{
        n_detected_obj++;
        if(n_detected_obj > found_obj_darknet){
            n_detected_obj = 0;
        }
    }
}

void DarknetDisparity::disparity_cb(const sensor_msgs::Image::ConstPtr &disp_msgs) {
    cv_disp = cv_bridge::toCvCopy(disp_msgs, sensor_msgs::image_encodings::MONO8);
//    float distance = 0;
    show_disp_image();
}

float DarknetDisparity::calculate_distance() {
    std::vector<cv::Mat> channels;
    cv::Rect crop(top_left,bot_right);
    cv::Mat img_crop = cv_disp->image(crop);
    cv::split(img_crop, channels);
    cv::Scalar m = mean(channels[0]);

    float baseline_x_fx_ = -45.3569;
    float principal_x_ = 450.6202;
    float principal_y_ = 231.8208;
    float fx_ = 444.3998;
    float fy_ = 444.3998;
    float u = (float) (crop.x + crop.width) / 2;
    float v = (float) (crop.y + crop.height) / 2;

    float disparity = (float) m[0];
    float dist_x, dist_y, dist_z;
    dist_z = baseline_x_fx_ / disparity;
    dist_x = (u - principal_x_) * (dist_z) / fx_;
    dist_y = (v - principal_y_) * (dist_z) / fy_;
    float distance = sqrt(dist_z * dist_z + dist_y * dist_y + dist_x * dist_x);
    return distance*10;
}

void DarknetDisparity::show_disp_image() {
    disp_img = cv_disp->image;
    cv::rectangle(disp_img, top_left, bot_right, cv::Scalar(0, 255, 0), 1, CV_AA);
    cv::imshow("Disparity", disp_img);
    cv::waitKey(1);
}
