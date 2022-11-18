#include <opencv2/opencv.hpp>
#include <glob.h>
#include <stereo_utility/config.hpp>


cv::Mat image, imageClone;
//to store left_t_corner and point on the right_b_corner of the circle
cv::Point left_t_corner, right_b_corner;

float distance_disp(cv::Rect rect, double &mean_disp) {


//    double principal_x_ = param_proj_left_.at<double>(0, 2);
//    double principal_y_ = param_proj_left_.at<double>(1, 2);
//    double fx_ = param_proj_left_.at<double>(0, 0);
//    double fy_ = param_proj_left_.at<double>(1, 1);
//    double baseline_x_fx_ = -param_proj_right_.at<double>(0, 3);
    //! Values from matlab
//    float  baseline_x_fx_ = -4.53568575e1;
//    float principal_x_ = 4.5062e2;
//    float principal_y_ = 2.3182e2;
//    float  fx_ = 4.44399799e2;
//    float fy_ = 4.44399799e2;
    //! Values form opencv
    float  baseline_x_fx_ = -4.53568575e1;
    float principal_x_ = 4.50620243 e2;
    float principal_y_ = 2.3182076e2;
    float  fx_ = 4.44399799e2;
    float fy_ = 4.44399799e2;
    float u = (float) (rect.x + rect.width) / 2;
    float v = (float) (rect.y + rect.height) / 2;

    auto disparity = mean_disp;
    float dist_x, dist_y, dist_z;
    dist_z = baseline_x_fx_ / disparity;
    dist_x = (u - principal_x_) * (dist_z) / fx_;
    dist_y = (v - principal_y_) * (dist_z) / fy_;
    float distance = sqrt(dist_z * dist_z + dist_y * dist_y + dist_x * dist_x);
    return distance;
}

double average_disparity(cv::Mat &image, cv::Point pt1, cv::Point pt2) {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::Mat> channels;
    cv::Rect crop(pt1, pt2);
    cv::Mat img_crop = gray(crop);
    cv::split(img_crop, channels);
    cv::Scalar m = mean(channels[0]);
    return m[0];
}


//Callback function which will be called for mouse events
void drawCircle(int action, int x, int y, int flags, void *userdata) {
    //action taken when left mouse button pressed
    if (action == cv::EVENT_LBUTTONDOWN) {
        left_t_corner = cv::Point(x, y);
        //Draw the left_t_corner
        circle(imageClone, left_t_corner, 1, cv::Scalar(0, 255, 0), 1, CV_AA);
    }
        //acion taken when left mouse button is released
    else if (action == cv::EVENT_LBUTTONUP) {
        right_b_corner = cv::Point(x, y);
        //Draw circle
        cv::rectangle(imageClone, left_t_corner, right_b_corner, cv::Scalar(0, 255, 0), 1, CV_AA);
        double mean = average_disparity(imageClone, left_t_corner, right_b_corner);
        float dist = distance_disp(cv::Rect(left_t_corner, right_b_corner), mean);
        std::string label = cv::format("Mean value : %.2f px", mean);
        std::cout << "Pixel mean: " << mean << " px \t Distance: " << dist * 10 << " m" << std::endl;
        putText(imageClone, label, cv::Point(100, 100),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 5);
        imshow("image", imageClone);
    }


}

int main() {
    //Read input image
    int i = 0;

    cv::String folder("/home/vant3d/Pictures/*.jpg");
    std::vector<cv::String> filenames;
    cv::glob(folder, filenames, false);


    //Create a clone of input image to work on
    namedWindow("image", cv::WINDOW_NORMAL);
    cv::setMouseCallback("image", drawCircle);

    //set up callback for mouse events

    for (int i = 0; i < filenames.size(); i++) {
        image = cv::imread(filenames[i], cv::IMREAD_COLOR);
        imageClone = image.clone();

        cv::imshow("image", imageClone);


        cv::waitKey(0);

    }


    //close all the opended windows
    cv::destroyAllWindows();

    return 0;
}