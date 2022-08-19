//
// Created by vant3d on 18/08/2022.
//

#include "stereo_utility/camera_param.hpp"


M210_STEREO::CameraParam::CameraParam(uint8_t location) : location_(location) {
    if (!initIntrinsicParam()) {
        std::cerr << "Failed to init intrinsic parameter";
    }

    if (!initDistortionParam()) {
        std::cerr << "Failed to init distortion parameter";
    }
}

M210_STEREO::CameraParam::~CameraParam() {

}

M210_STEREO::CameraParam::Ptr M210_STEREO::CameraParam::createCameraParam(uint8_t position) {
    return std::make_shared<CameraParam>(position);
}

bool M210_STEREO::CameraParam::initDistortionParam() {
    if (location_ == FRONT_LEFT) {
        param_distortion_ = Config::get<cv::Mat>("leftDistCoeffs");
    } else if (location_ == FRONT_RIGHT) {
        param_distortion_ = Config::get<cv::Mat>("rightDistCoeffs");
    } else {
        std::cerr << "Please specify the location of the camera\n";
        return false;
    }
    return true;
}

bool M210_STEREO::CameraParam::initIntrinsicParam() {
    if (location_ == FRONT_LEFT) {
        param_intrinsic_ = Config::get<cv::Mat>("leftCameraIntrinsicMatrix");
    } else if (location_ == FRONT_RIGHT) {
        param_intrinsic_ = Config::get<cv::Mat>("rightCameraIntrinsicMatrix");
    } else {
        std::cerr << "Please specify the location of the camera\n";
        return false;
    }
    return true;
}
