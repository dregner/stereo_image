//
// Created by regner on 14/10/2021.
//

//Uncomment the following line if you are compiling this code in Visual Studio
//#include "stdafx.h"

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    // Read the image file
    Mat imageR = imread("/home/regner/Documents/stereo_VGA_calib/1_5m_filtered/left/image_L10.png");
    Mat imageL = imread("/home/regner/Documents/stereo_VGA_calib/1_5m_filtered/right/image_R10.png");

    // Check for failure
    if (imageR.empty() || imageL.empty())
    {
        cout << "Could not open or find the image" << endl;
        cin.get(); //wait for any key press
        return -1;
    }

    String windowNameL = "left"; //Name of the window
    String windowNameR = "right"; //Name of the window


    imshow(windowNameL, imageL); // Show our image inside the created window.
    imshow(windowNameR, imageR); // Show our image inside the created window.

    waitKey(0); // Wait for any keystroke in the window

    destroyWindow(windowNameL); //destroy the created window
    destroyWindow(windowNameR); //destroy the created window

    return 0;
}