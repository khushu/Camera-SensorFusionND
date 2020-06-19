#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void harisCornner()
{
    // TODO: Based on the image gradients in both x and y, compute an image 
    // which contains the gradient magnitude according to the equation at the 
    // beginning of this section for every pixel position. Also, apply different 
    // levels of Gaussian blurring before applying the Sobel operator and compare the results.

    //

    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1gray.png");

    // convert image to grayscale
    cv::Mat imgGray;
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // Detector parameters
    int blockSize = 3; // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 5; // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.05; // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(imgGray.size(), CV_32FC1 );
    cv::cornerHarris( imgGray, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT ); 
    cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
    cv::convertScaleAbs( dst_norm, dst_norm_scaled );

    // visualize results
    string windowName = "Harris Corner Detector Response Matrix";
    cv::namedWindow( windowName, 4 );
    cv::imshow( windowName, dst_norm_scaled );
    cv::waitKey(0);
    string windowName1 = "Input";
    cv::namedWindow( windowName1, 4 );
    cv::imshow( windowName1, img );
    cv::waitKey(0);
}

int main()
{
    harisCornner();
}