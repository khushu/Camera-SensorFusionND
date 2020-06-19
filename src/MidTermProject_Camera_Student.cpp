/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include "matching2D.hpp"
#include "dataStructures.h"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    //Filepath for the results directory
    string resutBasePath = dataPath + "results/";
    
    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0;          // first file index to load (assumes Lidar and camera names 
                                    // have identical naming convention)
    int imgEndIndex = 9;            // last file index to load
    int imgFillWidth = 4;           // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;         // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer ;  // list of data frames which are held in memory at the same time

    bool bVis = false;              // visualize results
    bool bVehicleBBox = true;       // visualize only vehicle keypoints
    bool bConsoleLogging = true;    // Console minimum logging


    //TODO: 
    //Put counters and timers for the table and data collection
    //For each pair of detector / descriptor log for a table: Form a CSV File: (for logging the top 3-5 in each category)
    //One file each for 1 & 2 
    //1. Method pair, Number of keypoints image, and Distribution of neighbourhood size, feature/keypoint size(), variance, mean, distribution, time taken for tetector and discriptor 
    //avarage time taken per keypoint over the entire method pair (Accumulate)
    //2. Number of match keypoints 1-2, 2-3, 3-4.. matched, time taken per keypoint

    //TODO::Replace with a logging library (spdlog, as external)


    //7.Count the number of keypoints on the preceding vehicle for all 10 images and take 
    //note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.     


    //8. Count the number of matched keypoints for all 10 images using all possible combinations 
    //of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.time taken per keypoint
    
    //9. Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and 
    //based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.

    //Will not concentrate on accuracies and Reciever opperating charecteristics 
    //Estimating homographies on planner surfaces ( to compute the ground truth extraction)


    //What is the neighborhood size? In the case of Shi-Tomasi and Harris detectors, I believe it refers to parameter blockSize; 
    //but how is it defined/calculated for the other detectors?

    //distribution of their neighborhood size
    //Indeed in the case of Shi-Tomasi and Harris detectors it refers to the blockSize parameter. 
    //In this example of SIFT you can check that the neighborhood is represented by the meaningful keypoint size and represented 
    //by the circle radius around the detected keypoint. Therefore, you can access this feature using the size() 
    //parameter of the keypoints. In order to log it using the following code snippet, to be added once cleaned the unnecessary keypoints. 
    //keypoint.size();

    // double mean = std::accumulate(keypoints.size.begin(), keypoints.size.end(), 0.0)/keypoints.size();
    // auto add_square = [mean](double sum, int i)
    // {
    //     auto d = i - mean;
    //     return sum + d*d;
    // };
    // double total = std::accumulate(keypoints.size.begin(), keypoints.size.end(), 0.0, add_square);
    // double variance = total / keypoints.size();
    // Also, for each of the combination play with parameters to improve the results


    for (int iDetectorIndex = detector_SHITOMASI; iDetectorIndex <= detector_SIFT; ++iDetectorIndex)
    {
        for (int iDescriptorIndex = descriptor_BRISK; iDescriptorIndex <= descriptor_SIFT; ++iDescriptorIndex)
        {

            if ((iDescriptorIndex == descriptor_AKAZE && iDetectorIndex != detector_AKAZE) ||
                (iDescriptorIndex == descriptor_ORB   && iDetectorIndex == detector_SIFT))
            {
                // AKAZE descriptor extractor works only with key-points detected with KAZE/AKAZE detectors
                // see https://docs.opencv.org/3.0-beta/modules/features2d/doc/feature_detection_and_description.html#akaze
                
                // ORB descriptor extractor does not work with the SIFT detetor
                // see https://answers.opencv.org/question/5542/sift-feature-descriptor-doesnt-work-with-orb-keypoinys/
                continue;
            }

            //String for steam of data for the experiment
            ostringstream oss;
            //Make name of the file
            string resultFileName(resutBasePath);
            // oss << ToString(e_detector)     << '_'
            //   << ToString(e_descriptor) << '_'
            //   << ToString(CompatibleDescriptorTypes(e_descriptor)[0]) << '_'
            //   << ToString(e_matcher) << '_'
            //   << ToString(e_selector);

            //resutBasePath

            ofstream resultFile(resultFileName);

            //Log names 


            /* MAIN LOOP OVER ALL IMAGES */

            for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
            {
                if( bConsoleLogging ){
                    cout <<"        1        : LOAD IMAGE INTO BUFFER" << endl;
                    cout<< "Image Loop Index : "<< imgIndex << endl;
                }
                /* LOAD IMAGE INTO BUFFER */

                // assemble filenames for current index
                ostringstream imgNumber;
                imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
                string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

                // load image from file and convert to grayscale
                cv::Mat img, imgGray;
                img = cv::imread(imgFullFilename);
                cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

                //// STUDENT ASSIGNMENT
                //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize
                DataFrame frame;

                frame.cameraImg = imgGray;        
                //push image into data frame buffer
                dataBuffer.push_back(frame);

                //Ring buffer implementation on vector, erase the first element in the vector, which is the oldest 
                if (dataBuffer.size() > dataBufferSize)
                {
                    dataBuffer.erase(dataBuffer.begin());
                }        

                if( bConsoleLogging ){
                    cout<< "Databuffer Size  : "<< dataBuffer.size() << endl;          

                //// EOF STUDENT ASSIGNMENT
                    cout << "--------1--------: done" << endl;
                /* DETECT IMAGE KEYPOINTS */
                    cout << "        2        : DETECT IMAGE KEYPOINTS" << endl;
                }

                // extract 2D keypoints from current image
                vector<cv::KeyPoint> keypoints; // create empty feature list for current image

                string detectorType = "";
                //string detectorType = "SHITOMASI";
                //string detectorType = "HARRIS";
                //string detectorType = "FAST";
                //string detectorType = "BRISK";
                //string detectorType = "ORB";
                //string detectorType = "AKAZE";
                //string detectorType = "SIFT";

                //Switch based on the detector type
                switch (iDetectorIndex)
                {
                case detector_SHITOMASI:
                    detectorType = "SHITOMASI";
                    break;
                case detector_HARRIS:
                    detectorType = "HARRIS";
                    break;
                case detector_FAST:
                    detectorType = "FAST";
                    break;
                case detector_BRISK:
                    detectorType = "BRISK";
                    break;
                case detector_ORB:
                    detectorType = "ORB";
                    break;
                case detector_AKAZE:
                    detectorType = "AKAZE";
                    break;
                case detector_SIFT:
                    detectorType = "SIFT";
                    break;                            
                default:
                    std::cout<< "Dector Error     : Does not match implemented method, please check: "<<detectorType<< endl;
                    cv::waitKey(0); // wait for key to be pressed
                    return 1;                
                }


                //// STUDENT ASSIGNMENT
                //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
                //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

                if( bConsoleLogging ){
                    std::cout<< "Detector Type    : " << detectorType << endl;
                }

                if (detectorType.compare("SHITOMASI") == 0)
                {
                    // Detect keypoints in image using the traditional SHITOMASI detector
                    detKeypointsShiTomasi(keypoints, imgGray, bVis);
                }
                else if(detectorType.compare("HARRIS") == 0)
                {
                    // Detect keypoints in image using the traditional HARRIS detector
                    detKeypointsHarris(keypoints, imgGray, bVis);
                }
                else if(detectorType.compare("FAST") == 0)
                {
                    // Detect keypoints in image using the FAST detector
                    detKeypointsFAST(keypoints, imgGray, bVis);
                }
                else if(detectorType.compare("BRISK") == 0)
                {
                    // Detect keypoints in image using the BRISK detector
                    detKeypointsBRISK(keypoints, imgGray, bVis);
                }
                else if(detectorType.compare("ORB") == 0)
                {
                    // Detect keypoints in image using the ORB detector
                    detKeypointsORB(keypoints, imgGray, bVis);
                }
                else if(detectorType.compare("AKAZE") == 0)
                {
                    // Detect keypoints in image using the AKAZE detector
                    detKeypointsAKAZE(keypoints, imgGray, bVis);
                }
                else if(detectorType.compare("SIFT") == 0)
                {
                    // Detect keypoints in image using the SIFT detector
                    detKeypointsSIFT(keypoints, imgGray, bVis);
                }
                else{
                    std::cout<< "Dector Error     : Does not match implemented method, please check: "<<detectorType<< endl;
                    cv::waitKey(0); // wait for key to be pressed
                    return 0;
                }

                if( bConsoleLogging ){
                    cout << "--------2--------: Done" << endl;
                
                //// EOF STUDENT ASSIGNMENT

                //// STUDENT ASSIGNMENT
                //// TASK MP.3 -> only keep keypoints on the preceding vehicle

                    cout << "        3        : RETAIN BBOX KEYPOINTS" << endl;
                }

                // only keep keypoints on the preceding vehicle
                bool bFocusOnVehicle = true;
                
                //Vehicle Bounding box
                cv::Rect vehicleRect(535, 180, 180, 150);
                if (bFocusOnVehicle)
                {
                    //Remove the keypoints outside the rectangle            
                    for(auto i = keypoints.begin(); i != keypoints.end(); i++){
                        //check if the keypoint is not within the defined box
                        if(!vehicleRect.contains((*i).pt)) 
                        {
                            //Erase the keypoint and decrement the ittrator
                            keypoints.erase(i--);
                        }
                    }

                    // //retaining the keypoints in the bounding box
                    // vector<cv::KeyPoint> toBeRetained;
                    // for(int i = 0; i < keypoints.size(); i++){
                    //     //check if the keypoint is within the defined box, if not remove this keypoint
                    //     if(vehicleRect.contains(keypoints[i].pt)) 
                    //     {
                    //         toBeRetained.push_back(keypoints[i]);
                    //     }                
                    // }   
                    // //Removing all the keypoints         
                    // keypoints.clear();  
                    // //Cpying back all to be retained keypoints
                    // for(auto i = toBeRetained.begin(); i != toBeRetained.end(); i++ )
                    // {
                    //     keypoints.push_back(*i);
                    // } 
                    // //Removing all the keypoints
                    // toBeRetained.clear();
                                                                                                                   

                    if (bVehicleBBox)
                    {
                        cv::Mat visImage = img.clone();
                        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                        string windowName = "Only in BBox";
                        cv::rectangle( visImage, vehicleRect.tl(), vehicleRect.br(), (0,0,255), 2 );
                        cv::namedWindow(windowName, 6);
                        imshow(windowName, visImage);
                        cv::waitKey(0);
                    }
                }

                //// EOF STUDENT ASSIGNMENT
                // optional : limit number of keypoints (helpful for debugging and learning)
                bool bLimitKpts = true;
                if (bLimitKpts)
                {
                    int maxKeypoints = 50;   
                    /////NOTE: Removed the condition for only SHITOMASI, since this is common for all methods if we want to retain only 50 samples              
                    //if (detectorType.compare("SHITOMASI") == 0)
                    
                    // there is no response info, so keep the first 50 as they are sorted in descending quality order
                    keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                    cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                    cout << "Note             : Keypoints have been limited!" << endl;
                }

                // push keypoints and descriptor for current frame to end of data buffer
                //Keypoints of the current frame are assigned to the last item in the buffer 
                (dataBuffer.end() - 1)->keypoints = keypoints;
                
                if( bConsoleLogging ){
                    cout << "--------3--------: Done" << endl;
                    cout << "        4        : EXTRACT DESCRIPTORS" << endl;
                }

                /* EXTRACT KEYPOINT DESCRIPTORS */

                //// STUDENT ASSIGNMENT
                //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
                //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

                cv::Mat descriptors;
                string descriptorType = "";
                //string descriptorType = "BRISK"; // BRIEF, ORB, FREAK, AKAZE, SIFT
                //string descriptorType = "BRIEF";
                //string descriptorType = "ORB";
                //string descriptorType = "FREAK";
                //string descriptorType = "AKAZE";
                //string descriptorType = "SIFT";


                //Switch based on the descriptor type
                switch (iDescriptorIndex)
                {
                case descriptor_BRISK:
                    descriptorType = "BRISK";
                    break;
                case descriptor_BRIEF:
                    descriptorType = "BRIEF";
                    break;
                case descriptor_ORB:
                    descriptorType = "ORB";
                    break;
                case descriptor_FREAK:
                    descriptorType = "FREAK";
                    break;
                case descriptor_AKAZE:
                    descriptorType = "AKAZE";
                    break;
                case descriptor_SIFT:
                    descriptorType = "SIFT";
                    break;                            
                default:
                    std::cout<< "Descriptor Error  : Does not match implemented method, please check: "<<descriptorType<< endl;
                    cv::waitKey(0); // wait for key to be pressed
                    return 1;                
                }

                if( bConsoleLogging ){   
                    std::cout<< "Discriptor Type  : " << descriptorType << endl;       
                }
                
                descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);

                //// EOF STUDENT ASSIGNMENT

                // push descriptors for current frame to end of data buffer
                (dataBuffer.end() - 1)->descriptors = descriptors;

                if( bConsoleLogging ){

                    cout << "--------4--------: done" << endl;
                    cout << "       5/6       : MATCH KEYPOINT DESCRIPTORS" << endl;
                }

                if (dataBuffer.size() > 1) // wait until at least two images have been processed
                {

                    /* MATCH KEYPOINT DESCRIPTORS */

                    vector<cv::DMatch> matches;
                    string matcherType      = "MAT_BF";         // MAT_BF, MAT_FLANN
                    string descriptorType   = "DES_BINARY";     // DES_BINARY, DES_HOG
                    string selectorType     = "SEL_NN";         // SEL_NN, SEL_KNN


                    if( bConsoleLogging ){
                        std::cout<< "Macher Type      : " << matcherType << endl;
                        std::cout<< "Selector Type    : " << selectorType << endl;
                    }


                    //// STUDENT ASSIGNMENT
                    //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                    //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

                    matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                    (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                    matches, descriptorType, matcherType, selectorType);

                    //// EOF STUDENT ASSIGNMENT

                    // store matches in current data frame
                    (dataBuffer.end() - 1)->kptMatches = matches;

                    if( bConsoleLogging ){
                        cout << "-------5/6-------: done" << endl;
                    }

                    // visualize matches between current and previous image
                    bVis = true;
                    if (bVis)
                    {
                        cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                        cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                        (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                        matches, matchImg,
                                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                                        vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                        string windowName = "Matching keypoints between two camera images";
                        cv::namedWindow(windowName, 7);
                        cv::imshow(windowName, matchImg);
                        cout << "-----------------: Press key to continue to next image" << endl;
                        cv::waitKey(0); // wait for key to be pressed
                    }
                    bVis = false;
                }

            } // eof loop over all images
        }
    }

    return 0;
}
