/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include "matching2D.hpp"
#include "dataStructures.h"


/* Flag for compiling the code for logging the results in the csv files */
#define _GENERATE_RESULTS_FILE_        // Result files to be created 

#ifdef _GENERATE_RESULTS_FILE_
//For creating thge directories
#include <experimental/filesystem> 
namespace fs = std::experimental::filesystem;
#endif //end _GENERATE_RESULTS_FILE_

using namespace std;


/* MAIN PROGRAM */
int main(int argc, const char *argv[]){

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    //Filepath for the results directory
    string resutBasePath = dataPath + "results/";
    string keypointsPath = resutBasePath + "keypoints/";
    string matchedPath   = resutBasePath + "matches/";
    
    //camera
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
    bool bConsoleLogging = false;   // Console minimum logging
    


    double totalTime    = 0;        // Total time taken
    int keypointTotal   = 0;        // Total keypoints
    int matchesTotal    = 0;        // Total matches
    int iCombinationIndex = 0;      // Combination index for Sl number for the outer loop

    //Name of the files created
    string resultFileName(resutBasePath + "FinalResults.txt");


#ifdef _GENERATE_RESULTS_FILE_
    //Directory reation
    if (!fs::is_directory(resutBasePath) || !fs::exists(resutBasePath)) 
    { 
        // Check if src folder exists
        fs::create_directory(resutBasePath); // create src folder
    }
    if (!fs::is_directory(keypointsPath) || !fs::exists(keypointsPath)) 
    { 
        // Check if src folder exists
        fs::create_directory(keypointsPath); // create src folder
    }
    if (!fs::is_directory(matchedPath) || !fs::exists(matchedPath)) 
    { 
        // Check if src folder exists
        fs::create_directory(matchedPath); // create src folder
    }
#endif //end _GENERATE_RESULTS_FILE_


    string detectorType = "";
    string descriptorType = "";


    //TODO:

    //Clean up the code:

     //8. CSV File operation 
        //Put counters and timers for the table 
        //For each pair of detector / descriptor log for a table: Form a CSV File: (for logging the top 3-5 in each category)
        //One file each for 1 & 2 
        //1. Method pair, Number of keypoints image, and Distribution of neighbourhood size, feature/keypoint size(), variance, mean, distribution, time taken for tetector and discriptor 
        //avarage time taken per keypoint over the entire method pair (Accumulate)
        //2. Number of match keypoints 1-2, 2-3, 3-4.. matched, time taken per keypoint
    //13. Run the detector type and other loops
    //14. Rectify bVis why this does not work
    //3. The way to go about each of the activities, make a switch between point number 1-6 and then 7-9
    //5. Clean up the loop for enum
    //6. Clean up the cv::waitkey mess, make it switchable. 
    //7. Integrate a logging header (spdlog, as external)
    //9. C++ Design as much.. (see how that can be thread safe???)
    //10. How this can run in distributed settings ( and state machine settings??)
    //11. Mostly return only error codes, not values of the functions, return value must be through pass by refrence
    //12. Crash in macher when we run the for loop in different combinations( brief and shitomasi as en example)
    //2. Clean up the comments (Later)
    
   


    //For running the loops
    int iDetectorIndex      = detector_SHITOMASI;
    int iDescriptorIndex    = descriptor_BRIEF;
    //     det(detector_SHITOMASI,= 1) \
    //     det(detector_HARRIS,) \
    //     det(detector_FAST,) \
    //     det(detector_BRISK,) \
    //     det(detector_ORB,) \
    //     det(detector_AKAZE,) \
    //     det(detector_SIFT,) \
    //     des(descriptor_BRISK,= 1) \
    //     des(descriptor_BRIEF,) \
    //     des(descriptor_ORB,) \
    //     des(descriptor_FREAK,) \
    //     des(descriptor_AKAZE,) \
    //     des(descriptor_SIFT,) \
    //     desType(descriptorType_DES_BINARY,= 1) \
    //     desType(descriptorType_HOG,) 
    //     matcher(matcher_MAT_BF,= 1) \
    //     matcher(matcher_MAT_FLANN,) 
    //     selector(selector_SEL_NN,= 1) \
    //     selector(selector_SEL_KNN,) 





#ifdef _GENERATE_RESULTS_FILE_
    //Final file for total sum per pair to compare:
    ofstream resultFile;
    //std::ios::app|
    resultFile.open(resultFileName.c_str(),ios::out);
    
    //Append the header
    resultFile<<setw(SWL_SERIAL_NO)<<std::right<<"Sl No |"<<std::setw(SWL_DETECTOR)<<std::right<<"Detector |"<<std::setw(SWL_DESCRIPTOR)<<std::right<<"Descriptor |"
                    <<std::setw(SWL_TL_KEYPOINTS) <<std::right<< " Total Keypoints |"<<std::setw(SWL_TL_MATCHES)<<std::right<< "Total Matches |" 
                        <<std::setw(SWL_TL_TIME)<<std::right<< "Total Time (ms) |" <<std::setw(SWL_TL_RATIO)<<std::right<< "Ratio (matches/time) |" << std::endl;
    //Append the partition
    resultFile<<setw(SWL_SERIAL_NO)<<std::right<<"---- |"<<std::setw(SWL_DETECTOR)<<"---- |"<<std::setw(SWL_DESCRIPTOR)<<"---- |"<<std::setw(SWL_TL_KEYPOINTS)<<"---- |"
                    <<std::setw(SWL_TL_MATCHES)<<"---- |"<<std::setw(SWL_TL_TIME)<<"---- |"<<std::setw(SWL_TL_RATIO)<<"---- |"<<std::endl;
#endif //end _GENERATE_RESULTS_FILE_


    for (int iDetectorIndex = detector_SHITOMASI; iDetectorIndex <= detector_SIFT; ++iDetectorIndex)
    {

        // cout<<GetString((Detectors)iDetectorIndex)<<endl;
        // // descriptor_BRISK
        for (int iDescriptorIndex = descriptor_BRIEF; iDescriptorIndex <= descriptor_SIFT; ++iDescriptorIndex)
        {
            //detector_SHITOMASI->descriptor_FREAK
            //detector_SHITOMASI->descriptor_BRIEF after BRISK
            //detector_SHITOMASI->descriptor_SIFT
            //detector_HARRIS->descriptor_FREAK
            if ((iDescriptorIndex == descriptor_AKAZE && iDetectorIndex != detector_AKAZE) ||
                (iDescriptorIndex == descriptor_ORB   && iDetectorIndex == detector_SIFT)  )

            {
                // AKAZE descriptor extractor works only with key-points detected with KAZE/AKAZE detectors
                // see https://docs.opencv.org/3.0-beta/modules/features2d/doc/feature_detection_and_description.html#akaze
                
                // ORB descriptor extractor does not work with the SIFT detetor
                // see https://answers.opencv.org/question/5542/sift-feature-descriptor-doesnt-work-with-orb-keypoinys/
                continue;
            }


            // for (int iMatcherIndex = matcher_MAT_BF; iMatcherIndex <= matcher_MAT_FLANN; ++iMatcherIndex)
            // {
            //     for (int iSelecectorIndex = selector_SEL_NN; iSelecectorIndex <= selector_SEL_KNN; ++iSelecectorIndex)
            //     {
            //         //Call compatible decriptor type?
            //     }
            // }

            //Unique strings for discriptor and detectors 
            string uniqueDetector   = get_right_of_delim(GetString((Detectors)iDetectorIndex), "detector_");
            string uniqueDescriptor = get_right_of_delim(GetString((Descriptors)iDescriptorIndex), "descriptor_");


#ifdef _GENERATE_RESULTS_FILE_
            //Log names 
            string csvExtns(".csv");
            string underscoreExt("_");

            //CSV File for each pair of detectors and descriptors  
            //Csv file name name 
            
            string csvPairFileName = uniqueDetector+underscoreExt+uniqueDescriptor+csvExtns;

            //iCombinationIndex+1<<","
            string perPairFilePath(matchedPath + csvPairFileName);

            ofstream perPairFile;
            //std::ios::app|
            perPairFile.open(perPairFilePath.c_str(),ios::out);
            //Append the header
            perPairFile<<"Image 1 Index,"<<"Image 2 Index,"<<"Matched Keypoints,"<<"Ratio (matches/time)," << std::endl; 

            string csvKeypointsFileName = uniqueDetector+csvExtns;
            string keypointFilePath(keypointsPath + csvKeypointsFileName);

            std::cout<<"Generating file:"<<csvPairFileName<<"..."<<csvKeypointsFileName<<".."<<endl;

            ofstream keyPointFile;
            //std::ios::app|
            keyPointFile.open(keypointFilePath.c_str(),ios::out);
            //Append the header
            keyPointFile<<"Image index,"<<"BBox Keypoints,"<<"Distribution of Neighbourhood,"<<"Time taken per Keypoint"<< std::endl;

#endif //end _GENERATE_RESULTS_FILE_


            //8. Count the number of matched keypoints for all 10 images using all possible combinations 
            //of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.time taken per keypoint



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

                //// STUDENT ASSIGNMENT
                //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
                //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
                if( bConsoleLogging ){
                    std::cout<< "Detector Type    : " << detectorType << endl;
                }

                                // extract 2D keypoints from current image
                vector<cv::KeyPoint> keypoints; // create empty feature list for current image

                //To accumulate the detection time taken
                double detectorTime = 0;
                detectorType = uniqueDetector;

                //Detect keypoint function for allthe keypoint detector 
                //Note: Replaced the switch with a function to clean up the code
                detectorTime = detectKeypoints(keypoints, imgGray, iDetectorIndex, bVis, bConsoleLogging);

               
                //Calculating the total keypoints
                keypointTotal += keypoints.size();
                if( bConsoleLogging ){
                    cout << "--------2--------: Done" << endl;                
                //// EOF STUDENT ASSIGNMENT
                //// STUDENT ASSIGNMENT
                //// TASK MP.3 -> only keep keypoints on the preceding vehicle
                    cout << "        3        : RETAIN BBOX KEYPOINTS" << endl;
                }
                // only keep keypoints on the preceding vehicle
                bool bFocusOnVehicle = false;                
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
                bool bLimitKpts = false;
                if (bLimitKpts)
                {
                    int maxKeypoints = 50;   
                    /////NOTE: Removed the condition for only SHITOMASI, since this is common for all methods if we want to retain only 50 samples              
                    //if (detectorType.compare("SHITOMASI") == 0)
                    // there is no response info, so keep the first 50 as they are sorted in descending quality order
                    keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                    cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                    if( bConsoleLogging ){
                        cout << "Note             : Keypoints have been limited!" << endl;
                    }
                }
                // Keypoints of the current frame are assigned to the last item in the buffer 
                // push keypoints and descriptor for current frame to end of data buffer
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

                //Switch has been replaced by precomputed unique Descriptor and equated 
                descriptorType = uniqueDescriptor;

                if( bConsoleLogging ){   
                    std::cout<< "Discriptor Type  : " << descriptorType << endl;       
                }
                //Descriptor time for accumulating the time taken by the descriptor extraction
                double descriptorTime = 0;
                descriptorTime = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType, bConsoleLogging);                              
                
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
                    //// STUDENT ASSIGNMENT//////////////////////////////////////////////////////////////////////////////////////////////////////////////////TODO, Uncomment Matcher
                    //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                    //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
                    // matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                    //                 (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                    //                 matches, descriptorType, matcherType, selectorType);
                    //Total maches
                    matchesTotal += matches.size();

                    //7.Count the number of keypoints on the preceding vehicle for all 10 images and take 
                    //note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.     
                    

                    //What is the neighborhood size? In the case of Shi-Tomasi and Harris detectors, I believe it refers to parameter blockSize; 
                    //but how is it defined/calculated for the other detectors?

                    //distribution of their neighborhood size
                    //Indeed in the case of Shi-Tomasi and Harris detectors it refers to the blockSize parameter. 
                    //In this example of SIFT you can check that the neighborhood is represented by the meaningful keypoint size and represented 
                    //by the circle radius around the detected keypoint. Therefore, you can access this feature using the size() 
                    //parameter of the keypoints. In order to log it using the following code snippet, to be added once cleaned the unnecessary keypoints. 
                    //keypoint.size();

                    //https://stackoverflow.com/questions/10328298/what-does-size-and-response-exactly-represent-in-a-surf-keypoint
                    //(size is the diameter of the keypoint)
                    //(response is the response by which the most strong keypoints have been selected)
                    //Distribution of the neighbourhood size: Mean and varience and SD of each keypoint, over its descriptors, and error from the mean 
                    

                    // double mean = std::accumulate(keypoints.size.begin(), keypoints.size.end(), 0.0)/keypoints.size();
                    // auto add_square = [mean](double sum, int i)
                    // {
                    //     auto d = i - mean;
                    //     return sum + d*d;
                    // };
                    
                    // double total = std::accumulate(keypoints.size.begin(), keypoints.size.end(), 0.0, add_square);
                    // double variance = total / keypoints.size();
                    // Also, for each of the combination play with parameters to improve the results

                    //8. Count the number of matched keypoints for all 10 images using all possible combinations 
                    //of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.time taken per keypoint

#ifdef _GENERATE_RESULTS_FILE_
                    //Comma seperated file
                    perPairFile<<iCombinationIndex+1<<","<<uniqueDetector<<","<<uniqueDescriptor<<","<<
                                    std::setw(23)<<std::right <<keypointTotal<<","<<std::setw(23)<<std::right<<matchesTotal<<","
                                        <<std::setw(23)<<std::right<<totalTime * 1000.0<<","<< std::setw(23)<<std::right<< 
                                            ((totalTime!=0)? (matchesTotal / (totalTime * 1000.0)):0) <<","<<std::endl;

                    keyPointFile<<imgIndex<<","<<keypoints.size()<<","<<"Distribution of Neighbourhood,"<<((keypoints.size()!=0)? ((detectorTime * 1000.0)/keypoints.size()):0)<< std::endl;
#endif //end _GENERATE_RESULTS_FILE_
                    


                    //// EOF STUDENT ASSIGNMENT
                    // store matches in current data frame
                    (dataBuffer.end() - 1)->kptMatches = matches;
                    if( bConsoleLogging ){
                        cout << "-------5/6-------: done" << endl;
                    }
                    // visualize matches between current and previous image
                    bVis = false;
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
                totalTime += detectorTime + descriptorTime;


                if( bConsoleLogging ){
                    cout << "Total time       : Detector + Descriptor + Matcher is " << totalTime * 1000.0 << " ms \n";
                }
    
            //9. Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and 
            //based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.
            //Will not concentrate on accuracies and Reciever opperating charecteristics 
            //Estimating homographies on planner surfaces ( to compute the ground truth extraction)

            } // eof loop over all images
            
            


#ifdef _GENERATE_RESULTS_FILE_
            //Close the file
            perPairFile.close();
            keyPointFile.close();

            //File for the total update of the run
            //Append the new pair of data
            resultFile<<setw(SWL_SERIAL_NO-2)<<std::right<<iCombinationIndex+1<<" |"<<std::setw(SWL_DETECTOR-2)<<std::right
                        <<uniqueDetector<< " |"<<std::setw(SWL_DESCRIPTOR-2)<<std::right<<uniqueDescriptor<<" |" <<
                            std::setw(SWL_TL_KEYPOINTS-2)<<std::right <<keypointTotal<<" |"<<std::setw(SWL_TL_MATCHES-2)
                                <<std::right<<matchesTotal<<" |" <<std::setw(SWL_TL_TIME-2)<<std::right<<totalTime * 1000.0<<" |"
                                    << std::setw(SWL_TL_RATIO-2)<<std::right<< ((totalTime!=0)? (matchesTotal / (totalTime * 1000.0)):0) <<" |"<<std::endl;

#endif //end _GENERATE_RESULTS_FILE_
            //For index of the combination run
            iCombinationIndex++;

        }

    }

#ifdef _GENERATE_RESULTS_FILE_   
    //Close the file
    resultFile.close();    
#endif //end _GENERATE_RESULTS_FILE_

    cv::waitKey(0);
    return 0;
}
