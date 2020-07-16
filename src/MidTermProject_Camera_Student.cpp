/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <numeric> 
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include "matching2D.hpp"
#include "dataStructures.h"


// Flag for compiling the code for logging the results in the csv and log files 
#define _GENERATE_RESULTS_FILE_

//Flag for testing without loop for easy debugging ehen something to be experimented 
//for a single detector/descriptor pair
#define _testWithoutLoop_   

#ifdef _GENERATE_RESULTS_FILE_
//For creating thge directories
#include <experimental/filesystem> 
namespace fs = std::experimental::filesystem;
#endif //end _GENERATE_RESULTS_FILE_

using namespace std;


    //TODO:
    //Clean up the code:
    //8. CSV File operation 
        //Final txt file mostly done...check for what information is useful and dicard the rest
    //15. Make batch script to have call option to generate files or not 
    //      (Make a switch between point number 1-6 and then 7-9 (take command line arguments to switch) )
    //16. (pass in the function with single argument for both flows, and have logging also tie to the same)   

    //7.  Integrate a logging header (spdlog, as external)
    //9.  C++ Design as much.. (see how that can be thread safe???)
    //10. How this can run in distributed settings ( and state machine settings??)
    //11. Mostly return only error codes, not values of the functions, return value must be through pass by refrence
    //14. Rectify bVis why this does not work


///*************************************************************************************
/// \brief runKeypointMatching: This function runs keypoints detection, descriptor 
///                                 extraction and maching methods 
///                                 Only run for 10 images as the pipeline integration 
///                                 of the selected keypoints method 
///                                 (console logging enabled. Only for tasks 1 to 6)
///
/// \return: returns the status of the method, if itwas sucessfull of failed anywhere 
///*************************************************************************************
int runKeypointMatching();

///*************************************************************************************
/// \brief compareKeypointMatching: This function compares different keypoints 
///                                 detection, description and maching methods
///                                 Run for all possible methods generating files for 
//                                  the analysis (console logging Disabled. Only for 
//                                  tasks 7 to 9) 
/// \return: returns the status of the method, if itwas sucessfull of failed anywhere 
///*************************************************************************************
int compareKeypointMatching();

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    
    //Parsing command line arguments
    if (argc<2||argc>2)
    {
		std::cout <<"Usage: " << argv[0] << " 0 or 1, to generate method comparison files. Please check your inputs.."<<std::endl;
        return 1;
    }    
    else
    {   
        //If the arguments are correct 
        std::string options(argv[1]);     
        if (options == "0")
        {
            //Only run for 10 images as the pipeline integration of the selected keypoints method 
            //(console logging enabled. Only for taski 1 to 6)
            runKeypointMatching();
        }
        else if (options == "1")
        {
            //Run for all possible methods generating files for the analysis
            //(console logging Disabled. Only for tasks 7 to 9)
            compareKeypointMatching();
        }
        else
            std::cout <<"Usage: " << argv[0] << " 0 or 1, to generate method comparison files. Please check your inputs.."<<std::endl;
    }
    cv::waitKey(0);
    return 0;
}


///*************************************************************************************
/// \brief compareKeypointMatching: This function compares different keypoints 
///                                 detection, description and maching methods
///                                 Run for all possible methods generating files for 
///                                 the analysis (console logging Disabled. Only for 
///                                 tasks 7 to 9) 
/// \return: returns the status of the method, if itwas sucessfull of failed anywhere 
///*************************************************************************************
int compareKeypointMatching()
{
    /* INIT VARIABLES AND DATA STRUCTURES */
    // data location
    string dataPath = "../";

    bool bGenerateFiles = true;

    cout<<"Compareing...."<<endl;

    //Filepath for the results directory
    string resutBasePath = dataPath + "results/";
    string keypointsPath = resutBasePath + "keypoints/";
    string matchedPath   = resutBasePath + "matches/";
    
    //camera
    string imgBasePath  = dataPath + "images/";
    string imgPrefix    = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType  = ".png";
    int imgStartIndex   = 0;        // first file index to load (assumes Lidar and camera names 
                                    // have identical naming convention)
    int imgEndIndex     = 9;        // last file index to load
    int imgFillWidth    = 4;        // no. of digits which make up the file index (e.g. img-0001.png)

    bool bVis               = false; // visualize results
    bool bVehicleBBox       = false; // visualize only vehicle keypoints
    bool bConsoleLogging    = false; // Console minimum logging   

    int iCombinationIndex   = 0;     // Combination index for Sl number for the outer loop
    int iPrevDetector       = 0;

    //Name of the files created
    string resultFileName(resutBasePath + "FinalResults.txt");


#ifdef _GENERATE_RESULTS_FILE_
    if(bGenerateFiles)
    {
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
    }
#endif //end _GENERATE_RESULTS_FILE_


    string detectorType     = "";
    string descriptorType   = "";       

    //For running the discriptor detector indipendently for testing when not in loop
    //detector_SHITOMASI, detector_HARRIS, detector_FAST, detector_BRISK, detector_ORB, detector_AKAZE, detector_SIFT
    //descriptor_BRISK, descriptor_BRIEF, descriptor_ORB, descriptor_FREAK, descriptor_AKAZE, descriptor_SIFT
    int iDetectorIndex      = detector_FAST;
    int iDescriptorIndex    = descriptor_SIFT;

    
#ifdef _GENERATE_RESULTS_FILE_
    //Final file for total sum per pair to compare:
    ofstream resultFile;
    if(bGenerateFiles)
    {
        //std::ios::app|
        resultFile.open(resultFileName.c_str(),ios::out);
        
        //Append the header
        resultFile<<setw(SWL_SERIAL_NO)<<std::right<<"Sl No |"<<std::setw(SWL_DETECTOR)<<std::right<<"Detector |"
                <<std::setw(SWL_DESCRIPTOR)<<std::right<<"Descriptor |"
                    <<std::setw(SWL_TL_KEYPOINTS) <<std::right<< "T Keypnts |"<<std::setw(SWL_TIME_P_KEYP)<<"Tme P Keypnt |"
                        <<std::setw(SWL_TL_MATCHES)<<std::right<< "T Matches |"<<std::setw(SWL_TIME_P_MATCH)<<std::right <<"Tme P Mtch |"
                            <<std::setw(SWL_TL_PPLIN_TME)<<std::right<< "Ttl tme 10 frms ms |" << std::endl;
        //Append the partition
        resultFile<<setw(SWL_SERIAL_NO)<<std::right<<"---- |"<<std::setw(SWL_DETECTOR)<<"---- |"
                <<std::setw(SWL_DESCRIPTOR)<<"---- |"
                    <<std::setw(SWL_TL_KEYPOINTS)<<"---- |"<<std::setw(SWL_TIME_P_KEYP)<<"---- |"
                        <<std::setw(SWL_TL_MATCHES)<<"---- |"<<std::setw(SWL_TIME_P_MATCH)<<std::right <<"---- |"
                            <<std::setw(SWL_TL_PPLIN_TME)<<"---- |"<<std::endl;
    }
#endif //end _GENERATE_RESULTS_FILE_


#ifdef _testWithoutLoop_

    for (int iDetectorIndex = detector_SHITOMASI, iPrevDetector = detector_SHITOMASI; iDetectorIndex <= detector_SIFT; ++iDetectorIndex)
    {
        for (int iDescriptorIndex = descriptor_BRISK; iDescriptorIndex <= descriptor_SIFT; ++iDescriptorIndex)
        {

            if ((iDescriptorIndex == descriptor_AKAZE && iDetectorIndex != detector_AKAZE) ||
                (iDescriptorIndex == descriptor_ORB   && iDetectorIndex == detector_SIFT) )

            {
                //https://answers.opencv.org/question/5542/sift-feature-descriptor-doesnt-work-with-orb-keypoinys/
                // SIFT detetor does not ORB descriptor
                // https://docs.opencv.org/3.0-beta/modules/features2d/doc/feature_detection_and_description.html#akaze
                // For AKAZE descriptor extractor we have to only use AKAZE or KAZE detectors
                continue;
            }

#endif //end _testWithoutLoop_

            // Moving Data buffer inside the loop for reset of images for next set of pairs
            int dataBufferSize = 2;         // no. of images which are held in memory (ring buffer) at the same time
            vector<DataFrame> dataBuffer ;  // list of data frames which are held in memory at the same time



            //Unique strings for discriptor and detectors 
            string uniqueDetector   = get_right_of_delim(GetString((Detectors)iDetectorIndex), "detector_");
            string uniqueDescriptor = get_right_of_delim(GetString((Descriptors)iDescriptorIndex), "descriptor_");
            
            //Time for different aspents of programe for measuring performance
            double totalMatcherTime     = 0;
            double totalDetectorTime    = 0;
            double totalDescriptorTime  = 0; 
            double totalPipelineTime    = 0;  // Total time taken by the pipeline

            double detectorTime         = 0;
            double descriptorTime       = 0;
            double matcherTime          = 0;

            int keypointDetected        = 0;
            int keypointTotal           = 0;  // Total keypoints
            int matchesTotal            = 0;  // Total matches



#ifdef _GENERATE_RESULTS_FILE_
            ofstream perPairFile;
            ofstream keyPointFile;
            if(bGenerateFiles)
            {
                string csvExtns(".csv");
                string underscoreExt("_");

                //CSV File for each pair of detectors and descriptors                
                string csvPairFileName = uniqueDetector+underscoreExt+uniqueDescriptor+csvExtns;

                //iCombinationIndex+1<<","
                string perPairFilePath(matchedPath + csvPairFileName);


                //std::ios::app|
                perPairFile.open(perPairFilePath.c_str(),ios::out);
                //Append the header
                perPairFile<<"Img 1 Idx,"<<"Img 2 Idx,"<<"Img Kypnts,"<<"Mtchd Kypnts," <<"Mtchd tme tkn"<< std::endl; 

                string csvKeypointsFileName = uniqueDetector+csvExtns;
                string keypointFilePath(keypointsPath + csvKeypointsFileName);

                std::cout<<"Generating files : "<<csvPairFileName<<"... "<<csvKeypointsFileName<<".."<<endl;


                keyPointFile.open(keypointFilePath.c_str(),std::ios::app|ios::out);
    
                //Making sure for each detector, only once the header is generated
                if(iPrevDetector!=iDetectorIndex)
                {
                    iPrevDetector = iDetectorIndex;
                    //Append the header
                    keyPointFile<<"Img Idx,"<<"Kypnts in BBox,"<<"Tme P Kypnt,"<<"Mean of NS,"<<"Median of NS,"<<"SD of NS"<<std::endl;
                }
            }

#endif //end _GENERATE_RESULTS_FILE_


//#ifdef CodeNotCompile

            /* MAIN LOOP OVER ALL IMAGES */
            for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
            {
                //For logging
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

                //Impletemted a Ring buffer on vector, erase the first element in the vector, which is the oldest 
                if (dataBuffer.size() > dataBufferSize)
                {
                    dataBuffer.erase(dataBuffer.begin());
                }                
                //// EOF STUDENT ASSIGNMENT

                //For logging
                if( bConsoleLogging ){
                    cout<< "Databuffer Size  : "<< dataBuffer.size() << endl;
                    cout << "        2        : DETECT IMAGE KEYPOINTS" << endl;
                }
                

                /* DETECT IMAGE KEYPOINTS */
                //// STUDENT ASSIGNMENT
                //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based 
                ////  selection based on detectorType
                //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
                if( bConsoleLogging ){
                    std::cout<< "Detector Type    : " << uniqueDetector << endl;
                }

                // extract 2D keypoints from current image
                vector<cv::KeyPoint> keypoints; // create empty feature list for current image

                //To accumulate the detection time taken                
                detectorType = uniqueDetector;

                //Detect keypoint function for allthe keypoint detector 
                //Note: Replaced the switch with a function to clean up the code
                detectorTime = detectKeypoints(keypoints, imgGray, iDetectorIndex, bVis, bConsoleLogging);

                //Since we remove the keypints later, which should be also acounted for total time
                keypointDetected = keypoints.size();
                //Calculating the total keypoints
                keypointTotal += keypointDetected;
                
                //Summing the detector time
                totalDetectorTime += detectorTime;
              

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
                
                //7.Count the number of keypoints on the preceding vehicle for all 10 images and take 
                //note of the distribution of their neighborhood size. Do this for all the detectors you have implemented    
                //https://stackoverflow.com/questions/10328298/what-does-size-and-response-exactly-represent-in-a-surf-keypoint
                //For distribution of the keypoints, calculating the mean, median varience and standard diviation 

#ifdef _GENERATE_RESULTS_FILE_
                double mean, sum = 0;
                double stdev, variance, median;
                if(bGenerateFiles)
                {
                    std::vector<float> keypointSizes;
                    for (auto itterator : keypoints)  
                    {
                        sum += itterator.size; 
                        keypointSizes.push_back(itterator.size);                                
                    }
                    //Calculating the mean of the neighbourhood dristribution and varience
                    mean = sum/keypoints.size();

                    //Find Variance/standard deviation for the keypoint sizes
                    double sqSum = std::inner_product(keypointSizes.begin(), keypointSizes.end(), keypointSizes.begin(), 0.0);
                    variance = sqSum / keypointSizes.size() - mean * mean;
                    stdev = std::sqrt(variance);


                    // If the median is selected
                    bool findMedian = true; 
                    median = mean;
                    if(findMedian) 
                    {
                        //Find median for keypoint sizes
                        std::sort(keypointSizes.begin(), keypointSizes.end());
                        if (keypointSizes.size()%2==0)
                        {
                            median = 0.5 * (keypointSizes[keypointSizes.size() / 2 - 1] + 
                                keypointSizes[keypointSizes.size() / 2]);
                        }
                        else
                        {
                            median = keypointSizes[keypointSizes.size() / 2];
                        }
                    }
                }   
#endif //end _GENERATE_RESULTS_FILE_

                //// EOF STUDENT ASSIGNMENT

                // optional : limit number of keypoints (helpful for debugging and learning)
                bool bLimitKpts = false;
                if (bLimitKpts)
                {
                    int maxKeypoints = 50;   
                    //NOTE: Removed the condition for only SHITOMASI, since this is common 
                    //      for all methods if we want to retain only 50 samples                    
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
                    std::cout<< "Discriptor Type  : " << uniqueDescriptor << endl;       
                }
                //Descriptor time for accumulating the time taken by the descriptor extraction
                descriptorTime = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType, bConsoleLogging);                              
                
                //Summing the detector time
                totalDescriptorTime += descriptorTime;

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
                    //string matcherType      = "MAT_BF";         // MAT_BF, MAT_FLANN //For SIFT MAT_FLANN
                    
                    
                    //As discribed erlier, for SIFT based keypoints, L2 Norm and and FANN based methods used
                    //https://answers.opencv.org/question/10046/feature-2d-feature-matching-fails-with-assert-statcpp/
                    string matcherType      = CompatibleMatcherTypes((Descriptors)iDescriptorIndex);

                    string descriptorType   = "DES_BINARY";     // DES_BINARY, DES_HOG  
                    string selectorType     = "SEL_KNN";         // SEL_NN, SEL_KNN
                    if( bConsoleLogging ){
                        std::cout<< "Macher Type      : " << matcherType << endl;
                        std::cout<< "Selector Type    : " << selectorType << endl;
                    }

                    
                    //// STUDENT ASSIGNMENT//////////////////////////////////////////////////////////////////////////////////////////////////////////////////TODO, Uncomment Matcher
                    //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                    //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
                    matcherTime = matchDescriptors( (dataBuffer.end() - 2)->keypoints,  (dataBuffer.end() - 1)->keypoints,
                                                    (dataBuffer.end() - 2)->descriptors,(dataBuffer.end() - 1)->descriptors,
                                                    matches, descriptorType, matcherType, selectorType);
                    //Total maches
                    matchesTotal += matches.size();

                    //Summing the matcher time
                    totalMatcherTime += matcherTime;

                    //8. Count the number of matched keypoints for all 10 images using all possible combinations 
                    // of detectors and descriptors. In the matching step, the BF approach is used with the 
                    // descriptor distance ratio set to 0.8.
#ifdef _GENERATE_RESULTS_FILE_
                    if(bGenerateFiles)
                    {
                        //Comma seperated file for pair of detectors for performance of speed and total maches - total over 10 images
                        perPairFile<<imgIndex+1<<","<<imgIndex+2<<","
                            <<keypoints.size()<<","<<matches.size()<<","<<((matches.size()!=0)?(matcherTime/matches.size()):0)<<std::endl;
                    }
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

#ifdef _GENERATE_RESULTS_FILE_
                if(bGenerateFiles)
                {
                    //Comma seperated file per detector
                    keyPointFile<<imgIndex+1<<","<<keypoints.size()<<","
                        <<((keypointDetected!=0)? (detectorTime/keypointDetected):0)<<","
                            <<mean<<","<<median<<","<<stdev<<","<< std::endl;
                }
#endif //end _GENERATE_RESULTS_FILE_   

            } // eof loop over all images


            //Total Pipeline time
            totalPipelineTime = totalDetectorTime + totalDescriptorTime + totalMatcherTime;
            if( bConsoleLogging )
            {
                cout << "Total time       : Detector + Descriptor + Matcher is " << totalPipelineTime << " ms \n";
            }

//#endif //CodeNotCompile

            //9. Log the time it takes for keypoint detection and descriptor extraction. The results must 
            // be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor 
            // combinations must be recommended as the best choice for our purpose of detecting keypoints 
            // on vehicles. We will not concentrate on accuracies and Reciever opperating charecteristics 
            // Estimating homographies on planner surfaces (to compute the ground truth extraction)

#ifdef _GENERATE_RESULTS_FILE_
            if(bGenerateFiles)
            {
                //Close the file
                perPairFile.close();
                keyPointFile.close();

                //File for the total update of the run
                //Append the new pair of data
                resultFile<<setw(SWL_SERIAL_NO-2)<<std::right<<iCombinationIndex+1<<" |"
                        <<std::setw(SWL_DETECTOR-2)<<std::right<<uniqueDetector<< " |"
                            <<std::setw(SWL_DESCRIPTOR-2)<<std::right<<uniqueDescriptor<<" |" 
                                <<std::setw(SWL_TL_KEYPOINTS-2)<<std::right <<keypointTotal<<" |"<<std::setw(SWL_TIME_P_KEYP-2)<<std::right << ((keypointTotal!=0)?totalDetectorTime/keypointTotal:0)<<" |"
                                    <<std::setw(SWL_TL_MATCHES-2)<<std::right<<matchesTotal<<" |"<<std::setw(SWL_TIME_P_MATCH-2)<<std::right <<((matchesTotal!=0)?totalMatcherTime/matchesTotal:0)<<" |" 
                                        <<std::setw(SWL_TL_PPLIN_TME-2)<<std::right<<totalPipelineTime <<" |"<<std::endl;
            }
#endif //end _GENERATE_RESULTS_FILE_
            
            std::cout<<"\nDetector Type    : " << uniqueDetector 
                << "\nDiscriptor Type  : " << uniqueDescriptor <<"\nDone."<< endl;   
            //For index of the combination run
            iCombinationIndex++;

#ifdef _testWithoutLoop_
        }

    }
#endif  //end _testWithoutLoop_

#ifdef _GENERATE_RESULTS_FILE_   
    if(bGenerateFiles)
    {
        //Close the file
        resultFile.close();
    }
#endif //end _GENERATE_RESULTS_FILE_
    return 0;

}


///*************************************************************************************
/// \brief runKeypointMatching: This function runs keypoints detection, descriptor 
///                                 extraction and maching methods 
///                                 Only run for 10 images as the pipeline integration 
///                                 of the selected keypoints method 
///                                 (console logging enabled. Only for tasks 1 to 6)
///
/// \return: returns the status of the method, if itwas sucessfull of failed anywhere 
///*************************************************************************************
int runKeypointMatching()
{
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    bool bGenerateFiles = false;

    cout<<"Running only 10 images...."<<endl;

    //Filepath for the results directory
    string resutBasePath = dataPath + "results/";
    string keypointsPath = resutBasePath + "keypoints/";
    string matchedPath   = resutBasePath + "matches/";
    
    //camera
    string imgBasePath  = dataPath + "images/";
    string imgPrefix    = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType  = ".png";
    int imgStartIndex   = 0;        // first file index to load (assumes Lidar and camera names 
                                    // have identical naming convention)
    int imgEndIndex     = 9;        // last file index to load
    int imgFillWidth    = 4;        // no. of digits which make up the file index (e.g. img-0001.png)

    bool bVis               = false; // visualize results
    bool bVehicleBBox       = false; // visualize only vehicle keypoints
    bool bConsoleLogging    = true; // Console minimum logging   

    int iCombinationIndex   = 0;     // Combination index for Sl number for the outer loop
    int iPrevDetector       = 0;

    //Name of the files created
    string resultFileName(resutBasePath + "FinalResults.txt");


    string detectorType     = "";
    string descriptorType   = "";       

    //For running the discriptor detector indipendently for testing when not in loop
    //detector_SHITOMASI, detector_HARRIS, detector_FAST, detector_BRISK, detector_ORB, detector_AKAZE, detector_SIFT
    //descriptor_BRISK, descriptor_BRIEF, descriptor_ORB, descriptor_FREAK, descriptor_AKAZE, descriptor_SIFT
    int iDetectorIndex      = detector_FAST;
    int iDescriptorIndex    = descriptor_BRIEF;


    // Moving Data buffer inside the loop for reset of images for next set of pairs
    int dataBufferSize = 2;         // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer ;  // list of data frames which are held in memory at the same time



    //Unique strings for discriptor and detectors 
    string uniqueDetector   = get_right_of_delim(GetString((Detectors)iDetectorIndex), "detector_");
    string uniqueDescriptor = get_right_of_delim(GetString((Descriptors)iDescriptorIndex), "descriptor_");
    
    //Time for different aspents of programe for measuring performance
    double totalMatcherTime     = 0;
    double totalDetectorTime    = 0;
    double totalDescriptorTime  = 0; 
    double totalPipelineTime    = 0;  // Total time taken by the pipeline

    double detectorTime         = 0;
    double descriptorTime       = 0;
    double matcherTime          = 0;

    int keypointDetected        = 0;
    int keypointTotal           = 0;  // Total keypoints
    int matchesTotal            = 0;  // Total matches





    /* MAIN LOOP OVER ALL IMAGES */
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        //For logging
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

        //Impletemted a Ring buffer on vector, erase the first element in the vector, which is the oldest 
        if (dataBuffer.size() > dataBufferSize)
        {
            dataBuffer.erase(dataBuffer.begin());
        }                
        //// EOF STUDENT ASSIGNMENT

        //For logging
        if( bConsoleLogging ){
            cout<< "Databuffer Size  : "<< dataBuffer.size() << endl;
            cout << "        2        : DETECT IMAGE KEYPOINTS" << endl;
        }
        

        /* DETECT IMAGE KEYPOINTS */
        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based 
        ////  selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
        if( bConsoleLogging ){
            std::cout<< "Detector Type    : " << uniqueDetector << endl;
        }

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        //To accumulate the detection time taken                
        detectorType = uniqueDetector;

        //Detect keypoint function for allthe keypoint detector 
        //Note: Replaced the switch with a function to clean up the code
        detectorTime = detectKeypoints(keypoints, imgGray, iDetectorIndex, bVis, bConsoleLogging);

        //Since we remove the keypints later, which should be also acounted for total time
        keypointDetected = keypoints.size();
        //Calculating the total keypoints
        keypointTotal += keypointDetected;
        
        //Summing the detector time
        totalDetectorTime += detectorTime;
        

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
            //NOTE: Removed the condition for only SHITOMASI, since this is common 
            //      for all methods if we want to retain only 50 samples                    
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
            std::cout<< "Discriptor Type  : " << uniqueDescriptor << endl;       
        }
        //Descriptor time for accumulating the time taken by the descriptor extraction
        descriptorTime = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType, bConsoleLogging);                              
        
        //Summing the detector time
        totalDescriptorTime += descriptorTime;

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
            //string matcherType      = "MAT_BF";         // MAT_BF, MAT_FLANN //For SIFT MAT_FLANN
            
            
            //As discribed erlier, for SIFT based keypoints, L2 Norm and and FANN based methods used
            //https://answers.opencv.org/question/10046/feature-2d-feature-matching-fails-with-assert-statcpp/
            string matcherType      = CompatibleMatcherTypes((Descriptors)iDescriptorIndex);

            string descriptorType   = "DES_BINARY";     // DES_BINARY, DES_HOG  
            string selectorType     = "SEL_KNN";         // SEL_NN, SEL_KNN
            if( bConsoleLogging ){
                std::cout<< "Macher Type      : " << matcherType << endl;
                std::cout<< "Selector Type    : " << selectorType << endl;
            }

            
            //// STUDENT ASSIGNMENT//////////////////////////////////////////////////////////////////////////////////////////////////////////////////TODO, Uncomment Matcher
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
            matcherTime = matchDescriptors( (dataBuffer.end() - 2)->keypoints,  (dataBuffer.end() - 1)->keypoints,
                                            (dataBuffer.end() - 2)->descriptors,(dataBuffer.end() - 1)->descriptors,
                                            matches, descriptorType, matcherType, selectorType);
            //Total maches
            matchesTotal += matches.size();

            //Summing the matcher time
            totalMatcherTime += matcherTime;


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

    } // eof loop over all images

    //Total Pipeline time
    totalPipelineTime = totalDetectorTime + totalDescriptorTime + totalMatcherTime;
    if( bConsoleLogging )
    {
        cout << "Total time       : Detector + Descriptor + Matcher is " << totalPipelineTime << " ms \n";
    }
    
    std::cout<<"\nDetector Type    : " << uniqueDetector 
        << "\nDiscriptor Type  : " << uniqueDescriptor <<"\nDone."<< endl;   
    //For index of the combination run
    iCombinationIndex++;

    return 0;

}

