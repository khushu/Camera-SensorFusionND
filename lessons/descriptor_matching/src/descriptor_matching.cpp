#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include "structIO.hpp"

using namespace std;
    /**
    // 1. Load the 'BRISK_small' dataset with cross-check first turned off, then on. 
    //    Look at the visualized keypoint matches and at the number of matched pairs and describe your results.

    // 2. Add the k-nearest-neighbor matching (using cv::knnMatch) with k=2 and 
    //    implement the above-described descriptor distance ratio to filter out ambiguous matches with the threshold set to 0.8. 
    //    Visualize the results, count the percentage of discarded matches (for both the 'BRISK_small' and the 'BRISK_large' dataset) 
    //    and describe your observations.    

    // 3. Use both BF matching and FLANN matching on the 'BRISK_large' dataset and on the SIFT dataset and describe your observations.
    **/
void matchDescriptors(cv::Mat &imgSource, cv::Mat &imgRef, vector<cv::KeyPoint> &kPtsSource, vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      vector<cv::DMatch> &matches, string descriptorType, string matcherType, string selectorType)
{

    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    //Brute Force mattcher
    if (matcherType.compare("MAT_BF") == 0)
    {

        int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
        std::cout << "Observations 1   : BF matching cross-check=" << crossCheck << endl;
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F)
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }

        int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        //TODO: What to do with Cross Check, and how are we using descriptorType, we are not passing that parameter to any function?
    }

    //std::cout <<"After Macher Here"<<endl; 
    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        double t = (double)cv::getTickCount();
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        std::cout << "Observations 2   : NN with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms @ " << (1000 * t / 1.0)/(double)matches.size() << " ms per match"<<endl;
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
        double t = (double)cv::getTickCount();
        // TODO : implement k-nearest-neighbor matching with k = 2
        std::vector< std::vector<cv::DMatch> > knn_matches;
        matcher->knnMatch( descSource, descRef, knn_matches, 2 );

        // TODO : filter matches using descriptor distance ratio test

        //-- Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.8f;        
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                matches.push_back(knn_matches[i][0]);
            }
        }
        //TODO: Count the percentatge of discarded maches of 
        //    Visualize the results, count the percentage of discarded matches (for both the 'BRISK_small' and the 'BRISK_large' dataset) 
        //    and describe your observations.
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        double dDiscarddedMatches = 0;
        if(knn_matches.size()!=0){
            dDiscarddedMatches = 100 * (1 - (matches.size()/ (double)knn_matches.size()));
        }        
        std::cout << "Observations 1   : KNN with k = 2, total matches: " << knn_matches.size() << " good matches:" << matches.size()<< endl;
        std::cout << "Observations 2   : Discard percentage of: " << dDiscarddedMatches <<" in " << 1000 * t / 1.0 << " ms @ " << (1000 * t / 1.0)/(double)matches.size() << " ms per match" << endl;
    }

    //std::cout <<"After NN"<<endl;
    // visualize results
    cv::Mat matchImg = imgRef.clone();
    cv::drawMatches(imgSource, kPtsSource, imgRef, kPtsRef, matches,
                    matchImg, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), 
                    //cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    string windowName = "Matching keypoints between two camera images";
    cv::namedWindow(windowName, 7);
    cv::imshow(windowName, matchImg);
    cv::waitKey(0);
}

int main()
{
    cv::Mat imgSource = cv::imread("../images/img1gray.png");
    cv::Mat imgRef = cv::imread("../images/img2gray.png");
    //string KeyPntType = "SIFT";
    //string KeyPntType = "BRISK_l";
    string KeyPntType = "BRISK_s";

    vector<cv::KeyPoint> kptsSource, kptsRef; 

    if(KeyPntType.compare("BRISK_l") == 0){
        readKeypoints("../dat/C35A5_KptsSource_BRISK_large.dat", kptsSource);
        readKeypoints("../dat/C35A5_KptsRef_BRISK_large.dat", kptsRef);
    }
    else if(KeyPntType.compare("BRISK_s") == 0){
        readKeypoints("../dat/C35A5_KptsSource_BRISK_small.dat", kptsSource);
        readKeypoints("../dat/C35A5_KptsRef_BRISK_small.dat", kptsRef);
    }
    else{
        readKeypoints("../dat/C35A5_KptsSource_SIFT.dat", kptsSource);
        readKeypoints("../dat/C35A5_KptsRef_SIFT.dat", kptsRef);
    }

    cv::Mat descSource, descRef; 

    if(KeyPntType.compare("BRISK_l") == 0){
        readDescriptors("../dat/C35A5_DescSource_BRISK_large.dat", descSource);
        readDescriptors("../dat/C35A5_DescRef_BRISK_large.dat", descRef);
    }
    else if(KeyPntType.compare("BRISK_s") == 0){
        readDescriptors("../dat/C35A5_DescSource_BRISK_small.dat", descSource);
        readDescriptors("../dat/C35A5_DescRef_BRISK_small.dat", descRef);
    }
    else{
        readDescriptors("../dat/C35A5_DescSource_SIFT.dat", descSource);
        readDescriptors("../dat/C35A5_DescRef_SIFT.dat", descRef);
    }



    //Brisk - BF Matcher


    vector<cv::DMatch> matches;
    //string matcherType = "MAT_BF";
    string matcherType = "MAT_FLANN";
    
    string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
    //string selectorType = "SEL_NN";
    string selectorType = "SEL_KNN";
    
    // Frame 1 : Source
    // Frame 2 : Ref
    std::cout<< "Key Point Dataset: " << KeyPntType<< endl;
    std::cout<< "Macher Type      : " << matcherType << endl;
    std::cout<< "Discriptor Type  : " << descriptorType << endl;
    std::cout<< "Selector Type    : " << selectorType << endl;

    matchDescriptors(imgSource, imgRef, kptsSource, kptsRef, descSource, descRef, matches, descriptorType, matcherType, selectorType);
}
