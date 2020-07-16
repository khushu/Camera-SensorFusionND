#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
double matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;
 
    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = cv::NORM_HAMMING;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F)
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }

        //Norm type not used??
        int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        // Implemented FLANN matching
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);        

    }

    //Timing for matching keypoints
    double t = (double)cv::getTickCount();
    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
        //cout<<"SEL_KNN"<<endl;

        std::vector< std::vector<cv::DMatch> > knn_matches;
        matcher->knnMatch( descSource, descRef, knn_matches, 2 );

        //-- Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.8f;        
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                matches.push_back(knn_matches[i][0]);
            }
        }
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

    return t;
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
double descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType, bool bConsoleLogging)
{
    // select appropriate descriptor //BRISK - Binary, BRIEF, ORB, FREAK, AKAZE, SIFT
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {
        //Caling with default parameters
        int threshold = 30;         // FAST/AGAST detection threshold score.
        int octaves = 3;            // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f;  // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if(descriptorType.compare("BRIEF") == 0)
    {   
        //Caling with default parameters
        int bytes=32;               // legth of the descriptor in bytes
        bool use_orientation=false; // sample patterns using key points orientation
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(bytes, use_orientation);  
    }
    else if(descriptorType.compare("ORB") == 0)
    {
        //Caling with default parameters
        int nfeatures=500;          // The maximum number of features to retain.    
        float scaleFactor=1.2f;     // Pyramid decimation ratio, greater than 1.
        int nlevels=8;              // The number of pyramid levels.
        int edgeThreshold=31;       // This is size of the border where the features are not detected.
        int firstLevel=0;           // The level of pyramid to put source image to.
        int WTA_K=2;                // The number of points that produce each element of the oriented BRIEF descriptor.
        cv::ORB::ScoreType scoreType=cv::ORB::HARRIS_SCORE; 
                                    // The default HARRIS_SCORE means that Harris algorithm is used to rank features 
                                    // (the score is written to KeyPoint::score and is used to retain best nfeatures features);
                                    // FAST_SCORE is alternative value of the parameter that produces slightly less stable keypoints, 
                                    // but it is a little faster to compute.  
        int patchSize=31;           // Size of the patch used by the oriented BRIEF descriptor.
        int fastThreshold=20;       // the fast threshold    

        extractor = cv::ORB::create(nfeatures,scaleFactor,nlevels,edgeThreshold,firstLevel,WTA_K,scoreType,patchSize,fastThreshold);
    }
    else if(descriptorType.compare("FREAK") == 0)
    {
        //Caling with default parameters
        bool orientationNormalized=true,    // Enable orientation normalization. 
                scaleNormalized=true;       // Enable scale normalization.
        float patternScale=22.0f;           // Scaling of the description pattern. 
        int nOctaves=4;                     // Number of octaves covered by the detected keypoints. 

        extractor = cv::xfeatures2d::FREAK::create(orientationNormalized, scaleNormalized, patternScale, nOctaves);
    }
    else if(descriptorType.compare("AKAZE") == 0)
    {
        //Caling with default parameters
        cv::AKAZE::DescriptorType descriptor_type=cv::AKAZE::DESCRIPTOR_MLDB; 
                                    //Type of the extracted descriptor: DESCRIPTOR_KAZE, 
                                    //DESCRIPTOR_KAZE_UPRIGHT, DESCRIPTOR_MLDB or DESCRIPTOR_MLDB_UPRIGHT.
        int descriptor_size=0,      //Size of the descriptor in bits. 0 -> Full size
        descriptor_channels=3;      //Number of channels in the descriptor (1, 2, 3) 
        float threshold=0.001f;     //Detector response threshold to accept point
        int nOctaves=4,             //Maximum octave evolution of the image 
        nOctaveLayers=4;            //Default number of sublevels per scale level 
        cv::KAZE::DiffusivityType diffusivity=cv::KAZE::DIFF_PM_G2; 
                                    //Diffusivity type. DIFF_PM_G1, DIFF_PM_G2, DIFF_WEICKERT or DIFF_CHARBONNIER

        extractor = cv::AKAZE::create(descriptor_type,descriptor_size,descriptor_channels,threshold,nOctaves,nOctaveLayers,diffusivity);
    }
    else if(descriptorType.compare("SIFT") == 0)
    {

        //Caling with default parameters
        int nfeatures=0,                //The number of best features to retain. 
                                        //The features are ranked by their scores (measured in SIFT algorithm 
                                        //as the local contrast)
        nOctaveLayers=3;                //The number of layers in each octave. 3 is the value used in D. 
                                        //Lowe paper. The number of octaves is computed automatically from 
                                        //the image resolution.
        double contrastThreshold=0.04,  //The contrast threshold used to filter out weak features in 
                                        //semi-uniform (low-contrast) regions.The larger the threshold, 
                                        //the less features are produced by the detector.
        edgeThreshold=10,               //The threshold used to filter out edge-like features. Note that 
                                        //the its meaning is different from the contrastThreshold, i.e. 
                                        //the larger the edgeThreshold, the less features are filtered out 
                                        //(more features are retained).
        sigma=1.6;                      //The sigma of the Gaussian applied to the input image at the octave #0. 
                                        //If your image is captured with a weak camera with soft lenses, 
                                        //you might want to reduce the number. 

        extractor = cv::xfeatures2d::SIFT::create(nfeatures,nOctaveLayers,contrastThreshold,edgeThreshold,sigma);
    }
    else{

        std::cout<< "Descriptor Error : Does not match implemented method, please check: "<<descriptorType<< endl;
        cv::waitKey(0); // wait for key to be pressed        
    }
    

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    if( bConsoleLogging ){
        cout << "Observations     : "<<descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
    }
    return t; 
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
double detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis, bool bConsoleLogging)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    if( bConsoleLogging ){
        cout << "Observations     : Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms @ " << (1000 * t / 1.0)/(double)keypoints.size() << " ms per keypoint" << endl;
    }
    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
    return t;
}
// Detect keypoints in image using the traditional HARRIS detector
double detKeypointsHarris(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis, bool bConsoleLogging)
{

    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    int apertureSize = 3;
    double k = 0.04;
    int thresh = 100;

    // Get time
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;

    cv::Mat HarrisImage,dst_norm, dst_norm_scaled;
    HarrisImage = cv::Mat::zeros( img.size(), CV_32FC1 );

    // Apply corner detection
    cv::cornerHarris(img, HarrisImage, blockSize,	apertureSize,k, cv::BORDER_DEFAULT);

    /// Normalizing
    cv::normalize( HarrisImage, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs( dst_norm, dst_norm_scaled );

    // collecting the detected cornners in the results of keypoints 
    for( int j = 0; j < dst_norm.rows ; j++ )
    { 
        for( int i = 0; i < dst_norm.cols; i++ )
        {
            if( (int) dst_norm.at<float>(j,i) > thresh )
            {
                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i,j);
                newKeyPoint.size = blockSize;
                keypoints.push_back(newKeyPoint);
            }
        }
    }

    //Difference of time to calculate the time elapsed
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    if( bConsoleLogging ){
        cout << "Observations     : Harris Cornner detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms @ " << (1000 * t / 1.0)/(double)keypoints.size() << " ms per keypoint" << endl;
    }
    //visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Haris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
    return t;
}

// Detect keypoints in image using the traditional FAST detector
double detKeypointsFAST(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis, bool bConsoleLogging){

    //Parameter innitialization
    int threshold = 10;
    bool nonmaxSuppression = true ;

    // Get the current time
    double t = (double)cv::getTickCount();

    // FastFeatureDetector::TYPE_9_16, 
    // FastFeatureDetector::TYPE_7_12,
    // FastFeatureDetector::TYPE_5_8
    // Apply detection
    cv::Ptr<cv::FeatureDetector> detector = cv::FastFeatureDetector::create(threshold,nonmaxSuppression,cv::FastFeatureDetector::TYPE_9_16);
    detector->detect(img, keypoints);

    //Wrapper method slightly slower
    //cv::FAST( img, keypoints, threshold, nonmaxSuppression);

    //Calculate the time difference 
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    if( bConsoleLogging ){
        cout << "Observations     : Fast Feature detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms @ " << (1000 * t / 1.0)/(double)keypoints.size() << " ms per keypoint" << endl;
    }
    //visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Fast Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
    return t;
}
// Detect keypoints in image using the traditional BRISK detector
double detKeypointsBRISK(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis, bool bConsoleLogging){
    
    //Prepare parameters:
    int thresh=30, octaves=3;
    float patternScale=1.0f;

    //BRISK detector 
    cv::Ptr<cv::FeatureDetector> detector = cv::BRISK::create(thresh,octaves,patternScale);
    
    // Get the current time
    double t = (double)cv::getTickCount();

    // Apply detection
    detector->detect(img, keypoints);

    //Calculate the time difference 
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    if( bConsoleLogging ){
        cout << "Observations     : BRISK with n = " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms @ " << (1000 * t / 1.0)/(double)keypoints.size() << " ms per keypoint" << endl;
    }
    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "BRISK feature detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
    return t;

}
// Detect keypoints in image using the traditional ORB detector
double detKeypointsORB(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis, bool bConsoleLogging){

    //Parameter preparation
    int nfeatures=500, nlevels=8, edgeThreshold=31, firstLevel=0, WTA_K=2, 
        scoreType=cv::ORB::HARRIS_SCORE, patchSize=31, fastThreshold=20;
    float scaleFactor=1.2f;

    //ORB detector 
    // cv::Ptr<cv::ORB> detector = cv::ORB::create(nfeatures, scaleFactor, nlevels, 
                                            // edgeThreshold, firstLevel, WTA_K, 
                                            // scoreType, patchSize, fastThreshold);
    
    //For less control
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    
    // Get the current time
    double t = (double)cv::getTickCount();

    // Apply detection
    detector->detect(img, keypoints);

    //Calculate the time difference 
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    if( bConsoleLogging ){
        cout << "Observations     : ORB with n = " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms @ " << (1000 * t / 1.0)/(double)keypoints.size() << " ms per keypoint" << endl;
    }

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "ORB Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
    return t;
}
// Detect keypoints in image using the traditional AKAZE detector
double detKeypointsAKAZE(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis, bool bConsoleLogging){

    //Parameter preparation
    int  descriptor_size = 0, descriptor_channels = 3, nOctaves = 4, nOctaveLayers = 4;
    float threshold = 0.001f;                               

    //AKAZE detector 
    // cv::Ptr<cv::AKAZE> detector = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, descriptor_size,
    //                                     descriptor_channels, threshold, nOctaves,
    //                                     nOctaveLayers, cv::KAZE::DIFF_PM_G2);

    
    //For less control
    cv::Ptr<cv::AKAZE> detector = cv::AKAZE::create();
    
    // Get the current time
    double t = (double)cv::getTickCount();

    // Apply detection
    detector->detect(img, keypoints);

    //Calculate the time difference 
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    if( bConsoleLogging ){
        cout << "Observations     : AKAZE with n = " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms @ " << (1000 * t / 1.0)/(double)keypoints.size() << " ms per keypoint" << endl;
    }

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "AKAZE Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
    return t;
}
// Detect keypoints in image using the traditional SIFT detector
double detKeypointsSIFT(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis, bool bConsoleLogging){

    //Create Sift detector 
    cv::Ptr<cv::FeatureDetector> detector = cv::xfeatures2d::SIFT::create();

    // Get the current time
    double t = (double)cv::getTickCount();

    //Detect the keypoints
    detector->detect(img, keypoints);

    //Calculate the time difference 
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    if( bConsoleLogging ){
        cout << "Observations     : SIFT detector with n = " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms @ " << (1000 * t / 1.0)/(double)keypoints.size() << " ms per keypoint" << endl;
    }

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "SIFT feature detector  Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
    return t;
}

// Detect keypoints in image using the traditional Modern keypoint
double detectKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, int iDetectorIndex, bool bVis, bool bConsoleLogging){

    // Get the current time
    double t = 0;

    //Switch based on the detector type
    switch (iDetectorIndex)
    {
    case detector_SHITOMASI:
        // Detect keypoints in image using the traditional SHITOMASI detector
        t = detKeypointsShiTomasi(keypoints, img, bVis, bConsoleLogging);
        break;
    case detector_HARRIS:
        // Detect keypoints in image using the traditional HARRIS detector
        t = detKeypointsHarris(keypoints, img, bVis, bConsoleLogging);
        break;
    case detector_FAST:
        // Detect keypoints in image using the FAST detector
        t = detKeypointsFAST(keypoints, img, bVis, bConsoleLogging);
        break;
    case detector_BRISK:
        // Detect keypoints in image using the BRISK detector
        t = detKeypointsBRISK(keypoints, img, bVis, bConsoleLogging);
        break;
    case detector_ORB:
        // Detect keypoints in image using the ORB detector
        t = detKeypointsORB(keypoints, img, bVis, bConsoleLogging);
        break;
    case detector_AKAZE:
        // Detect keypoints in image using the AKAZE detector
        t = detKeypointsAKAZE(keypoints, img, bVis, bConsoleLogging);
        break;
    case detector_SIFT:
        // Detect keypoints in image using the SIFT detector
        t = detKeypointsSIFT(keypoints, img, bVis, bConsoleLogging);
        break;                            
    default:
        std::cout<< "Dector Error     : Does not match implemented method, please check: "<<GetString((Detectors)iDetectorIndex)<< endl;
        cv::waitKey(0); // wait for key to be pressed
        return 1;                
    }

    return t;
}

