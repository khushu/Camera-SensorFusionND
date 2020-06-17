#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>


struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
};

//Make enums of all the option, increment enum variable, put conditions 
//Also, for each of the combination play with parameters to improve the results 
enum Detectors{
    detector_SHITOMASI = 1,
    detector_HARRIS,
    detector_FAST,
    detector_BRISK,
    detector_ORB,
    detector_AKAZE,
    detector_SIFT,
};

enum Descriptors{
    descriptor_BRISK = 1,
    descriptor_BRIEF,
    descriptor_ORB,
    descriptor_FREAK,
    descriptor_AKAZE,
    descriptor_SIFT,
};

enum DescriptorsType{
    descriptorType_DES_BINARY = 1,
    descriptorType_DES_HOG
};

enum Matchers{
    matcher_MAT_BF = 1,
    matcher_MAT_FLANN
};

enum Selectors{
    selector_SEL_NN = 1,
    selector_SEL_KNN
};


// inline std::vector<DescriptorType> CompatibleDescriptorTypes(const Descriptor descriptor)
// {

//     switch(r)
//     {
//         case Descriptors::descriptor_BRISK  : std::cout << "Brinsk\n";   break;
//         case Descriptors::green: std::cout << "green\n"; break;
//         case Descriptors::blue : std::cout << "blue\n";  break;
//     }
//   switch (descriptor)
//   {
//     case descriptor_BRISK:
//     case descriptor_BRIEF:
//     case descriptor_ORB:
//     case descriptor_FREAK:
//     case descriptor_AKAZE:
//       return { descriptor_type_DES_BINARY, descriptor_type_DES_HOG, };
//     case descriptor_SIFT:
//       return { descriptor_type_DES_HOG, };
//     default:
//       throw std::logic_error("some descriptors are not presented in the list of 'case' statements");
//   }
// }


// template <typename T>
// inline const char* ToString(const char* const names[], T index)
// {
//   return names[static_cast<size_t>(index)];
// }

// inline std::string ToString(Detector det)
// {
//   return ToString(detector_names, det);
// }

// inline std::string ToString(Descriptor desc)
// {
//   return ToString(descriptor_names, desc);
// }

// inline std::string ToString(DescriptorType desc_type)
// {
//   return ToString(descriptor_type_names, desc_type);
// }

// inline std::string ToString(Matcher mtch)
// {
//   return ToString(matcher_names, mtch);
// }

// inline std::string ToString(Selector sel)
// {
//   return ToString(selector_names, sel);
// }


#endif /* dataStructures_h */
