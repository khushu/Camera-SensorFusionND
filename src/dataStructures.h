#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>



struct DataFrame {                      // represents the available sensor information at the same time instance    
    cv::Mat cameraImg;                  // camera image    
    std::vector<cv::KeyPoint> keypoints;// 2D keypoints within camera image
    cv::Mat descriptors;                // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
};

//Make enums of all the option, increment enum variable, put conditions 
// enum Detectors{
//     detector_SHITOMASI = 1,
//     detector_HARRIS,
//     detector_FAST,
//     detector_BRISK,
//     detector_ORB,
//     detector_AKAZE,
//     detector_SIFT,
// };
// enum Descriptors{
//     descriptor_BRISK = 1,
//     descriptor_BRIEF,
//     descriptor_ORB,
//     descriptor_FREAK,
//     descriptor_AKAZE,
//     descriptor_SIFT,
// };
// enum DescriptorsType{
//     descriptorType_DES_BINARY = 1,
//     descriptorType_DES_HOG
// };
// enum Matchers{
//     matcher_MAT_BF = 1,
//     matcher_MAT_FLANN
// };

// enum Selectors{
//     selector_SEL_NN = 1,
//     selector_SEL_KNN
// };


//Learned new conceppts:
//1. Macros to define enums clases and string equivalent in c & c++, using # and ## operators
//https://stackoverflow.com/questions/147267/easy-way-to-use-variables-of-enum-types-as-string-in-c
//https://stackoverflow.com/questions/15424218/c-class-factory-macro


//2. Macro expanders:
//https://stackoverflow.com/questions/3019609/any-utility-to-test-expand-c-c-define-macros
//https://stackoverflow.com/questions/5900419/c-macro-expander

//3. Finally added inline to remove multiple definition ld error due to header file functions definitions
//https://stackoverflow.com/questions/41597744/why-am-i-getting-a-multiple-definition-error-how-do-i-fix-it


// expansion macro for enum value definition
#define ENUM_VALUE(name,assign) name assign,

// expansion macro for enum to string conversion
#define ENUM_CASE(name,assign) case name: return #name;

// expansion macro for string to enum conversion
#define ENUM_STRCMP(name,assign) if (!strcmp(str,#name)) return name;


// declare the access function and define enum values
#define DECLARE_ENUM(EnumType,ENUM_DEF) \
  enum EnumType { \
    ENUM_DEF(ENUM_VALUE) \
  }; \
  const char *GetString(EnumType dummy); \
  EnumType Get##EnumType##Value(const char *string); \

// define the access function names
#define DEFINE_ENUM(EnumType,ENUM_DEF) \
  inline const char *GetString(EnumType value) \
  { \
    switch(value) \
    { \
      ENUM_DEF(ENUM_CASE) \
      default: return "Error, not implemented, please check"; /* handle input error */ \
    } \
  } \
  inline EnumType Get##EnumType##Value(const char *str) \
  { \
    ENUM_DEF(ENUM_STRCMP) \
    return (EnumType)0; /* handle input error */ \
  } \


//Declare and Define the detector enum
#define DETECTOR_ENUM(det) \
    det(detector_SHITOMASI,= 1) \
    det(detector_HARRIS,) \
    det(detector_FAST,) \
    det(detector_BRISK,) \
    det(detector_ORB,) \
    det(detector_AKAZE,) \
    det(detector_SIFT,) \

DECLARE_ENUM(Detectors,DETECTOR_ENUM);
DEFINE_ENUM(Detectors,DETECTOR_ENUM);

//Declare and Define the descriptor enum
#define DESCRIPTOR_ENUM(des) \
    des(descriptor_BRISK,= 1) \
    des(descriptor_BRIEF,) \
    des(descriptor_ORB,) \
    des(descriptor_FREAK,) \
    des(descriptor_AKAZE,) \
    des(descriptor_SIFT,) \

DECLARE_ENUM(Descriptors,DESCRIPTOR_ENUM);
DEFINE_ENUM(Descriptors,DESCRIPTOR_ENUM);

//Declare and Define the descriptor type enum
#define DESCRIPTORTYPE_ENUM(desType) \
    desType(descriptorType_DES_BINARY,= 1) \
    desType(descriptorType_HOG,) 

DECLARE_ENUM(DescriptorsType,DESCRIPTORTYPE_ENUM);
DEFINE_ENUM(DescriptorsType,DESCRIPTORTYPE_ENUM);


//Declare and Define the matcher enum
#define MATCHER_ENUM(matcher) \
    matcher(matcher_MAT_BF,= 1) \
    matcher(matcher_MAT_FLANN,) 

DECLARE_ENUM(Matchers,MATCHER_ENUM);
DEFINE_ENUM(Matchers,MATCHER_ENUM);


//Declare and Define the selector enum
#define SELECTOR_ENUM(selector) \
    selector(selector_SEL_NN,= 1) \
    selector(selector_SEL_KNN,) 

DECLARE_ENUM(Selectors,SELECTOR_ENUM);
DEFINE_ENUM(Selectors,SELECTOR_ENUM);

inline std::string get_right_of_delim(std::string const& str, std::string const& delim)
{
  return str.substr(str.find(delim) + delim.size());
}

//Word length for logfiles
#define SWL_SERIAL_NO     7
#define SWL_DETECTOR      13
#define SWL_DESCRIPTOR    14
#define SWL_TL_KEYPOINTS  18
#define SWL_TIME_P_KEYP   21
#define SWL_TL_MATCHES    16
#define SWL_TIME_P_MATCH  21
#define SWL_TL_PPLIN_TME  40

#endif /* dataStructures_h */
