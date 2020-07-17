# 2D Feature detection, extraction and matching

Mid-term project for Camera module towards Udacity's Sensor Fusion nanao degree. 

<img src="images/keypoints.png" width="820" height="248" />

Mid-term project is work towards the final project to design a real-time collision avoidance system with data from multiple sensors (Radar, Lidar, Camera) on a vehicle. 

The aim of this project is to make a pipeline to compare different  detector / descriptor methods implemented in OpenCV, and recommend the best combination, in terms of speed of execution and number of matches. 

We would use the best recommended combination, since there are stringent real-time requirements for calculating TTC (time-to-collision) within a specified time from a monocular camera feed used in Collision avoidance system.

For the pipeline, there are 10 consecutive test images, pre-captured from a camera mounted on the top of the Subject Vehicle (SV). The project is supposed to detect and match keypoints in consecutive frames, at a defined ROI directly in front of the the SV and generate total number of matched points with timing information to analyze the performance of detector / descriptor pairs. 


**Observations and parts of the project:**

The flow of the module has been modified, so that the code is a bit clean, with 2 separate flows for task 1-6 with visuals to show the integration of the pipeline and 2nd generating the file for comparison and analysis of different  detector / descriptor of pairs. Also, because for generation of files, more expensive functions are called like file operations and sort, which hinder the speed and can not have visual for testing.

One flow runs all the detector / descriptor pair, and other runs one the best with visuals.

**MP.0 Mid-Term Report**

Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.

Created a README.md

**Done.**

**MP.1 Data Buffer Optimization**

Implement a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements). This can be achieved by pushing in new elements on one end and removing elements on the other end. 

Using erase in a vector method to delete the first data element in the vactor to implement circular buffer of images. Works as per specification, No known issues.
 
```
//Implemented a Ring buffer on vector, erase the first element in the vector, which is the oldest 
if (dataBuffer.size() > dataBufferSize)
{
    dataBuffer.erase(dataBuffer.begin());
}
```

  
**Done.**

**MP.2 Keypoint Detection**

Implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly.

All the detectors mentioned are working.

**Done.**

**MP.3 Keypoint Removal**

Remove all keypoints outside of a predefined rectangle and only use the keypoints within the rectangle for further processing. 

All the keypoints outside the box are removed using contains method. 

```
//check if the keypoint is not within the defined box    
if(!vehicleRect.contains((*i).pt)) 
{
    //Erase the keypoint and decrement the iterator
    keypoints.erase(i--);
}
```

**Done.**

**MP.4 Keypoint Descriptors**

Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.

All the descriptors mentioned are working.

**Done**

**MP.5 Descriptor Matching**

Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function. 

Both FLANN matching and k-nearest neighbor are working, which can be selected by a string. 

**Done.**

**MP.6 Descriptor Distance Ratio**

Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.

The descriptor distance ratio test to filter keypoints based on the distance ratio threshold set to 0.8.

**Done.**

**MP.7 Performance Evaluation 1**

Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented. 

A directory created inside results/keypoints, for all the keypoints, in which each keypoint extraction is a seperate file with number of keypoints detected, time taken per keypoint and distribution of neighborhood sizes for each keypoint extraction. Distribution has mean, median, and standard deviation information. 

The neighborhood size standard deviations are minimum as the following in ascending order (listed only smallest numbers in each keypoint): 
```
SHITOMASI 	- 0
HARIS 		- 0
FAST		- 0
AKAZE		- 3.37
SIFT 		- 5.14
BRISK 		- 12.61
ORB 		- 23.67
```

**Done.**


**MP.8 Performance Evaluation 2**

Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.

A directory created inside results/matches, for all the matches, in which each of the detector / descriptor pair is a seperate file with number of matches detected per frame with respect to detected keypoints, time taken per match. 

Also, implemented descriptor distance ratio test with ratio threshold set to 0.8.

**Done.**


**MP.9 Performance Evaluation 3**

Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.


In results directory, 2 files gets created "FinalResults" which has same content, one .txt and one .csv. This is the final analysis based on the timing for all 10 images per detector / descriptor pair and the summary table is shown below. The conclusion and argument follows below:

**Done.**

**Final result table**

The executable if run in generating files mode (./2D_feature_tracking 1), generates a results directory, in parallel to build directory. Inside results directory, creates 2 more directories containing keypoints and matches files. Also, in results directory it creates FinalResults.csv/FinalResults.txt which has same content. Summary is shown below: 

Result table for all the detector / descriptor pair methods, then sorting based on the speed and then number of matches to finally conclude.
```
Sl No |  Detector | Descriptor | T Keypnts | Tme P Keypnt | T Matches |   Tme P Mtch | Ttl tme 10 frms (s) |
 ---- |      ---- |       ---- |      ---- |         ---- |      ---- |         ---- |                ---- |
    1 | SHITOMASI |      BRISK |     13423 |  1.07751e-05 |       767 |  7.32659e-06 |            0.170768 |
    2 | SHITOMASI |      BRIEF |     13423 |   1.0268e-05 |       944 |  2.01467e-06 |            0.153785 |
    3 | SHITOMASI |        ORB |     13423 |  1.14131e-05 |       908 |  2.47325e-06 |            0.165662 |
    4 | SHITOMASI |      FREAK |     13423 |  8.52419e-06 |       768 |  2.63931e-06 |            0.478168 |
    5 | SHITOMASI |       SIFT |     13423 |  8.20131e-06 |       926 |  2.25701e-05 |            0.283115 |
    6 |    HARRIS |      BRISK |      4239 |  2.29635e-05 |       478 |  3.05518e-06 |            0.108909 |
    7 |    HARRIS |      BRIEF |      4239 |  2.25123e-05 |       477 |  2.85883e-06 |            0.101013 |
    8 |    HARRIS |        ORB |      4239 |  2.33935e-05 |       467 |  2.82105e-06 |            0.108283 |
    9 |    HARRIS |      FREAK |      4239 |  2.28981e-05 |       466 |  3.46627e-06 |            0.425162 |
   10 |    HARRIS |       SIFT |      4239 |  2.44996e-05 |       464 |  3.39565e-05 |            0.266408 |
   11 |      FAST |      BRISK |     49204 |  3.98207e-07 |      2183 |  7.04869e-06 |           0.0737474 |
   12 |      FAST |      BRIEF |     49204 |  4.17384e-07 |      2831 |  5.62399e-06 |           0.0511946 |
   13 |      FAST |        ORB |     49204 |  4.35442e-07 |      2768 |  5.68211e-06 |           0.0524653 |
   14 |      FAST |      FREAK |     49204 |  4.20041e-07 |      2233 |  7.68106e-06 |            0.397394 |
   15 |      FAST |       SIFT |     49204 |  4.38625e-07 |      2789 |  2.41865e-05 |            0.416115 |
   16 |     BRISK |      BRISK |     27116 |  1.37929e-05 |      1570 |  4.81568e-06 |            0.412218 |
   17 |     BRISK |      BRIEF |     27116 |  1.34101e-05 |      1704 |  4.39922e-06 |            0.381617 |
   18 |     BRISK |        ORB |     27116 |  1.42067e-05 |      1514 |  6.25166e-06 |            0.446313 |
   19 |     BRISK |      FREAK |     27116 |  1.42039e-05 |      1524 |   4.2049e-06 |            0.760146 |
   20 |     BRISK |       SIFT |     27116 |  1.34417e-05 |      1659 |  2.61659e-05 |            0.818746 |
   21 |       ORB |      BRISK |      5000 |  1.56921e-05 |       751 |  2.31141e-06 |           0.0936804 |
   22 |       ORB |      BRIEF |      5000 |   1.3195e-05 |       545 |  3.34565e-06 |           0.0738613 |
   23 |       ORB |        ORB |      5000 |  1.26552e-05 |       763 |  2.36857e-06 |             0.11041 |
   24 |       ORB |      FREAK |      5000 |  1.25843e-05 |       420 |  2.31427e-06 |            0.387516 |
   25 |       ORB |       SIFT |      5000 |  1.57515e-05 |       765 |  2.35328e-05 |            0.625177 |
   26 |     AKAZE |      BRISK |     13429 |  4.89336e-05 |      1215 |  2.53702e-06 |            0.677395 |
   27 |     AKAZE |      BRIEF |     13429 |  5.04557e-05 |      1266 |  2.87044e-06 |            0.688759 |
   28 |     AKAZE |        ORB |     13429 |  4.62386e-05 |      1182 |  3.02764e-06 |            0.662697 |
   29 |     AKAZE |      FREAK |     13429 |  5.06667e-05 |      1187 |  2.66212e-06 |             1.02358 |
   30 |     AKAZE |      AKAZE |     13429 |  4.73513e-05 |      1259 |  2.76358e-06 |             1.14582 |
   31 |     AKAZE |       SIFT |     13429 |  5.84856e-05 |      1277 |  2.27221e-05 |             1.09282 |
   32 |      SIFT |      BRISK |     13862 |  8.91143e-05 |       592 |  4.78064e-06 |             1.25731 |
   33 |      SIFT |      BRIEF |     13862 |  9.79274e-05 |       702 |  4.17575e-06 |             1.37049 |
   34 |      SIFT |      FREAK |     13862 |  8.23701e-05 |       593 |  4.19811e-06 |             1.46483 |
   35 |      SIFT |       SIFT |     13862 |  6.64414e-05 |       803 |   3.0621e-05 |             1.72726 |
 ---- |      ---- |       ---- |      ---- |         ---- |      ---- |         ---- |                ---- |
```
**Top 5 based on the time (speed):**
```
Sl No | Rank | Detector | Descriptor | T Keypnts | Tme P Keypnt | Matches |  Tme P Mtch | tme 10 frms (s) |
 ---- | ---- |     ---- |       ---- |      ---- |         ---- |    ---- |        ---- |            ---- |
   12 |    1 |     FAST |      BRIEF |     49204 |  4.17384e-07 |    2831 | 5.62399e-06 |       0.0511946 |
   13 |    2 |     FAST |        ORB |     49204 |  4.35442e-07 |    2768 | 5.68211e-06 |       0.0524653 |
   11 |    3 |     FAST |      BRISK |     49204 |  3.98207e-07 |    2183 | 7.04869e-06 |       0.0737474 |
   22 |    4 |      ORB |      BRIEF |      5000 |   1.3195e-05 |     545 | 3.34565e-06 |       0.0738613 |
   21 |    5 |      ORB |      BRISK |      5000 |  1.56921e-05 |     751 | 2.31141e-06 |       0.0936804 |
 ---- | ---- |     ---- |       ---- |      ---- |         ---- |    ---- |        ---- |            ---- |
```
**Top 5 based on the matches**
```
Sl No | Rank | Detector | Descriptor | T Keypnts | Tme P Keypnt | Matches |  Tme P Mtch | tme 10 frms (s) |
 ---- | ---- |     ---- |       ---- |      ---- |         ---- |    ---- |        ---- |            ---- |
   12 |    1 |     FAST |      BRIEF |     49204 |  4.17384e-07 |    2831 | 5.62399e-06 |       0.0511946 |
   15 |    2 |     FAST |       SIFT |     49204 |  4.38625e-07 |    2789 | 2.41865e-05 |        0.416115 |
   13 |    3 |     FAST |        ORB |     49204 |  4.35442e-07 |    2768 | 5.68211e-06 |       0.0524653 |
   14 |    4 |     FAST |      FREAK |     49204 |  4.20041e-07 |    2233 | 7.68106e-06 |        0.397394 |
   11 |    5 |     FAST |      BRISK |     49204 |  3.98207e-07 |    2183 | 7.04869e-06 |       0.0737474 |
 ---- | ---- |     ---- |       ---- |      ---- |         ---- |    ---- |        ---- |            ---- |
```
**Final recommendation:**

In both Speed and number of matches, row with Sl No 12, comes 1st and outperforms other detector / descriptor methods.
```
Detector	: FAST
Descriptor	: BRIEF
```

**Machine configuration for benchmark timing:**
 1. HP Spectre 360
 2. Ubuntu 18.04.4 LTS (64 bits)
 3. Intel® Core™ i7-7500U CPU @ 2.70GHz × 4
 4. Intel® HD Graphics 620 (Kaby Lake GT2)
 5. Memory: 15.5 GB


## Dependencies for Running Locally
* git
  * install git on your local machine [click here for installation instructions](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) 
  * checkout/clone the repository "2D Feature detection, extraction and matching"
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

**Through bash files**
```
on Linux/mac (TODO: not yet tested on windows)
Files:
CLEAN: this files deletes old "results" and "build" directories
BUILD: this creates a build directory and compiles the code and makes binaries/executable
RUN: runs the executable generating the files
RUN_VIS: runs the executable in visual mode with one of the methods
```
1. Clone this repo.
2. run: chmod +x CLEAN BUILD RUN RUN_VIS
3. run: ./CLEAN
4. run: ./BUILD
5. run: ./RUN or ./RUN_VIS

OR

2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking 0` or `./2D_feature_tracking 1`

 * `./2D_feature_tracking 0` - Runs with only visual and for only 10 frames 
 * `./2D_feature_tracking 1` - Runs with generating the files for all the detector / descriptor combinations
 * `./2D_feature_tracking`   - will take the default parameter of 1 and generate the files


