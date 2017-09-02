# GUI App for benchmarking Image detectors and descriptors


Computer Vision-based application for robotics rely on extracting meaningful features from images which can be used to perform many tasks like place recognition, place categorization, tracking, Visual Odometry, SLAM, object detection, object recognition, etc. All these tasks rely on a set of features/key-points and descriptors that describe them. Each of the features/key-points are associated with some sort of key signatures (called descriptors of those key points). Many existing methods for detection/description of these features exists, each descriptor has certain properties that are suitable for certain vision applications. Some of the existing methods include SIFT, SURF, BRISK, FAST, ORB,AKAZE, LATCH, LSD/BLD (lines), and many more. Some of the detectors already provide descriptors but some of them do not, so different combinations of descriptors can be used. Most of these approaches are already implemented in OpenCV. In this project a GUI application is built which provides a benchmark for these descriptors/key points for certain vision tasks as described below.

To run the code follow the instructions:

```
  $ cd mrpt_home
  $ mkdir build
  $ cd build
  $ cmake ..
  $ make 
  $ cd bin
  $ ./benchmarkingImageFeatures_GUI

```

Current Version has the following functionality:

**Basic Detector / Descriptor Functionality**

1. Detectors can be chosen to detect keypoints: Detectors currently included are: KLT, Harris Corner, SIFT, SURF, FAST, FASTER9, FASTER10, FASTER12, ORB detectors, AKAZE and LSD Detectors. Parameters specific to each detector can be tuned to observe the result.
2. Descriptors can be computed viualized around each key-point by clicking on any detector key-point. Descriptors currently included are: SIFT, SURF, Intensity Domain Spin Image, Polar Image, Log Polar Image, ORB, BLD and LATCH descriptors. Parameters specific to each descriptor can be tuned to observe the result.
3. Robust Stereo Matching: user can click on a detected key-point in the image1 (left image) and the best matching key-point is shown in image2 (right image). A plot showing descriptor distances of selected key-point in image1 to all other descriptors in image 2 is also shown. Descriptors from both the images are also shown for visualization purposes.
3. Different Input Types: User can choose the following input types in the app (single image, stereo image, MRPT rawlog datasets, single image dataset and stereo image dataset)
4. Forward and Backward Step-by-Step play of images: the app supports loading of single and stereo image datasets. User has an option to iterate over the dataset and apply various detectors and descriptors to any given image and tuning the detector/descriptor parameters as required. This allows user to inspect the evolution of performance.
5. Image Decimation: user can decimate a given image by a set factor to enlarge or shrink the chosen image
6. Repeatability: This provides the option to the user to evaluate detector performance. It returns the depeatability of detectors in the given image sequence based on detecting the key-points with a small window of +/- 5 pixels.
7. Homography Based Repeatability: This allows the user to provide the path to homography files and then checks if key-points are detected at the same points in subsequent images using the homographies and finally reports the homography
8. Detector Evaluation Metrics: Detectors are evaluated in terms computational cost, dispersion of the image, number of detected key-points, etc.
9. Descriptor Evaluation Metrics: Descriptors are evaluated in terms of percentage of successful matches for stereo images, descriptor distance between close matches, number of false positives / negatives, computational cost for computing the descriptors.

**Benchmarking against some standard vision tasks**

The developed GUI app is benchmarked with 3 vision tasks
- Visual Odometry
- Key-Point Tracking
- Place Recognition

**Visual Odometry:** The ego motion of the camera can be estimated using the GUI app. User can tune the detector parameters and observe how the performance of the VO changes. The user needs to provide the Ground Truth file, the camera calibration file and the input single image dataset file. This has been tested against the KITTI Autonomous Driving dataset. The app requires the user to enter the following information in the app:
- single image path: provide the path of the grayscale / color images of the KITTI dataset which can be downloaded from: http://www.cvlibs.net/datasets/kitti/eval_odometry.php . 
- calibration files: download the calibration files from: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
- ground truth poses: download the ground truth poses from: http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

**Key-Point Tracking:** The app supports tracking of key-points from monocular image datasets. This feature is activated when the user selects the Single Image input option. The user can provide the tracker parameters that he wants. User can choose the following parameters: maximum number of features, patch size, window width, window height, remove lost features, add new features, etc. User can change these parameters or simply leave them as is to their default values.

**Place Recognition:** The app supports Place Recognition. Place Recognition is the ability for a robot/machine to recognize a previously observed seen place. The user can select the detector and descriptor to extract features from the training image dataset. Each extracted descriptor from a single image is labelled with the lavel of the place it belongs to. This is done for every image in the dataset, so we have a vocabulary of training descriptors with each descriptor associated with a place label. So at test time for a given query image, descriptors are extracted and each descriptor is matched with the best match in the training dataset based on a Sum of Absolute Difference (SAD). The maximum number of votes for the place among all descriptors from the image is the predicted place. User has the option to provide detector/descriptor parameters too based on which the performace changes. So the user can see what combination (detector/descriptor) works best. Currently the app supports running the place recognition task for the KTH IDOL Dataset publically available at: http://www.cas.kth.se/IDOL/  
One can extend the `place_recognition.h` and `place_recognition.cpp` files to extend this to any other dataset too. 

**Sample Run**
- A sample run of the app requires the user to choose the detector, descriptor and enter the required parameters for the selected options.
- Click on evaluate Detector, then Click on evaluate detector and finally click on visualize Descriptor
- After this the user can select a specific Key-Point in image 1 and visualize the descriptor around it for a single image or find the best matching key-point in case of a stereo image.

**Step-by-Step Instructions to use the app**
More details about this are available at the tutorial file `tutorial.md` . This file shows clear step-by-step instructions to perform a sample run for performing different vision tasks that the user might want to do.

**Sample Video showing the app Usage**
A sample video run of the GUI app is available here which shows the exact usage of the app for different tasks.

=======
This project is available in the mrpt-2.0-devel branch for development

