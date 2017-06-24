# GUI App for benchmarking Image detectors and descriptors

<<<<<<< HEAD
This application is a GUI app to benchmark image detectors and descriptors. The performance of the descriptors and detectors will be evaluated in terms of dispersion, computational cost, number of found features, etc.

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

Current Version has the following:

1. Existing features from CFeatureExtraction class implemented
2. Existing descriptors from CFeatureExtraction class implemented with further modifications requried like visualizing the descriptors
3. Activate/Deactivate Non-maximal suppressions for some detectors
4. Forward and backward step-by-step play of images for application of detector / descriptor to certain images, to allow user to inpect the evolution of performance
5. Option for the user to provide input as single image, stereo image or datasets

To be done in the immediate future by June 21:
1. Add code for visulaizing descriptors
2. Ativate / Deactivate Robust Stereo Matching using metrics like correlation, SAD, etc.
3. Add Features like LSD/BLD detector/descriptor, AKAZE Detector,  ORB and LATCH Descriptor by creating wrappers around opencv code using CFeatureExtraction class

=======
This project is available in the mrpt-2.0-devel branch for development
>>>>>>> 6623600ae98bf3249c5e1eabfe8f4d531359af30
