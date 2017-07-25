# GUI App for benchmarking Image detectors and descriptors

 HEAD
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

1. Existing detectors/descriptors from CFeatureExtraction class implemented with further modifications requried like visualizing the detectors / descriptors. Detectors currently supported include : KLT, Harris Corner, SIFT, SURF, FAST, FASTER9, FASTER10, FASTER12 and ORB detectors. Descriptors currently supported include: SIFT, SURF, Intensity Domain Spin Image, Polar Image, Log Polar Image and ORB descriptors. User can tune the parameters for each of the detectors/descriptors
2. Robust stereo matching is supported currently, User can click on a detected key point in image1 and the best matching key-point in image 2 is displayed. A plot showing descriptor distances to all other descriptors in image 2 is also shown. Descriptors from both the images are also shown for visualization purposes. 
3. Forward and backward step-by-step play of images for application of detector / descriptor to certain images, to allow user to inpect the evolution of performance
4. Option for the user to provide input as single image, stereo image or datasets (single/stereo) and perform detection / description on each image is also provided.
5. Image Decimation by providing a decimation factor.

To be done in the immediate future:
1. Add Features like LSD/BLD detector/descriptor, AKAZE Detectora and LATCH Descriptor by creating wrappers around opencv code using CFeatureExtraction class

**Sample Run**
- A sample run of the app requires the user to choose the detector, descriptor and enter the required parameters for the selected options.
- Click on evaluate Detector, then Click on evaluate detector and finally click on visualize Descriptor
- After this the user can select a specific Key-Point in image 1 and visualize the descriptor around it for a single image or find the best matching key-point in case of a stereo image.

=======
This project is available in the mrpt-2.0-devel branch for development

