Steps to use the app:


**Viewing Detectors /Descriptor on a given single image / single dataset**

1. Choose Detector
2. Just leave the detector parameters to their default values or change if required
3. Choose Descriptor
4. Just leave the descriptor parameters to their default values or change if required
5. Choose input type single image or single dataset
6. Browse input file path
7. Click Evaluate Detector
8. Click Evaluate Descriptor
9. Click on Visualize Descriptor
10. Click on any Key-Point in the image

Result: See the descriptor of the selected key-point and its distance to all other descriptors below, evaluation metrics like repeatability for dataset only, and false positives/negatives, % of correct matches, etc. can be seen. If single image dataset selected, user can click on next/previous button to navigate through the dataset and evaluate detectors performance

**Viewing Detectors /Descriptor on a given stereo image / stereo dataset**

1. Choose Detector
2. Just leave the detector parameters to their default values or change if required
3. Choose Descriptor
4. Just leave the descriptor parameters to their default values or change if required
5. Choose input type stereo image or stereo dataset
6. Browse input file path
7. Click Evaluate Detector
8. Click Evaluate Descriptor
9. Click on Visualize Descriptor
10. Click on any Key-Point in the image

Result: See the descriptor of the selected key-point and its distance to all other descriptors below, evaluation metrics like repeatability for dataset only, and false positives/negatives, % of correct matches, etc. can be seen. If stereo image dataset selected, user can click on next/previous button to navigate through the dataset and evaluate detectors performance

**Vision Tasks**

**Visual Odometry**
1. Choose Detector
2. Choose input type in the combobox as Single Image dataset
3. Browse for the KITTI dataset folder having colored images
4. Click on Evaluate Detector
5. Check/Tick the box of "Perform Visual Odometry"
6. Browse for the ground truth file something like 00.txt, 01.txt,.. etc.
7. Click on generate visual odometry

Result: The ground truth and the predicted VO path can be seen in the app.

**Tracking**

1. Choose Detector
2. Choose Input type in the combobox as Single Image Dataset
3. Browse for any tracking dataset
4. Check/Tick the Activate Tracking of Key-Points
5. Enter the tracking parameters or leave default unchanged
6. Click on Track Key-Points button 2 times and start seeing the tracking from the third frame onwards
7. Repeatedly clicking on the track button tracks the keypoints.
8. At any point change the parameters to change the tracking behaviour

Result: Clicking on the Track-KeyPoints button tracks the keypoints in the subsequent frames, the tracker parameters can be changed at any time to change the tracking behaviour.

**Homography Based Repeatability**

1. Choose Detector
2. Just leave the detector parameters to their default values or change if required
3. Choose Descriptor
4. Just leave the descriptor parameters to their default values or change if required
5. Choose input type as single or stereo image dataset
6. Browse input file path
7. Check/Tick the Activate Homography Based Repeatability
8. Click on Browse Homography to choose the folder which has homography files stored
9. Click Evaluate Detector
10. Click Evaluate Descriptor
11. Click on Visualize Descriptor
12. Click on any Key-Point in the image
13. View the results on the right side of the image about the different evaluation metrics

Result: The results of the selected keypoint in the image can be seen on the right side of the image.

**Place Recognition**

1. Choose Detector
2. Just leave the detector parameters to their default values or change if required
3. Choose Descriptor
4. Just leave the descriptor parameters to their default values or change if required
5. Choose input type as single image dataset
6. Check the Perform Place Recognition CheckBox
7. Click on the browse training dataset button and choose the folder for the training dataset
8. Click on the browse testing dataset button and choose the folder for the testing dataset
9. Click on the Train Place Recognition Button to train the model (extraction of features from the training dataset)
10. Click on the Recognize Next Image button to classify the subsequent images
11. Keep clicking on the recognize button to classify subsequent frames
12. The output of each frame and the accuracy can be seen below the image
