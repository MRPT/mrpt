#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#define NUM_DETECTORS 12
#define NUM_DESCRIPTORS 8
#define IMAGE_WIDTH 400
#define IMAGE_HEIGHT 400
#define WIDGET_HEIGHT 30
#define PARAMS_HEIGHT 30
#define PARAMS_WIDTH 80
#define WIDGET_WIDTH 160
#define BUTTON_WIDTH 150
#define BUTTON_HEIGHT 30
#define CIRCLE_THICKNESS 4
#define CROSS_THICKNESS 3
#define CROSS_SIZE 20



//for the visualize descriptor part
#define DESCRIPTOR_WIDTH 100
#define DESCRIPTOR_HEIGHT 100
#define DESCRIPTOR_GRID_SIZE 9
#define DESCRIPTOR_ROW_SIZE 3

#define DESCRIPTOR_GRID_SIZE2 9
#define MAX_DESC 500


#include <QMainWindow>
#include <QObject>
#include <QWidget>
#include <QApplication>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QRadioButton>
#include <QLineEdit>
#include <QMessageBox>
#include <QGroupBox>
#include <QListWidget>
#include <QComboBox>
#include <QFileDialog>
#include <QMap>
#include <QtGui>
#include <QVector>

#include <QMouseEvent>
#include <QEvent>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "opencv2/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"


#include <mrpt/vision/CFeatureExtraction.h>

#include "my_qlabel.h"

//#include "visual_odometry.cpp"

using namespace cv;
using namespace std;

using namespace mrpt::vision;
using namespace mrpt::utils;

using namespace mrpt::math;
using namespace mrpt;

class MainWindow : public QMainWindow
{
    Q_OBJECT


public:
    QWidget *window_gui;
    //VisualizeDialog *visualize_dialog;

    QLabel *sample_image;
    QImage my_image;

    // for image decimation
    double sampling_rate;

    QPushButton *button_generate;
    QPushButton *button_close;
    QPushButton *next_desc;

    QPushButton *prev_button;
    QPushButton *next_button;
    int current_imageIndex;

    QPushButton *browse_button;
    QPushButton *browse_button2;
    QPushButton *generateVisualOdometry;
    QGridLayout *layout_grid;

    int currentInputIndex;
    int detector_selected;
    int descriptor_selected;

    QGroupBox *groupBox1; // has the
    QGroupBox *groupBox2;
    QGroupBox *groupBox_images;

    QComboBox *inputs;
    QLineEdit *inputFilePath;
    QLineEdit *inputFilePath2;
    string file_path1;
    string file_path2;

    QLineEdit *numFeaturesLineEdit;

    QComboBox *detectors_select;
    QComboBox *descriptors_select;

    my_qlabel *image1;
    QLabel *image2;
    QImage qimage1;
    QImage qimage2;

    QLabel *param1;
    QLabel *param2;
    QLabel *param3;
    QLabel *param4;
    QLabel *param5;

    QLineEdit *param1_edit;
    QLineEdit *param2_edit;
    QLineEdit *param3_edit;
    QLineEdit *param4_edit;
    QLineEdit *param5_edit;


    QLabel *param1_desc;
    QLabel *param2_desc;
    QLabel *param3_desc;
    QLabel *param4_desc;
    QLabel *param5_desc;

    QLineEdit *param1_edit_desc;
    QLineEdit *param2_edit_desc;
    QLineEdit *param3_edit_desc;
    QLineEdit *param4_edit_desc;
    QLineEdit *param5_edit_desc;

    //provide user options like stereo matching, activate/deactivate non-maximal suppression, image decimation, step-by-step playback of images.
    QCheckBox *stereo_matching;

    //image decimation options
    QLineEdit *decimateFactor;



    //visual odom parameters
    QLineEdit *inputFilePath3;
    QPushButton *browse_button3;
    string file_path3;  //!<stores the ground truth for poses


    QLabel *output1;

    CFeatureExtraction fext;
    CFeatureList featsImage1, featsImage2;
    CImage img1, img2;
    TDescriptorType desc_to_compute;

    int numFeats; //  number of features

    // Detector OPTIONS

    /** FAST Options */
    struct FASTOptions
    {
        float threshold;
        float min_distance;
        bool non_max_suppresion;
        bool use_KLT_response;
    }fast_opts;

    /** KLT Options */
    struct KLTOptions
    {
        float threshold;
        int radius;
        float min_distance;
        bool tile_image;
    }klt_opts;

    /** Harris Options */
    struct HarrisOptions
    {
        float threshold;
        float k;
        float sigma;
        float radius;
        float min_distance;
        bool tile_image;
    }harris_opts;

    /** SIFT Options */
    struct SIFTOptions
    {
        float edge_threshold;
        float threshold;
    }SIFT_opts;

    /** SURF Options */
    struct SURFOptions
    {
        int hessianThreshold;
        int nLayersPerOctave;
        int nOctaves;
        bool rotation_invariant;
    }SURF_opts;

    /** ORB Options */
    struct ORBOptions
    {
        int min_distance;
        int n_levels;
        float scale_factor;
        bool extract_patch;
    }ORB_opts;

    // not providing option to choose descriptor_type and diffusivity, using default values currently
    /** AKAZE Options */
    struct AKAZEOptions
    {
        //int descriptor_type
        int  	descriptor_size;
        int  	descriptor_channels;
        float  	threshold ;
        int  	nOctaves;
        int  	nOctaveLayers;
        //int  	diffusivity;
    }AKAZE_opts;

    /** LSD Options */
    struct LSDOptions
    {
        int  	scale;
        int  	nOctaves;
    }LSD_opts;

    //DESCRIPTOR OPTIONS
    /** SpinImagesOptions Options
                    */
    struct SpinImageOptions
    {
        int radius;
        int hist_size_intensity;
        int hist_size_distance;
        float std_dist;
        float std_intensity;
    }spin_opts;

    /** PolarImagesOptions Options
				  */
    struct PolarImageOptions
    {
        int radius;
        int bins_angle;
        int bins_distance;
    }polar_opts;

    /** LogPolarImagesOptions Options
				  */
    struct LogPolarOptions
    {
        int radius;
        int num_angles;
        float rho_scale;

    }log_polar_opts;

    /** BLDOptions Options
				  */
    struct BLDOptions
    {
        int ksize_;
        int reductionRatio;
        int numOfOctave;
        int widthOfBand;
    }BLD_opts;

    /** LATCHOptions Options
				  */
    struct LATCHOptions
    {
        int bytes;
        bool rotationInvariance;
        int half_ssd_size;
    }LATCH_opts;



    //FOR THE VISUALIZE DESCRIPTOR PART
public:
    QLabel *images_static;
    QLabel *images_static_sift_surf;

    QLabel *images_static2;
    QLabel *images_static_sift_surf2;

    QImage descriptors;

    QGridLayout *desc_VisualizeGrid;

    long numDesc1, numDesc2;
    int cnt; // counter to iterate over all the descriptors


    QLabel *images_label_coordinates;
    QLabel *images_label_coordinates_sift_surf;
    QLabel *featureMatched;

    QLabel *images_plots_sift_surf;

    double  mouse_x, mouse_y;
    int old, newer;
    bool flag_descriptor_match;
    bool flag_read_files_bug;


    my_qlabel *sample2;


    // variables for bug fixes

    bool evaluate_detector_clicked;
    bool evaluate_descriptor_clicked;
    bool visualize_descriptor_clicked;
    bool activate_stereo_matching;


    //! <Parameters for Evaluations Characteristiscs

    double elapsedTime_detector;
    double elapsedTime_descriptor;
    QLabel *detector_info;
    QLabel *descriptor_info;
    QLabel *descriptor_info2;
    QLabel *descriptor_info3;
    QLabel *evaluation_info;
    double closest_dist;

    QLabel *visualOdom;


signals:
    // None yet

public:

    /**
     * MainWindow(QWidget) constructor which creates all the widgets and displays them on screen registers all the signals and slots for the buttons, comboboxes, etc.
     * it sets the layout/sizes for all the widgets to be displayed in the QMain window
     * @param window_gui asks a QWidget default=0
     */
    explicit MainWindow(QWidget *parent=0);

    /**
     * readInputFormat() reads and stores the states of the user selections for different inputs like (currentInputIndex,
     * detector_selected, descriptor selected, file_path1 and file_path2) and can be used by other functions when required
     */
    void ReadInputFormat();

    /**
     * makeAllDetectorParamsVisible(bool) makes all the detector parameters (QLabel, QLineEdits) in the main window either visible or hides them depending on the flag variable's value
     * @param flag variable to store if all labels/text fields are to be made visible or hidden
     */
    void makeAllDetectorParamsVisible(bool flag);

    /**
     * makeAllDescriptorParamsVisible(bool) makes all the descriptors parameters (QLabel, QLineEdits) in the main window either visible or hides them depending on the flag variable's value
     * @param flag variable to store if all labels/text fields are to be made visible or hidden
     */
    void makeAllDescriptorParamsVisible(bool flag);

    /**
     * fillDetectorInfo() function is used to fill in the required parameters for the chosen detector, the parameters entered
     * by the user are read and stored in the appropriate variables, later these values are stored in the fext.options.DETECTOR.parameter field
     * example fext.options.KLTOptions.min_distance, fext.options.harrisOptions.sigma, etc.
     */
    void fillDetectorInfo();

    /**
     * fillDescriptorInfo() function is used to fill in the required parameters for the chosen descriptor, the parameters entered
     * by the user are read and stored in the appropriate variables, later these values are stored in the fext.options.DESCRIPTOR.parameter field
     * example fext.options.SURFOptions.hessianThreshold, fext.options.SIFTOptions.edgeThreshold, etc.
     */
    void fillDescriptorInfo();

    /**
      * readFilesFromFolder(int) reads the files from a folder, the function is called when the user presses the next or previous button
      * it iterates over the images present in the image dataset folder specified by the user
      * the function works under the assumption both the stereo image datasets have the same number of images
      * @param next_prev indicates 1 for next image and 0 for previous button being pressed
      */
    void readFilesFromFolder(int next_prev);

    /**
     * displayImagesWithoutDetector() function displays the images without detectors as QLabels, this function is called when NEXT or PREVIOUS buttons are pressed
     */
    void displayImagesWithoutDetector();

    /**
     * initializeParameters() initializes the content for the detector/descriptor QLabels/QLineEdits, it also sets the size of these labels and text fields
     */
    void initializeParameters();

    /**
     * findClosest function finds the closest point to the input (x,y) coordinates from the array of points (X[],Y[])
     * the function is called when the user clicks on a key-point in the QLabel which has the QImage
     * @param x input mouse click's x-coordinate
     * @param y input mouse click's y-coordinate
     * @param X input array of X coordinates which has all key-points' x-coordinates
     * @param Y input array of Y coordinates which has all key-points' y-coordinates
     * @param n input the size of the X[],Y[] array of key-points
     * @return returns the index of the closest key-point to that clicked by the mouse
     */
    int findClosest(double x, double y, double X[], double Y[], int n);

    /**
     * drawLineLSD() draws the lines (LSD line detectors) in the image on the key-points for the image.
     * @param img the input image on which the lines need to be drawn
     * @param image_left_right  this indicates the image on which to draw the lines (0: left; 1:right)
     */
    void drawLineLSD(Mat img, int image_left_right);

    string findRepeatability(float mouse_x, float mouse_y);

    bool checkIfSamePoint(float x, float y, float x2, float y2, int threshold);

    string falsePositivesNegatives();
    void showEvaluation(int mode);



private slots:

    /**
     * Mouse_current_pos() slot function simply used to test the functionality of MouseEvents, not currently used
     */
    void Mouse_current_pos();

    /**
     * Mouse_Pressed() slot function called when the user clicks on a particular point in the QLabel for image 1 in the GUI, it finds the closest keypoint in image1
     * and shows the corresponding best image in image2, it reads the descriptors stored by the on_button_generate_clicked() from global variables
     * it then displays the descriptors from the single/stereo images onto the GUI
     * also shows the plot of all descriptor distances in image2 to the descriptor (around key-point) clicked by the user
     */
    void Mouse_Pressed();

    /**
     * Mouse_left() slot function called when mouse leaves the QLabel on which this function is registeres as slot, not currently used
     */
    void Mouse_left();


public slots:

    /**
     * on_button_generate_clicked() is called when user clicks on visualize descriptor.
     * the function computes and stores the visualizations in memory to be later used when the user clicks on the QLabel to show the descriptors.
     * the function populates all the descriptors in the left and right images in appropriate variables and also stores the which of the descriptors in image1
     * match best to those in image2.
     */
    void on_button_generate_clicked();

    /**
     * button_close_clicked() simply closes the wondow
     */
    void button_close_clicked();

    /**
     * on_detector_button_clicked() function shows the performance of the selected detector on the input image/dataset selected by the user
     * detected key-points are shown on the images which are rendered as QLabels
     * For the case when user selects SingleImage/StereoImage Dataset, this calls the readFilesFromFolder(int) function which stores the file names for all images present in the folder
     * this function also calls fillDetectorInfo to populate detector parameters
     * the performance metric displayed is in terms of repeatability, dispersion of image, computational cost, number of found points, etc.
     * */
    void on_detector_button_clicked();

    /**
     * on_descriptor_button_clicked() slot function is called when the user clicks the evaluate Descriptor button, it computes the selected descriptor from the QComboBox
     * for the image and stores the descriptor in the featsImage1 and featsImage2 variables.
     * the function calls the fillDescriptorInfo() function which fills the parameters for the selected descriptor
     */
    void on_descriptor_button_clicked();

    /**
     * on_browse_button_clicked() slot function browses for the image files that the user can select or the directory that the user can select for the first image/folder
     * it stores the file paths in the appropriate global variables which are then used by other functions to read the images from
     */
    void on_browse_button_clicked();

    /**
     * on_browse_button_clicked2() slot function browses for the image files that the user can select or the directory that the user can select for the second image/folder
     * it stores the file paths in the appropriate global variables which are then used by other functions to read the images from
     */
    void on_browse_button_clicked2();

    void on_browse_button_clicked3();

    /**
     * on_descriptor_choose() slot function is called when the user selects the descriptor from the list of available descriptors from the QComboBox
     * the function populates the QLabels and the QLineEdits for the selected descriptor option, it also hides the QLabels and QLineEdits that are not required
     * @param choice stores the descriptor chosen by the user
     */
    void on_descriptor_choose(int choice);

    /**
     * on_detector_choose() slot function is called when the user selects the detector from the list of available detectors from the QComboBox
     * the function populates the QLabels and the QLineEdits for the selected detector option, it also hides the QLabels and QLineEdits that are not required
     * @param choice stores the detector chosen by the user
     */
    void on_detector_choose(int choice);

    /**
     * on_file_input_choose(int) slot function is called when the user selects the type of the input to be used from the QCombo
     * @param choice stores the type of input selected, 0: single image, 1: stereo iamge, 2: image raw log file, 3: single image dataset, 4: stereo image dataset
     */
    void on_file_input_choose(int choice);

    /**
     * this function is currently not required as stereo matching is done by mouse clicks
     * @param state
     */
    void onStereoMatchingChecked(int state);

    /**
     * on_next_button_clicked() function is called when the user clicks on the next button t oiterate over the images in the dataset
     * this function calls the readFilesFromFolder(int) and displayImagesWithoutDetector() functions
     */
    void on_next_button_clicked();

    /**
     * on_prev_button_clicked() function is called when the user clicks on the previous button to iterate over the images in the dataset
     * this function calls the readFilesFromFolder(int) and displayImagesWithoutDetector() functions
     */
    void on_prev_button_clicked();

    /**
     * on_sample_clicked() slot function decimates the image by the decimation factor entered by the user
     * e.g, entering 0.1 enlarges the image to 110% and -0.2 reduces it to 80% of its actual resolution
     */
    void on_sample_clicked();

    void on_generateVisualOdometry_clicked();

};

#endif // MAINWINDOW_H
