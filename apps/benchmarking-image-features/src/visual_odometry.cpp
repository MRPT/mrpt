//
// Created by raghavender on 19/07/17.
//

#include "visual_odometry.h"

int ground_truth_x[MAX_FRAME];
int ground_truth_y[MAX_FRAME];

int predicted_x[MAX_FRAME];
int predicted_y[MAX_FRAME];

/************************************************************************************************
*					    Display function                                                        *
************************************************************************************************/
void VisualOdometry::display()
{
    for(int i=2 ; i <MAX_FRAME ; i++)
    {
        cout << ground_truth_x[i] << " gt_x " << predicted_x[i] << " pd_x" << endl;
        cout << ground_truth_y[i] << " gt_y " << predicted_y[i] << " pd_y" << endl;

    }
}

/************************************************************************************************
*					    Compute Odometry Error                                                  *
************************************************************************************************/
double VisualOdometry::computeOdomError()
{
    double error = 0.0;
    double total = 0;
    for(int i=2 ; i<MAX_FRAME-1 ; i++)
    {
        error = error + sqrt((pow((predicted_x[i]-predicted_x[i+1]),2) + pow((predicted_y[i]-predicted_y[i+1]),2)));
        total = total + sqrt((pow((ground_truth_x[i]-ground_truth_x[i+1]),2) + pow((ground_truth_y[i]-ground_truth_y[i+1]),2)));
    }
    double percentage = abs(error-total) / (double) total * 100.00;
    cout << error << " error " << total << " total " << percentage << " % accuracy" << endl;
    //display();
    return percentage;
}

/************************************************************************************************
*					    Get Absolute Scale                                                      *
************************************************************************************************/
double VisualOdometry::getAbsoluteScale(int frame_id, int sequence_id, double z_cal, string poses_ground_truth)
{

    string line;
    int i = 0;
    //ifstream myfile ("/home/raghavender/Downloads/dataset3/poses/01c.txt");
    ifstream myfile (poses_ground_truth);


    double x =0, y=0, z = 0;
    double x_prev, y_prev, z_prev;
    if (myfile.is_open())
    {
        while (( getline (myfile,line) ) && (i<=frame_id))
        {
            z_prev = z;
            x_prev = x;
            y_prev = y;
            std::istringstream in(line);
            //cout << line << '\n';
            for (int j=0; j<12; j++)  {
                in >> z ;
                if (j==7)
                {
                    y = z;
                    // ground_truth_y[i] = y;
                }
                if (j==3)
                {
                    x=z;
                    // ground_truth_x[i] = x;
                }
            }
            i++;
        }
        myfile.close();
    }

    else {
        cout << "Unable to open file";
        return 0;
    }

    return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;

}

/************************************************************************************************
*					    Store Ground Truth                                                      *
************************************************************************************************/
void VisualOdometry::storeGroundTruth(string poses_ground_truth)	{

    string line;
    int i = 0;
    //ifstream myfile ("/home/raghavender/Downloads/dataset3/poses/01c.txt");
    ifstream myfile (poses_ground_truth);

    double x =0, y=0, z = 0;
    if (myfile.is_open())
    {
        while (( getline (myfile,line) ) && (i<=MAX_FRAME))
        {
            std::istringstream in(line);
            //cout << line << '\n';
            for (int j=0; j<12; j++)
            {
                in >> z ;
                //cout << z << " z value " ;
                if (j==11)
                {
                    y = z;
                    ground_truth_y[i] = (int)y+100;
                    cout << y << " y ";
                }
                if (j==3)
                {
                    x=z;

                    ground_truth_x[i] = (int)x+300;
                    cout << x << " x " << endl;
                }
            }
            i++;
        }
        myfile.close();
    }
    else
    {
        cout << "Unable to open file";
    }
}

/************************************************************************************************
*					    Visual Odometry Constructor                                             *
************************************************************************************************/
VisualOdometry::VisualOdometry(int detector_selected, CFeatureExtraction fext, int numFeats)
{
    this->detector_selected = detector_selected;
    this->fext = fext;
    this->numFeats = numFeats;
}

/************************************************************************************************
*					    Generate VO main funciton                                               *
************************************************************************************************/
Mat VisualOdometry::generateVO(string dataset, string groundtruth, int feat_type )	{

    Mat img_1, img_2;
    Mat R_f, t_f; //the final rotation and tranlation vectors containing the transform

    ofstream myfile;
    myfile.open ("results1_1.txt");

    cout << "in generateVO method" << endl;

    double scale = 1.00;
    //char filename1[200];
    //char filename2[200];

    string filename1,filename2;

    stringstream temp1;
    temp1 << dataset << "/000000.png";
    filename1 = temp1.str();
    stringstream temp2;
    temp2 << dataset << "/000001.png";
    filename2 = temp2.str();

    char text[100];
    int fontFace = FONT_HERSHEY_PLAIN;
    double fontScale = 1.5;
    int thickness = 1;
    cv::Point textOrg(10, 50);

    //read the first two frames from the dataset
    Mat img_1_c = imread(filename1);
    Mat img_2_c = imread(filename2);

    Mat dummy;
    if ( !img_1_c.data || !img_2_c.data ) {
        std::cout<< " --(!) Error reading images " << std::endl; return dummy;
    }

    // we work with grayscale images
    cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
    cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);

    CImage img1,img2;
    img1.loadFromFile(filename1);

    img2.loadFromFile(filename2);

    // feature detection, tracking
    vector<Point2f> points1, points2;        //vectors to store the coordinates of the feature points

    featureDetection(img1, points1, feat_type);        //detect features in img_1

    vector<uchar> status;
    featureTracking(img_1,img_2,points1,points2, status); //track those features to img_2

    //TODO: add a fucntion to load these values directly from KITTI's calib files
    // WARNING: different sequences in the KITTI VO dataset have different intrinsic/extrinsic parameters
    double focal = 718.8560;
    cv::Point2d pp(607.1928, 185.2157);
    //recovering the pose and the essential matrix
    Mat E, R, t, mask;
    E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, points2, points1, R, t, focal, pp, mask);

    Mat prevImage = img_2;
    Mat currImage;
    vector<Point2f> prevFeatures = points2;
    vector<Point2f> currFeatures;

    char filename[100];

    R_f = R.clone();
    t_f = t.clone();

    clock_t begin = clock();

    //namedWindow( "Road facing camera", WINDOW_AUTOSIZE );// Create a window for display.
    //namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.

    Mat traj = Mat::zeros(600, 600, CV_8UC3);

    for(int numFrame=2; numFrame < MAX_FRAME; numFrame++)
    {
        stringstream temp3;
        temp3 << dataset << "/%06d.png" ;
        string temp4 = temp3.str();

        sprintf(filename, temp4.c_str(), numFrame);

        //cout << numFrame << endl;
        Mat currImage_c = imread(filename);
        CImage img3;
        img3.loadFromFile(filename);

        cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);
        vector<uchar> status;
        featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

        E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
        recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

        Mat prevPts(2,prevFeatures.size(), CV_64F), currPts(2,currFeatures.size(), CV_64F);


        for(int i=0;i<prevFeatures.size();i++)	{   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
            prevPts.at<double>(0,i) = prevFeatures.at(i).x;
            prevPts.at<double>(1,i) = prevFeatures.at(i).y;

            currPts.at<double>(0,i) = currFeatures.at(i).x;
            currPts.at<double>(1,i) = currFeatures.at(i).y;
        }

        scale = getAbsoluteScale(numFrame, 0, t.at<double>(2), groundtruth);

        //cout << "Scale is " << scale << endl;

        if ((scale>0.1)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {

            t_f = t_f + scale*(R_f*t);
            R_f = R*R_f;

        }

        else {
            //cout << "scale below 0.1, or incorrect translation" << endl;
        }

        // lines for printing results
        // myfile << t_f.at<double>(0) << " " << t_f.at<double>(1) << " " << t_f.at<double>(2) << endl;

        // a redetection is triggered in case the number of feautres being trakced go below a particular threshold
        if (prevFeatures.size() < MIN_NUM_FEAT)	{
            //cout << "Number of tracked features reduced to " << prevFeatures.size() << endl;
            //cout << "trigerring redection" << endl;
            featureDetection(img3, prevFeatures, feat_type);
            featureTracking(prevImage,currImage,prevFeatures,currFeatures, status);

        }

        prevImage = currImage.clone();
        prevFeatures = currFeatures;

        int x = int(t_f.at<double>(0)) + 300;
        int y = int(t_f.at<double>(2)) + 100;
        circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);

        //cout << "predict: " << x << " x " << y << " y " << endl;
        predicted_x[numFrame] = x;
        predicted_y[numFrame] = y;

        rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
        //sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
        sprintf(text, "Green: ground truth ; Red: Actual path");
        putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
    }

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << "Total time taken: " << elapsed_secs << "s" << endl;

    storeGroundTruth(groundtruth);
    for(int i=0 ; i<MAX_FRAME ; i++)
    {
        int x = (int)ground_truth_x[i];
        int y = (int)ground_truth_y[i];
        cout << "x: " << x << " y: " <<y << endl;

        circle(traj, Point(x, y) ,1, CV_RGB(0,255,0), 2);
    }

    computeOdomError();
    return traj;
}


/************************************************************************************************
*					    Feature Tracking method                                                 *
************************************************************************************************/
void VisualOdometry::featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)	{

//this function automatically gets rid of points for which tracking fails

    vector<float> err;
    Size winSize=Size(21,21);
    TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

    calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    int indexCorrection = 0;
    for( int i=0; i<status.size(); i++)
    {
        Point2f pt = points2.at(i- indexCorrection);
        if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
            if((pt.x<0)||(pt.y<0))	{
                status.at(i) = 0;
            }
            points1.erase (points1.begin() + (i - indexCorrection));
            points2.erase (points2.begin() + (i - indexCorrection));
            indexCorrection++;
        }

    }

}

/************************************************************************************************
*					    Feature Detection method                                                *
************************************************************************************************/
void VisualOdometry::featureDetection(CImage img1, vector<Point2f>& points1, int feat_type)	{   //uses FAST as of now, modify parameters as necessary

    vector<KeyPoint> keypoints_1;


    CFeatureList featsList1;
    fext.detectFeatures(img1,featsList1,numFeats);


    vector<KeyPoint>::iterator iterator = keypoints_1.begin();


    for(int i=0 ; i<featsList1.size() ; i++)
    {
        Point2f temp_pt(featsList1.getFeatureX(i),featsList1.getFeatureY(i));
        KeyPoint temp;
        temp.pt.x= 3;
        temp.pt.y = 4;
        keypoints_1.push_back(temp);
        points1.push_back(temp_pt);
    }

}