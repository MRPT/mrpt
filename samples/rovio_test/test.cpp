/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers/CRovio.h>
#include <mrpt/hwdrivers/CJoystick.h>
#include <mrpt/utils/utils_defs.h>
#include <mrpt/slam.h>

#include <fstream>
#include <cmath>


using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace std;


/*--------------------------------
|            VARIABLES            |
---------------------------------*/
struct data_st
{
	CRovio Robot;
	bool showvideo, features_taken, matched, matching_done, fm_calculated;
};

CDisplayWindow   win("Video"),wind1,wind2;	//wind3,wind4,wind5,wind6; 	//open a window to show the video

/*--------------------------------
|            FUNCTIONS            |
---------------------------------*/

// Int to binary and print.
void binary(int number) {
    int remainder;

    if(number <= 1) {
        cout << number;
        return;
    }
    remainder = number%2;
    binary(number >> 1);
    cout << remainder;
}


//Print the encoders readed from the CRovio Class
void print_Encoders(int *arr)
{
	string field_name[12]={"Packet length","Not Used","Left Wheel:Dir Rotation","Left Wheel:Ticks","Right Wheel:Dir Rotation","Right Wheel:Ticks","Rear Wheel:Dir Rotation","Rear Wheel:Ticks","Not used","Head Position","Batery","Config Status"};

	//Code
		cout << "\n-----------------ENCODERS------------------\n";
		for(int i=0;i<=11;i++)
		{
			cout << field_name[i] <<" -> ";

			if(i==10) cout <<hex << arr[i] <<dec;
			else cout << arr[i];

			if (i==11)
			{
				cout <<" = "; binary(arr[i]);	//print binary value
			}
			cout<<endl;
		}
		cout << "\n-------------------------------------------\n";
}


//Show the Position given by the NorthStar system
void showPosition(data_st &Pdata)
{
	mrpt::math::TPose2D pose_out; //Pose sent to the console
	
	if (Pdata.Robot.getPosition(pose_out))
	{
		cout << "\n pose: " << CPose2D(pose_out) << endl;
	}
	else cerr << "Error\n";
}

void goThere(data_st &Pdata, string& response, string& errormsg)
{
	mrpt::math::TPose2D pose;
	int x_t, y_t; float theta_t;	//Target Position
	float  x_d, y_d, theta_d, distance, angle;	//Distance to the Target
	bool target=false;
	do{

		cout<<"\nEnter the target (x, y & theta)\n";
		cin >> x_t >>y_t >>theta_t;
		
		Pdata.Robot.getPosition(pose);

		x_d=x_t-pose.x;
		y_d=y_t-pose.y;
		distance=sqrt(x_d*x_d+y_d*y_d);
		while(distance>500)
		{
			theta_d=atan2(x_d,(-y_d));
			angle=theta_d-pose.phi;
			if(angle>M_PI)
				angle-=2*M_PI;
			if(angle<(-M_PI))
				angle+=2*M_PI;
			while(angle>0.6 || angle<(-0.6))
			{
				if(angle>0.6)
					Pdata.Robot.Moving(18, 5, 10, response, errormsg);
				if(angle<(-0.6))
					Pdata.Robot.Moving(18, 6, 10, response, errormsg);
				Pdata.Robot.getPosition();
				angle=theta_d-pose.phi;
				if(angle>M_PI)
					angle-=2*M_PI;
				if(angle<(-M_PI))
					angle+=2*M_PI;
			}
			Pdata.Robot.Moving(18, 1, 5, response, errormsg);
			Pdata.Robot.getPosition();
			x_d=x_t-pose.x;
			y_d=y_t-pose.y;
			distance=sqrt(x_d*x_d+y_d*y_d);
		}
		angle=theta_t-pose.phi;
		if(angle>M_PI)
			angle-=2*M_PI;
		if(angle<(-M_PI))
			angle+=2*M_PI;
		while(angle>0.6 || angle<(-0.6))
		{
			if(angle>0.6)
				Pdata.Robot.Moving(18, 6, 10, response, errormsg);
			if(angle<(-0.6))
				Pdata.Robot.Moving(18, 5, 10, response, errormsg);
			
			Pdata.Robot.getPosition(pose);
			angle=theta_t-pose.phi;
			if(angle>M_PI)
				angle-=2*M_PI;
			if(angle<(-M_PI))
				angle+=2*M_PI;
		}
		if(distance<500 && theta_d<0.6 && theta_d>(-0.6))
			target=true;
	}while(!target);
}



//Shows the video in the "win" window.
 void videowindow(data_st* data)
{
	CObservationImagePtr lastimg;

	while(data->showvideo)
	{
		data->Robot.getLastImage2(lastimg);
		if(lastimg)	//Avoid NULL image
			win.showImage(lastimg->image);
		mrpt::system::sleep(50);
	}
}


/*--------------------------------
|       JOYSTICK CONTROLS         |
---------------------------------*/
void JoystickControl(data_st *data)
{
	CJoystick joy;
	float x,y,z;
	vector <bool> buttons;
	string response, errormsg;
	int action, direction;
	//CTicTac tictac;
	while(joy.getJoystickPosition(0,x,y,z,buttons))
	{
		//printf("Joystick readings: %.03f, %.03f, %.03f  (", x, y, z);
		int speed=(int)(10-(x*x+y*y)*9);
		if(x<-0.2 || x>0.2 || y <-0.2 || y>0.2)
		{
			if(y<-0.2)
			{
				if(x<0.2 && x>-0.2){//Move Forward
					action=18; direction=1;}
				if(x<-0.2){//Move Forward & Left
					action=18; direction=7;}
				if(x>0.2){//Move Forward & Right
					action=18; direction=8;}
			}

			if(y>0.2)
			{
				if(x<0.2 && x>-0.2){ //Move Backward
					action=18; direction=2;}
				if(x>0.2){ //Move Backward & Right
					action=18; direction=10;}
				if(x<-0.2){ //Move Backward & Left
					action=18; direction=9;}
			}
			if(y>-0.2 && y<0.2)
			{
				if(x<-0.2){ //Move Left
					action=18; direction=3;}
				if(x>0.2){ //Move Right
					action=18; direction=4;}
			}
			data->Robot.Moving(action, direction, speed, response, errormsg);
		}

		if((buttons[3] && buttons[2])||(buttons[0] && buttons[1])||(y<-0.5 && buttons[2])||(buttons[0] && buttons[2]))
		{
			if(buttons[3] && buttons[2]) //Take head up
				data->Robot.Moving(18, 11, 5, response, errormsg);
			if(buttons[0] && buttons[1]) //Go home
				data->Robot.Moving(12, 12, 5, response, errormsg);
			if(y<-0.5 && buttons[2]) //Go home & dock
				data->Robot.Moving(13, 12, 5, response, errormsg);
			if(buttons[0] && buttons[2]) //Get Report
			{
				data->Robot.Moving(1, 12, 5, response, errormsg);
				cout<<"Response:\n"<<response;
			}
		}
		else
		{
			if(buttons[0]) //Turn to the left
				data->Robot.Moving(18, 5, 5, response, errormsg);
			if(buttons[1]) //Turn to the right
				data->Robot.Moving(18, 6, 5, response, errormsg);
			if(buttons[2]) //Take head to the middle
				data->Robot.Moving(18, 13, 5, response, errormsg);
			if(buttons[3]) //Take head down
				data->Robot.Moving(18, 12, 5, response, errormsg);
		}

		fflush(stdout);
		mrpt::system::sleep(20);
	}
	cout<<"No Joystick connected...\n\n\n"<<endl;
}



/*--------------------------------
|			MATCHING		      |
---------------------------------*/

//Method to compare two matches list
/*
CMatchedFeatureList comp2Matched_lists2(CMatchedFeatureList &list1, CMatchedFeatureList &list2)
{
	CMatchedFeatureList final_list;
	CMatchedFeatureList::iterator			itVec1, itVec2;				// We set an iterator for the vector
	for( itVec1 = list1.begin(); itVec1 != list1.end(); itVec1++)		// ... we search it into the vector
	{
		for( itVec2 = list2.begin(); itVec2 != list2.end(); itVec2++)
		{
			if( itVec1->second->ID == itVec2->first->ID )
			{
				std::pair<CFeaturePtr,CFeaturePtr> mPair( itVec1, itVec2 );
				final_list.insert( mPair );
			}
		} // end for itVec2
	} // end for itVec1
	return final_list;
}
*/
CMatchedFeatureList comp2Matched_lists(CMatchedFeatureList &list1, CMatchedFeatureList &list2)
{
	CMatchedFeatureList final_list=list1;
	CMatchedFeatureList::iterator			itVec1, itVec2;				// We set an iterator for the vector
	for( itVec1 = final_list.begin(); itVec1 != final_list.end();)		// ... we search it into the vector
	{
		for( itVec2 = list2.begin(); itVec2 != list2.end(); itVec2++)
		{
			if( itVec1->second->ID == itVec2->first->ID )
				break;
		} // end for itVec2
		if(itVec2 == list2.end())
			itVec1 = final_list.erase( itVec1 );
		else
			itVec1++;
	} // end for itVec1
	return final_list;
}
void matching(data_st *data)
{
	CTicTac	tictac;
	TMatchingOptions opt;
	opt.useXRestriction=false;
	if(data->matching_type==epipolar_match)
	{
		opt.useEpipolarRestriction=true;
		opt.parallelOpticalAxis=false;
		opt.epipolar_TH=2.0;
		opt.maxEDSD_TH=0.18;
		opt.EDSD_RATIO=0.6;
		opt.F.loadFromTextFile("FM_Harris2ways.txt");
	}
	else
	{
		opt.useEpipolarRestriction=false;
		opt.maxEDSD_TH=0.18;
		opt.EDSD_RATIO=0.6;
	}
	cout<<"\nCalculating matches...\n";

	//opt.F=data->FM_Harris2ways;
	opt.matching_method=TMatchingOptions::mmDescriptorSURF;
	tictac.Tic();
	mrpt::vision::utils::matchFeatures2(data->featsHarris, data->featsHarris2, data->Harris_matched, opt);
	cout << "Detected " << data->Harris_matched.size() << " Harris matches in " << endl;
	cout << format("  %.03fms",tictac.Tac()*1000) << endl;
	data->Harris_matched.saveToTextFile("Harris_matches.txt");

//Calculate matches in the inverse way

	//opt.F=(~opt.F);
	//opt.F=(~data->FM_Harris2ways);
	tictac.Tic();
	mrpt::vision::utils::matchFeatures2(data->featsHarris2, data->featsHarris, data->Harris_matched_inv, opt);
	cout << "Detected " << data->Harris_matched_inv.size() << " Harris matches 'INV' in " << endl;
	cout << format("  %.03fms",tictac.Tac()*1000) << endl;
	data->Harris_matched_inv.saveToTextFile("Harris_matches_inv.txt");

// Only matches found in both ways are kept in a new file (Detect matches in only one way and delete it)
	data->Harris_matched2ways = comp2Matched_lists(data->Harris_matched, data->Harris_matched_inv);
	cout << data->Harris_matched2ways.size() << " Harris matches in both ways\n" << endl;

	data->matching_done=true;
	cout<<"\nMatching finished\n";
}

void matching2(data_st* data)
{
	//data_st *data=(data_st*) Pdata;	//structure cast
	CTicTac	tictac;
	TMatchingOptions opt;
	opt.useXRestriction=false;
	if(data->matching_type==epipolar_match)
	{
		opt.useEpipolarRestriction=true;
		opt.parallelOpticalAxis=false;
		opt.epipolar_TH=2.0;
	}
	else
		opt.useEpipolarRestriction=false;
	cout<<"\nCalculating matches...\n";

//HARRIS

	if(data->matching_type==epipolar_match)
	{	
		//data->FM_Harris.loadFromTextFile("FM_Harris.txt");
		opt.F=data->FM_Harris;

		//opt.minCC_TH=0.94999999;
		//opt.rCC_TH=0.920;

		//opt.maxEDD_TH=0.2;
		//opt.EDD_RATIO=0.5;

		opt.maxEDSD_TH=0.18;
		opt.EDSD_RATIO=0.6;
	}
	else
	{
		//opt.minCC_TH=0.94999999;
		//opt.rCC_TH=0.92;

		//opt.maxEDD_TH=0.2;
		//opt.EDD_RATIO=0.5;

		opt.maxEDSD_TH=0.16;
		opt.EDSD_RATIO=0.55;
	}
	opt.matching_method=TMatchingOptions::mmDescriptorSURF;
	//opt.matching_method=TMatchingOptions::mmCorrelation;
	tictac.Tic();
	mrpt::vision::utils::matchFeatures2(data->featsHarris, data->featsHarris2, data->Harris_matched, opt);
	cout << "Detected " << data->Harris_matched.size() << " Harris matches in " << endl;
	cout << format("  %.03fms",tictac.Tac()*1000) << endl;
	data->Harris_matched.saveToTextFile("Harris_matches.txt");

	opt.F=data->FM_Harris_inv;
	tictac.Tic();
	mrpt::vision::utils::matchFeatures2(data->featsHarris2, data->featsHarris, data->Harris_matched_inv, opt);
	cout << "Detected " << data->Harris_matched_inv.size() << " Harris matches 'INV' in " << endl;
	cout << format("  %.03fms",tictac.Tac()*1000) << endl;
	data->Harris_matched_inv.saveToTextFile("Harris_matches_inv.txt");

//Detect matches in only one way and delete it (Only matches found in both ways are kept in a new file)
	data->Harris_matched2ways = comp2Matched_lists(data->Harris_matched, data->Harris_matched_inv);
	cout << data->Harris_matched2ways.size() << " Harris matches in both ways\n" << endl;
	data->Harris_matched2ways.saveToTextFile("Harris_matches2ways.txt");

/*
//SIFT
	if(data->matching_type==epipolar_match)
	{	
		opt.F=data->FM_SIFT;
		opt.maxEDD_TH=0.2;
		opt.EDD_RATIO=0.5;
	}
	else
	{
		opt.maxEDD_TH=0.2;
		opt.EDD_RATIO=0.4;
	}
	opt.matching_method=TMatchingOptions::mmDescriptorSIFT;
	tictac.Tic();
	mrpt::vision::utils::matchFeatures2(data->featsSIFT_Hess, data->featsSIFT_Hess2, data->SIFT_matched, opt);
	cout << "Detected " << data->SIFT_matched.size() << " SIFT matches in " << endl;
	cout << format("  %.03fms",tictac.Tac()*1000) << endl;
	data->SIFT_matched.saveToTextFile("SIFT_matches.txt");

//SURF	
	if(data->matching_type==epipolar_match)
	{	
		opt.F=data->FM_SURF;
		//opt.maxEDSD_TH=0.15;
		//opt.EDSD_RATIO=0.6;
	}
	else
	{
		opt.maxEDSD_TH=0.12;
		opt.EDSD_RATIO=0.5;
	}
	opt.matching_method=TMatchingOptions::mmDescriptorSURF;
	tictac.Tic();
	mrpt::vision::utils::matchFeatures2(data->featsSURF, data->featsSURF2, data->SURF_matched, opt);
	cout << "Detected " << data->SURF_matched.size() << " SURF matches in " << endl;
	cout << format("  %.03fms",tictac.Tac()*1000) << endl;
	data->SURF_matched.saveToTextFile("SURF_matches.txt");
*/
	data->matching_done=true;
	cout<<"\nMatching finished\n";
}


//GETTING FUNDAMENTAL MATRIX
void getFMat(data_st *data)
{
	CvMat* points1;
	CvMat* points2;
	CvMat* status;
	CvMat* fundamental_matrix;
	//ofstream mat;

// HARRIS
	if(data->Harris_matched.size()<8)
		cout<<"WARNING->Less than 8 elements...\n";
	else
	{
		int point_count = data->Harris_matched.size();
		points1 = cvCreateMat(1,point_count,CV_32FC2);
		points2 = cvCreateMat(1,point_count,CV_32FC2);
		status = cvCreateMat(1,point_count,CV_8UC1);

		CMatchedFeatureList::iterator it;
		unsigned int i, j;

		/* Fill the points here ... */
		for(i=0, it=data->Harris_matched.begin(); it!=data->Harris_matched.end(); it++,i++)
		{
			points1->data.fl[i*2] = it->first->x;
			points1->data.fl[i*2+1] = it->first->y;
			points2->data.fl[i*2] = it->second->x;
			points2->data.fl[i*2+1] = it->second->y;
		}

		fundamental_matrix = cvCreateMat(3,3,CV_32FC1);

		int fm_count = cvFindFundamentalMat( points1,points2,fundamental_matrix,
			CV_FM_RANSAC, 1, 0.99, status );

		if(fm_count!=0)
		{
			cout<<"\n\nFUNDAMENTAL MATRIX HARRIS:\n\n";
			for(i=0; i<data->Harris_matched.size();i++)
				cout<< (status->data.ptr[i] == 0 ? "0" : "1");
			cout<<"\n";
			//mat.open("FM_Harris.txt");
			//if(!mat.is_open())
			//	cout<<"Fundamental Matrix file not opened";
			for(i=0; i<3; i++)
			{
				for(j=0; j<3; j++)
				{
					data->FM_Harris(i,j)=cvGetReal2D(fundamental_matrix,i,j);
					cout<<data->FM_Harris(i,j)<<"\t";
					//mat<<data->FM_Harris(i,j)<<"\t";
				}
				cout<<"\n";
				//mat<<"\n";
			}
			data->FM_Harris.saveToTextFile("FM_Harris.txt");
			//mat.close();
		}
		else
			cout<<"\nNo Fundamental Matrix found\n";
	}

//inv
	if(data->Harris_matched_inv.size()<8)
		cout<<"WARNING->Less than 8 elements...\n";
	else
	{
		int point_count = data->Harris_matched_inv.size();
		points1 = cvCreateMat(1,point_count,CV_32FC2);
		points2 = cvCreateMat(1,point_count,CV_32FC2);
		status = cvCreateMat(1,point_count,CV_8UC1);

		CMatchedFeatureList::iterator it;
		unsigned int i, j;
		
		/* Fill the points here ... */
		for(i=0, it=data->Harris_matched_inv.begin(); it!=data->Harris_matched_inv.end(); it++,i++)
		{
			points1->data.fl[i*2] = it->first->x;
			points1->data.fl[i*2+1] = it->first->y;
			points2->data.fl[i*2] = it->second->x;
			points2->data.fl[i*2+1] = it->second->y;
		}

		fundamental_matrix = cvCreateMat(3,3,CV_32FC1);

		int fm_count = cvFindFundamentalMat( points1,points2,fundamental_matrix,
			CV_FM_RANSAC, 1, 0.99, status );
		
		if(fm_count!=0)
		{
			cout<<"\n\nFUNDAMENTAL MATRIX HARRIS 'INV':\n\n";
			for(i=0; i<data->Harris_matched_inv.size();i++)
				cout<< status->data.ptr[i] == 0 ? "0" : "1";
			cout<<"\n";
			//mat.open("FM_Harris_inv.txt");
			//if(!mat.is_open())
			//	cout<<"Fundamental Matrix file not opened";
			for(i=0; i<3; i++)
			{
				for(j=0; j<3; j++)
				{
					data->FM_Harris_inv(i,j)=cvGetReal2D(fundamental_matrix,i,j);
					cout<<data->FM_Harris_inv(i,j)<<"\t";
					//mat<<data->FM_Harris_inv(i,j)<<"\t";
				}
				cout<<"\n";
				//mat<<"\n";
			}
			//mat.close();
		}
		else
			cout<<"\nNo Fundamental Matrix 'INV' found\n";
	}

//2ways
	if(data->Harris_matched2ways.size()<8)
		cout<<"WARNING->Less than 8 elements...\n";
	else
	{
		int point_count = data->Harris_matched2ways.size();
		points1 = cvCreateMat(1,point_count,CV_32FC2);
		points2 = cvCreateMat(1,point_count,CV_32FC2);
		status = cvCreateMat(1,point_count,CV_8UC1);

		CMatchedFeatureList::iterator it;
		unsigned int i, j;
		
		/* Fill the points here ... */
		for(i=0, it=data->Harris_matched2ways.begin(); it!=data->Harris_matched2ways.end(); it++,i++)
		{
			points1->data.fl[i*2] = it->first->x;
			points1->data.fl[i*2+1] = it->first->y;
			points2->data.fl[i*2] = it->second->x;
			points2->data.fl[i*2+1] = it->second->y;
		}

		fundamental_matrix = cvCreateMat(3,3,CV_32FC1);

		int fm_count = cvFindFundamentalMat( points1,points2,fundamental_matrix,
			CV_FM_RANSAC, 1, 0.99, status );
		
		if(fm_count!=0)
		{
			cout<<"\n\nFUNDAMENTAL MATRIX HARRIS '2WAYS':\n\n";
			for(i=0; i<data->Harris_matched2ways.size();i++)
				cout<< status->data.ptr[i] == 0 ? "0" : "1";
			cout<<"\n";
			//mat.open("FM_Harris2ways.txt");
			//if(!mat.is_open())
			//	cout<<"Fundamental Matrix file not opened";
			for(i=0; i<3; i++)
			{
				for(j=0; j<3; j++)
				{
					data->FM_Harris2ways(i,j)=cvGetReal2D(fundamental_matrix,i,j);
					cout<<data->FM_Harris2ways(i,j)<<"\t";
					//mat<<data->FM_Harris2ways(i,j)<<"\t";
				}
				cout<<"\n";
				//mat<<"\n";
			}
			//mat.close();
		}
		else
			cout<<"\nNo Fundamental Matrix 'INV' found\n";
	}
/*
// SIFT
	if(data->SIFT_matched.size()<8)
		cout<<"WARNING->Less than 8 elements...\n";
	else
	{
		int point_count = data->SIFT_matched.size();
		points1 = cvCreateMat(1,point_count,CV_32FC2);
		points2 = cvCreateMat(1,point_count,CV_32FC2);
		status = cvCreateMat(1,point_count,CV_8UC1);

		CMatchedFeatureList::iterator it;
		unsigned int i, j;

		// Fill the points here ...
		for(i=0, it=data->SIFT_matched.begin(); it!=data->SIFT_matched.end(); it++,i++)
		{
			points1->data.fl[i*2] = it->first->x;
			points1->data.fl[i*2+1] = it->first->y;
			points2->data.fl[i*2] = it->second->x;
			points2->data.fl[i*2+1] = it->second->y;
		}

		fundamental_matrix = cvCreateMat(3,3,CV_32FC1);
		int fm_count = cvFindFundamentalMat( points1,points2,fundamental_matrix,
											 CV_FM_RANSAC, 1, 0.99, status );

		if(fm_count!=0)
		{
			cout<<"\n\nFUNDAMENTAL MATRIX - SIFT:\n\n";
			for(i=0; i<data->SIFT_matched.size();i++)
				cout<<status->data.ptr[i];
			cout<<"\n";
			mat.open("FM_SIFT.txt");
			if(!mat.is_open())
				cout<<"Fundamental Matrix file not opened";
			for(i=0; i<3; i++)
			{
				for(j=0; j<3; j++)
				{
					data->FM_SIFT(i,j)=cvGetReal2D(fundamental_matrix,i,j);
					cout<<data->FM_SIFT(i,j)<<"\t";
					mat<<data->FM_SIFT(i,j)<<"\t";
				}
				cout<<"\n";
				mat<<"\n";
			}
			mat.close();
		}
		else
			cout<<"\nNo Fundamental Matrix found\n";
	}

// SURF
	if(data->SURF_matched.size()<8)
		cout<<"WARNING->Less than 8 elements...\n";
	else
	{
		int point_count = data->SURF_matched.size();
		points1 = cvCreateMat(1,point_count,CV_32FC2);
		points2 = cvCreateMat(1,point_count,CV_32FC2);
		status = cvCreateMat(1,point_count,CV_8UC1);

		CMatchedFeatureList::iterator it;
		unsigned int i, j;

		// Fill the points here ...
		for(i=0, it=data->SURF_matched.begin(); it!=data->SURF_matched.end(); it++,i++)
		{
			points1->data.fl[i*2] = it->first->x;
			points1->data.fl[i*2+1] = it->first->y;
			points2->data.fl[i*2] = it->second->x;
			points2->data.fl[i*2+1] = it->second->y;
		}

		fundamental_matrix = cvCreateMat(3,3,CV_32FC1);
		int fm_count = cvFindFundamentalMat( points1,points2,fundamental_matrix,
											 CV_FM_RANSAC, 1, 0.99, status );

		if(fm_count!=0)
		{
			cout<<"\n\nFUNDAMENTAL MATRIX SURF:\n\n";
			for(i=0; i<data->SURF_matched.size();i++)
				cout<<status->data.ptr[i];
			cout<<"\n";
			mat.open("FM_SURF.txt");
			if(!mat.is_open())
				cout<<"Fundamental Matrix file not opened";
			for(i=0; i<3; i++)
			{
				for(j=0; j<3; j++)
				{
					data->FM_SURF(i,j)=cvGetReal2D(fundamental_matrix,i,j);
					cout<<data->FM_SURF(i,j)<<"\t";
					mat<<data->FM_SURF(i,j)<<"\t";
				}
				cout<<"\n";
				mat<<"\n";
			}
			mat.close();
		}
		else
			cout<<"\nNo Fundamental Matrix found\n";
	}
*/
	cout<<"\nFundamental Matrix calculation finished\n";
	data->fm_calculated=true;
}

void getFMat_from_txt(data_st *data)
{
	CvMat* points1;
	CvMat* points2;
	CvMat* status;
	CvMat* fundamental_matrix;
	ofstream mat;

	CMatrix A;
	int point_count;
	int i,j;

	// HARRIS
	A.loadFromTextFile("Harris_matches.txt");
	point_count = A.getRowCount();
	if(point_count<8)
		cout<<"WARNING->Less than 8 elements...\n";
	else
	{
		points1 = cvCreateMat(1,point_count,CV_32FC2);
		points2 = cvCreateMat(1,point_count,CV_32FC2);
		status = cvCreateMat(1,point_count,CV_8UC1);
		/* Fill the points here ... */
		for(i=0; i<point_count; i++)
		{
			points1->data.fl[i*2] = A.get_unsafe(i,1);
			points1->data.fl[i*2+1] = A.get_unsafe(i,2);
			points2->data.fl[i*2] = A.get_unsafe(i,4);
			points2->data.fl[i*2+1] = A.get_unsafe(i,5);
		}

		fundamental_matrix = cvCreateMat(3,3,CV_32FC1);

		int fm_count = cvFindFundamentalMat( points1,points2,fundamental_matrix,
			CV_FM_RANSAC, 2, 0.8, status );

		if(fm_count!=0)
		{
			cout<<"\n\nFUNDAMENTAL MATRIX HARRIS:\n\n";
			for(i=0; i<point_count;i++)
				cout<< (status->data.ptr[i]==0 ? "A" : "B");
			cout<<"\n";
			mat.open("FM_Harris.txt");
			if(!mat.is_open())
				cout<<"Fundamental Matrix file not opened";
			for(i=0; i<3; i++)
			{
				for(j=0; j<3; j++)
				{
					data->FM_Harris(i,j)=cvGetReal2D(fundamental_matrix,i,j);
					cout<<data->FM_Harris(i,j)<<"\t";
					mat<<data->FM_Harris(i,j)<<"\t";
				}
				cout<<"\n";
				mat<<"\n";
			}
		}
		else
			cout<<"\nNo Fundamental Matrix found\n";
	}

// SIFT
	A.loadFromTextFile("SIFT_matches.txt");
	point_count = A.getRowCount();

	if(point_count<8)
		cout<<"WARNING->Less than 8 elements...\n";
	else
	{
		points1 = cvCreateMat(1,point_count,CV_32FC2);
		points2 = cvCreateMat(1,point_count,CV_32FC2);
		status = cvCreateMat(1,point_count,CV_8UC1);

		/* Fill the points here ... */
		for(i=0; i<point_count; i++)
		{
			points1->data.fl[i*2] = A.get_unsafe(i,1);
			points1->data.fl[i*2+1] = A.get_unsafe(i,2);
			points2->data.fl[i*2] = A.get_unsafe(i,4);
			points2->data.fl[i*2+1] = A.get_unsafe(i,5);
		}

		fundamental_matrix = cvCreateMat(3,3,CV_32FC1);
		int fm_count = cvFindFundamentalMat( points1,points2,fundamental_matrix,
											 CV_FM_RANSAC, 2, 0.8,  status );

		if(fm_count!=0)
		{
			cout<<"\n\nFUNDAMENTAL MATRIX SIFT:\n\n";
			for(i=0; i<point_count;i++)
				cout<<status->data.ptr[i];
			cout<<"\n";
			mat.open("FM_SIFT.txt");
			for(i=0; i<3; i++)
			{
				for(j=0; j<3; j++)
				{
					data->FM_SIFT(i,j)=cvGetReal2D(fundamental_matrix,i,j);
					cout<<data->FM_SIFT(i,j)<<"\t";
					mat<<data->FM_SIFT(i,j)<<"\t";
				}
				cout<<"\n";
				mat<<"\n";
			}
		}
		else
			cout<<"\nNo Fundamental Matrix found\n";
	}

// SURF
	A.loadFromTextFile("SURF_matches.txt");
	point_count = A.getRowCount();

	if(point_count<8)
		cout<<"WARNING->Less than 8 elements...\n";
	else
	{

		points1 = cvCreateMat(1,point_count,CV_32FC2);
		points2 = cvCreateMat(1,point_count,CV_32FC2);
		status = cvCreateMat(1,point_count,CV_8UC1);

		/* Fill the points here ... */
		for(i=0; i<point_count; i++)
		{
			points1->data.fl[i*2] = A.get_unsafe(i,1);
			points1->data.fl[i*2+1] = A.get_unsafe(i,2);
			points2->data.fl[i*2] = A.get_unsafe(i,4);
			points2->data.fl[i*2+1] = A.get_unsafe(i,5);
		}

		fundamental_matrix = cvCreateMat(3,3,CV_32FC1);
		int fm_count = cvFindFundamentalMat( points1,points2,fundamental_matrix,
											 CV_FM_RANSAC, 2, 0.8,  status );



		if(fm_count!=0)
		{
			cout<<"\n\nFUNDAMENTAL MATRIX SURF:\n\n";
			for(i=0; i<point_count;i++)
				cout<<status->data.ptr[i];
			cout<<"\n";
			mat.open("FM_SURF.txt");
			for(i=0; i<3; i++)
			{
				for(j=0; j<3; j++)
				{
					data->FM_SURF(i,j)=cvGetReal2D(fundamental_matrix,i,j);
					cout<<data->FM_SURF(i,j)<<"\t";
					mat<<data->FM_SURF(i,j)<<"\t";
				}
				cout<<"\n";
				mat<<"\n";
			}
		}
		else
			cout<<"\nNo Fundamental Matrix found\n";
	}
	cout<<"\nFundamental Matrix calculation finished\n";
	data->fm_calculated=true;
}

//void getAffineMat(CMatrixDouble16 first, CMatrixDouble16 second, CvMat affine)
//{
//	//CvMat* warp_mat = cvCreateMat(2,3,CV_32FC1);
//	CvPoint2D32f srcTri[3], dstTri[3];
//	// Compute warp matrix
//	srcTri[0].x = first.get_unsafe(1,1);
//	srcTri[0].y = first.get_unsafe(1,2);
//	srcTri[1].x = first.get_unsafe(1,3);
//	srcTri[1].y = first.get_unsafe(1,4);
//	srcTri[2].x = first.get_unsafe(1,5);
//	srcTri[2].y = first.get_unsafe(1,6);
//
//	dstTri[0].x = second.get_unsafe(1,1);
//	dstTri[0].y = second.get_unsafe(1,2);
//	dstTri[1].x = second.get_unsafe(1,3);
//	dstTri[1].y = second.get_unsafe(1,4);
//	dstTri[2].x = second.get_unsafe(1,5);
//	dstTri[2].y = second.get_unsafe(1,6);
//
//	cvGetAffineTransform( srcTri, dstTri, warp_mat );
//
//}
/*--------------------------------
|	Features & Match STREAM	      |
---------------------------------*/
void FeaturesStream(void* Pdata)
{
	data_st *data=(data_st*) Pdata;	//structure cast

	TestExtractFeatures2(Pdata, "img1.jpg", "f_harris1.txt");
	int k = 2;
	string pictname, fharrisname, fharrismatch, fharrismatch_inv, fharrismatch2ways, FM1, FM2, FM_;
	pictname = format("img%i.jpg",k);

	while(data->pict2.loadFromFile(pictname))
	{
		fharrisname = format("f_harris%i.txt",k);
		data->featsHarris = data->featsHarris2;
		TestExtractFeatures2(Pdata, pictname, fharrisname);
		
		data->matching_type=complete_match;
		matching(data);
		
		getFMat(data);
		FM1 = format("FM_Harris%i.txt",k);
		FM2 = format("FM_Harris_inv%i.txt",k);
		FM_ = format("FM_Harris2ways%i.txt",k);
		data->FM_Harris.saveToTextFile(FM1);
		data->FM_Harris_inv.saveToTextFile(FM2);
		data->FM_Harris2ways.saveToTextFile(FM_);

		data->matching_type=epipolar_match;
		matching(data);
		fharrismatch = format("Harris_matches%i.txt",k);
		fharrismatch_inv = format("Harris_matches_inv%i.txt",k);
		fharrismatch2ways = format("Harris_matches2ways%i.txt",k);
		data->Harris_matched.saveToTextFile(fharrismatch);
		data->Harris_matched_inv.saveToTextFile(fharrismatch_inv);
		data->Harris_matched2ways.saveToTextFile(fharrismatch2ways);

		k++;
		pictname = format("img%i.jpg",k);
		data->featsHarris=data->featsHarris2;
	}
	cout<<"Last Image analysed: " <<(k-1) <<endl;
}

void Tracking(void* Pdata)
{
	data_st *data=(data_st*) Pdata;	//structure cast
	string pict1name, pict2name, featsname;
	int k = 1;
	TestExtractFeatures2(Pdata, "img1.jpg", "harris1.txt");
	data->pict1 = data->pict2;
	while(data->pict2.loadFromFile(format("img%i.jpg",(k+1))))
	{
	featsname = format("harris%i.txt",(k+1));
	data->featsHarris = data->featsHarris2;
	trackFeatures(data->pict1, data->pict2, data->featsHarris2);

//	CFeatureList::iterator it;
//	for(it=data->featsHarris2.begin(); it!=data->featsHarris2.end(); it++)
//	{
//		if((*it)->x < 0)
//			(*it).
//	}
	data->featsHarris2.saveToTextFile(featsname);
	data->pict1 = data->pict2;
	k++;
	}
}



/*--------------------------------
|	  TRANSLATION & ROTATION      |
---------------------------------*/
void getRTMat(void* Pdata)
{
	data_st *data=(data_st*) Pdata;	//structure cast
	CMatrix K, F;
	K=data->intrinsic_matrix;
	//F=data->FM_SURF;
	F.loadFromTextFile("FM_Harris.txt");
	data->RT=(~K)*F*K;
	//RT.multiply(data->FM_Harris,data->intrinsic_matrix);
}
/*--------------------------------
|            MAIN                 |
---------------------------------*/
int main()
{
	//VAR
	char decision;	//to read from keyboard
	data_st data;
	bool output=false;
	data.showvideo=false;
	string response, errormsg;
	mrpt::system::TThreadHandle screen_hd, featuring_hd, matching_hd, fm_hd;
	int *a_enc;
	int act=0; //Action
	int dir=0; //Direction
	int speed=3; //Speed

	data.matched=false;			//this var checks if there are extracted features
	data.features_taken=false, data.matching_done=false, data.fm_calculated=false;
	//string line;
	//fstream f_harris, f_harris_b;
	ofstream matRT;
	CImage Picture;

	std::vector<double> distortion2;
	CMatrix distortion_matrix2(1,4), intrinsic_matrix2(3,3);
	string imgname;

	string MenuJoystick=
	"---------------------------Joystick Controls---------------------------\n\n"
	"|1->Turn to the left		2->Turn to the right			|\n"
	"|3->Head to the middle		4->Head down		3&4->Head up	|\n"
	"|1&2->Go Home			Up&3->Go Home and Dock	1&3->Get Report	|\n"
	"-----------------------------------------------------------------------\n";
	string MenuKeyboard=
	"---------------------------Keyboard Controls---------------------------\n"
	"|MOVEMENT			CAMERA			 PATHS		|\n"
	"|'w'->up			'c'->Middle		'r'->Start	|\n"
	"|'s'->down			'z'->Up			 't'->Stop/Save	|\n"
	"|'a'->left			'x'->Down		 'y'->Show	|\n"
	"|'d'->up						 'u'->Delete	|\n"
	"|'q'->Move left 					 'i'->Run Ford	|\n"
	"|'e'->move Right					 'o'->Run Back	|\n"
	"|'g'->Move to target position						|\n"
	"|									|\n"
	"|EXTRA									|\n"
	"|'-'->-Speed			'+'->+Speed		'v'->Video	|\n"
	"|'h'->Gohome			'j'->menuJY		'r'->menuKB	|\n"
	"|'l'->ShowPosition		'n'->ShowEnc		'k'->Get Report	|\n"
	"|'f'->Ext Features		'b'->Match Feat		'*'->FMatrix	|\n"
	"|				'.'->EXIT				|\n"
	"-----------------------------------------------------------------------\n\n";

	//Code START here
	try
	{
		//initialization of Rovio Values (Ip, user and password)
		data.Robot.Initialize(errormsg);

		if(errormsg.empty())
		{
			cout<<"What do you want from the robot?\n\n";
			cout<<MenuKeyboard<<endl;
			//cout<<MenuJoystick<<endl;

			//Calling Joystick in other thread
			//createThread(JoystickControl,&data);

			do{
				decision=mrpt::system::os::getch();
				switch(decision)
				{
				//Movement control
				case 'w':
					act=18;dir=1;break;
				case 's':
					act=18;dir=2;break;
				case 'q':
					act=18;dir=3;break;
				case 'e':
					act=18;dir=4;break;
				case 'a':
					act=18;dir=5;break;
				case 'd':
					act=18;dir=6;break;
				//camera head movement
				case 'z':
					act=18;dir=11;break;
				case 'x':
					act=18;dir=12;break;
				case 'c':
					act=18;dir=13;break;
				//Paths
	 			case 'r':	//start
					act=2;
					cout << "Recording Path" << endl; break;
				case 't':	//stop and save path
					act=4;
					cout << "Path Saved" << endl; break;
				case 'y':	//GetPath
					act=6;
					cout << "Getting saved Paths" << endl;
					output=true; break;
				case 'u':	//delete path
					act=5;
					cout << "Path deleted" << endl; break;
				case 'i':	//fordward
					act=7;
					cout << "Running path Fordward mode" << endl; break;
				case 'o':	//backward
					act=8;
					cout << "Running path Backward mode" << endl; break;
				//Speed control
				case '-':
					act=0;
					if (speed<=7)
						speed+=2;
					cout << "Speed: "<< speed <<endl;
					break;
				case '+':
					act=0;
					if(speed>=3)
						speed-=2;
					cout << "Speed: "<< speed <<endl;
					break;
				//Camera On/Off
				case 'v':
					act=0;
					if(!data.showvideo)	//open the window
					{
						cout<<"VIDEO ON"<<endl;
						data.showvideo=!data.showvideo;
						screen_hd=createThread(videowindow, &data);
					}
					else //close the window
					{
						cout<<"VIDEO OFF"<<endl;
						data.showvideo=!data.showvideo; // this kill the thread
						joinThread(screen_hd); //Wait till thread finish
						win.~CDisplayWindow();
					}
					break;
				case 'h':
					cout << "GOING HOME" << endl;
					act=13;break;
				case 'j':
					act=0;
					cout<<MenuJoystick<<endl;break;
				case 'm':
					act=0;
					cout<<MenuKeyboard<<endl;break;
				case 'k':
					act=1; output=true;
					cout <<"--------------------------REPORT------------------------" << endl;
					break;
				case 'l': //Print the position
					act=0; output=false;
					showPosition(data);
					break;
				case 'g': //Go to Target Position
					goThere(data, response, errormsg);
					break;
				case 'n': //Print the encoder's values.
					act=0; output=false;
					a_enc=data.Robot.getEncoders();
					print_Encoders(a_enc);
					break;
				case 'f': //Take Features
					act=0; output=false;
					data.features_taken=false;	//this var checks if the function TestExtractFeatures has finished
					featuring_hd=createThread(TestExtractFeatures, &data);
					break;
				case 'b': //Match Features without epipolar restriction
					act=0; output=false;
					data.matching_done=false;	//this var checks if the function TestExtractFeatures has finished
					data.matching_type=complete_match;
					matching_hd=createThread(matching, &data);
					break;
				case '1': //Change the matching to complete matching, get all the correspondences using epipolar restriction
					act=0; output=false;
					data.matching_done=false;	//this var checks if the function TestExtractFeatures has finished
					data.matching_type=epipolar_match;
					matching_hd=createThread(matching, &data);
					break;
				case '*': //Calculate Fundamental Matrix
					act=0; output=false;
					data.fm_calculated=false;	//this var checks if the function TestExtractFeatures has finished
					fm_hd=createThread(getFMat, &data);
					break;
				case '/': //Calculate Fundamental Matrix
					act=0; output=false;
					data.fm_calculated=false;	//this var checks if the function TestExtractFeatures has finished
					fm_hd=createThread(getFMat_from_txt, &data);
					break;
				case '2': //Calculate the Translation & Rotation Matrix for the camera
					act=0; output=false;
					getRTMat(&data);
					matRT.open("RT.txt");
					cout<<"\nRT-Matrix\n\n";
					cout<<data.RT<< endl;
					matRT << data.RT.inMatlabFormat() << endl;
					matRT.close();
					break;
				case '3': //Get the camera's intrinsic matrix
					act=0; output=false;
					data.intrinsic_matrix.loadFromTextFile("intrinsic_matrix.txt");
					cout<<data.intrinsic_matrix;
					break;
				case 'p': //Take Picture and show it in a window
					act=0; output=false;
					data.Robot.captureImageRect(Picture);
					win.showImage(Picture);
					break;
				case '9': //Extract features from a stream and match them 
					act=0; output=false;
					FeaturesStream(&data);
					break;
				case '8': //Track features
					act=0; output=false;
					Tracking(&data);
					break;
				case '7': //Rectify images
					intrinsic_matrix2.loadFromTextFile("intrinsic_matrix.txt");
					distortion_matrix2.loadFromTextFile("distortion_matrix.txt");
					distortion2.resize(4);
					distortion2[0] = distortion_matrix2.get_unsafe(0,0);
					distortion2[1] = distortion_matrix2.get_unsafe(0,1);
					distortion2[2] = distortion_matrix2.get_unsafe(0,2);
					distortion2[3] = distortion_matrix2.get_unsafe(0,3);
					imgname="1.jpg";
					for(int i=1;Picture.loadFromFile(imgname);i++)
					{
						Picture.rectifyImageInPlace(intrinsic_matrix2, distortion2);
						Picture.saveToFile(imgname);
						imgname=format("%i.jpg",i);
					}
					break;
				default:
					act=0;	//No action is executed
				}//end switch

//EXECUTION
				if (act!=0)
				{
					data.Robot.Moving(act,dir,speed, response, errormsg);
					//a_enc=data.Robot.getEncoders();
					//print_Encoders(a_enc);
				}

				if(output)
				{
					cout << "RESPONSE->" << response <<endl;
					output=false;
				}
				if (!errormsg.empty()){
					cout <<"---------------------------------------------------" << endl;
					cout << "ERROR->" <<errormsg <<endl;
				}

				if(data.features_taken)	//End features thread
					joinThread(featuring_hd);
				if(data.matching_done)	//End matching thread
					joinThread(matching_hd);
				if(data.fm_calculated)	//End getFM thread
					joinThread(fm_hd);

				mrpt::system::sleep(10);
			}while(decision!='.');
		}//end if
		else
			mrpt::system::pause();
	}//end try
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
		return -1;
	}
	return 0;
}
