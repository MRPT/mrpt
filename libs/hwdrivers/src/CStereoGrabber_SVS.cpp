/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/hwdrivers/CStereoGrabber_SVS.h>


// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 

#if MRPT_HAS_SVS
#	include <svsclass.h>
#	include <dcs.h>
#endif

using namespace std;
using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::hwdrivers;

/*-------------------------------------------------------------
					Constructor
 -------------------------------------------------------------*/
CStereoGrabber_SVS::CStereoGrabber_SVS( int cameraIndex, const TCaptureOptions_SVS &options ) :
        m_bInitialized(false),
        m_videoObject(NULL),
        m_stereoImage(NULL),
        m_disparityParams(NULL),
        m_resolutionX( options.frame_width ),
        m_resolutionY( options.frame_height ),
        m_procesOnChip(options.m_procesOnChip),
        m_calDisparity(options.m_calDisparity),
        m_options( options )
{
#if MRPT_HAS_SVS

    // get the svsVideoImages object from the currently loaded camera interface
    m_videoObject =static_cast<svsVideoImages*>( getVideoObject());
    cout<<"Using live images:"<<endl;
    cout<<"svsVideoIdent"<<endl;

    // Open the stereo device
    bool ret;
    ret = static_cast<svsVideoImages*>(m_videoObject)->Open();
    if (ret)
    {
        cout<<" stereo device Opened"<<endl;

        static_cast<svsVideoImages*>(m_videoObject)->SetSize(m_resolutionX,m_resolutionY); // width x height image


        m_disparityParams = static_cast<svsVideoImages*>(m_videoObject)->GetDP();
        static_cast<svsVideoImages*>(m_videoObject)->SetNDisp(m_options.m_NDisp);	// 32 disparities
        static_cast<svsVideoImages*>(m_videoObject)->SetCorrsize(m_options.m_Corrsize); // correlation window size
        static_cast<svsVideoImages*>(m_videoObject)->SetLR(m_options.m_LR);	// no left-right check, not available
        static_cast<svsVideoImages*>(m_videoObject)->SetThresh(m_options.m_Thresh);	// texture filter
        static_cast<svsVideoImages*>(m_videoObject)->SetUnique(m_options.m_Unique);	// uniqueness filter
        static_cast<svsVideoImages*>(m_videoObject)->SetHoropter(m_options.m_Horopter);	// horopter offset

        if(!(static_cast<svsVideoImages*>(m_videoObject)->SetExposure(0,0,true,true)))
        {
            cout<<"Can't set Auto exposure"<<endl;
        }else
        {
            cout<<"Autoexposure set to 0 0"<<endl;
        }

        /*	videoObject->SetBrightness(true, 0);
           videoObject->SetAutoExpParams(0.0, -2.0);
       */
        ///videoObject->SetGamma(); search for auto gamma ?

        static_cast<svsVideoImages*>(m_videoObject)->SetRate(m_options.framerate);
        static_cast<svsVideoImages*>(m_videoObject)->SetSpeckleSize(m_options.m_SpeckleSize); //TODO add in config

        //TODO call CheckParam
        if(static_cast<svsVideoImages*>(m_videoObject)->CheckParams())
        {
            cout<<"Params OK !"<<endl;
            m_initialized =true;
            m_status = true;
            bool ret = static_cast<svsVideoImages*>(m_videoObject)->Start();
            if (ret){

                cout<<" Start Continuous mode"<<endl;

                // NOTE: to do rectification, we have to turn it on...
                // Here we optionally set up acquisition to rectify the image

                ret = static_cast<svsVideoImages*>(m_videoObject)->SetRect(true);
                if (ret){
                    cout<<"Images will be rectified"<<endl;
                }else{
                    cout<<"Can't set rectification"<<endl;
                }
                if(m_procesOnChip)
                {
                    // NOTE: for STOC device, turn on stereo processing on-chip
                    if (static_cast<svsVideoImages*>(m_videoObject) && static_cast<svsVideoImages*>(m_videoObject)->is_proc_capable) // can we process on-camera?
                    {
                        static_cast<svsVideoImages*>(m_videoObject)->SetProcMode(PROC_MODE_DISPARITY);
                        cout<<"Setting STOC disparity mode"<<endl;
                    }
                }
                else
                {
                    if (m_processObject) m_processObject = new svsStereoProcess();
                    // NOTE: for STOC device, turn off stereo processing on-chip
                    if (static_cast<svsVideoImages*>(m_videoObject) && static_cast<svsVideoImages*>(m_videoObject)->is_proc_capable) // can we process on-camera?
                    {
                        static_cast<svsVideoImages*>(m_videoObject)->SetProcMode(PROC_MODE_OFF);
                        cout<<"Setting STOC stereo mode"<<endl;
                    }
                }
            }
            else
            {
                cout<<"Can't start continuous capture"<<endl;
                m_status = false;

            }




        }
        else
        {
            m_initialized =false;
            m_status = false;
            cout<<"Params Unconsistents !"<<endl;
        }

    }
    else

        //TODO essayer de faire un close...
        cout<<"Can't open stereo device"<<endl;

    m_status = false;

#else
	MRPT_UNUSED_PARAM(cameraIndex);
	THROW_EXCEPTION("This class requires MRPT built with Videre SVS library.")
#endif
}

/*-------------------------------------------------------------
					Destructor
 -------------------------------------------------------------*/
CStereoGrabber_SVS::~CStereoGrabber_SVS()
{
#if MRPT_HAS_SVS
    static_cast<svsVideoImages*>(m_videoObject)->Close();
#endif // No need to raise an exception on "#else" since it's already raised upon construction.
}

/*-------------------------------------------------------------
					get the image
 -------------------------------------------------------------*/
bool  CStereoGrabber_SVS::getStereoObservation( mrpt::obs::CObservationStereoImages &out_observation )
{
#if MRPT_HAS_SVS
    if ( (m_stereoImage = static_cast<svsVideoImages*>(m_videoObject)->GetImage(500)) &&  static_cast<svsStereoImage*>(m_stereoImage)->haveImages ) // 500 ms timeout //TODO adjust timeout with framerate
    {

                //get disparity params
                m_disparityParams = static_cast<svsVideoImages*>(m_videoObject)->GetDP();

                const size_t sizeOfMat = m_resolutionX * m_resolutionY;

                IplImage* ImageLeft =  cvCreateImageHeader(cvSize(m_resolutionX,m_resolutionY),IPL_DEPTH_8U,1);

                ImageLeft->widthStep=ImageLeft->width; // JL: The next line assumes this
                ImageLeft->imageData =(char*) static_cast<svsStereoImage*>(m_stereoImage)->Left();

                if(m_procesOnChip)
                {
                    IplImage* ImageDisparity = cvCreateImage(cvSize(m_resolutionX,m_resolutionY),IPL_DEPTH_8U,1);

                    unsigned char *ptrOutDisp;
                    short int *ptrDisp;

                    ptrDisp = static_cast<svsStereoImage*>(m_stereoImage)->Disparity();
                    ptrOutDisp = (unsigned char*) ImageDisparity->imageData;

                    ASSERT_(ImageDisparity->widthStep==ImageDisparity->width);  // JL: The code below assumes image_width == widthStep

                    for(int pix = 0;pix<sizeOfMat;pix++,ptrOutDisp++,ptrDisp++ )
                    {
                        if(*(ptrDisp)>0)
                        *(ptrOutDisp) =  (unsigned char)((*(ptrDisp)>>2)&0x00FF);
                        else
                            *(ptrOutDisp) = 0;
                    }

                    // Create the object to be return (it will have a fresh timestamp if it's created now and here):
                    CObservationStereoImages ret_obj(
                                            cvCloneImage( ImageLeft ),  // Create a new IplImage* which will be owned by the observation object.
                                            NULL /*has no right*/,
                                            ImageDisparity,
                                            true /* own the memory, so we don't have to free it here */);

                    out_observation.swap(ret_obj); // Send as output (faster than a "=").
                }
                else if (m_calDisparity)
                {
                    static_cast<svsStereoProcess*>(m_processObject)->CalcStereo(static_cast<svsStereoImage*>(m_stereoImage));
                    IplImage* ImageDisparity = cvCreateImage(cvSize(m_resolutionX,m_resolutionY),IPL_DEPTH_8U,1);

                    unsigned char *ptrOutDisp;
                    short int *ptrDisp;

                    ptrDisp = static_cast<svsStereoImage*>(m_stereoImage)->Disparity();
                    ptrOutDisp = (unsigned char*) ImageDisparity->imageData;

                    ASSERT_(ImageDisparity->widthStep==ImageDisparity->width);  // JL: The code below assumes image_width == widthStep

                    for(int pix = 0;pix<sizeOfMat;pix++,ptrOutDisp++,ptrDisp++ )
                    {
                        if(*(ptrDisp)>0)
                        *(ptrOutDisp) =  (unsigned char)((*(ptrDisp)>>2)&0x00FF);
                        else
                            *(ptrOutDisp) = 0;
                    }

                    // Create the object to be return (it will have a fresh timestamp if it's created now and here):
                    CObservationStereoImages ret_obj(
                                            cvCloneImage( ImageLeft ),  // Create a new IplImage* which will be owned by the observation object.
                                            NULL /*has no right*/,
                                            ImageDisparity,
                                            true /* own the memory, so we don't have to free it here */);

                    out_observation.swap(ret_obj); // Send as output (faster than a "=").

                }
                else
                {

                    IplImage* ImageRight =  cvCreateImageHeader(cvSize(m_resolutionX,m_resolutionY),IPL_DEPTH_8U,1);
                    ImageRight->widthStep=ImageRight->width; // JL: The next line assumes this
                    ImageRight->imageData =(char*) static_cast<svsStereoImage*>(m_stereoImage)->Right();

                    // Create the object to be return (it will have a fresh timestamp if it's created now and here):
                    CObservationStereoImages ret_obj(
                                            cvCloneImage( ImageLeft ),  // Create a new IplImage* which will be owned by the observation object.
                                            cvCloneImage( ImageRight ),
                                            NULL /*has no disparity*/,
                                            true /* own the memory, so we don't have to free it here */);

                    out_observation.swap(ret_obj); // Send as output (faster than a "=").

                }



                // cvReleaseImage(&Image Disparity); // No need anymore to release images...
    return true;
    }

    return false;
     // All ok
#else
	MRPT_UNUSED_PARAM(out_observation);
	// No need to raise an exception on "#else" since it's already raised upon construction.
	return false;	// This shouldn't actually be never reached, just to please the compiler.
#endif
}

/*-------------------------------------------------------------
                        TCaptureOptions_bumblebee Constructor
 -------------------------------------------------------------*/
TCaptureOptions_SVS::TCaptureOptions_SVS(int _frame_width, int _frame_height , double _framerate, int _NDisp,
                                         int _Corrsize, int _LR , int _Thresh, int _Unique, int _Horopter,int _SpeckleSize,bool _procesOnChip,bool _calDisparity)
{
    frame_width     = _frame_width;
    frame_height    = _frame_height;
    framerate       = _framerate;
    m_NDisp         = _NDisp;	// 32 disparities
    m_Corrsize      =_Corrsize; // correlation window size
    m_LR            =_LR;// no left-right check, not available
    m_Thresh        =_Thresh;	// texture filter
    m_Unique        =_Unique;	// uniqueness filter
    m_Horopter      =_Horopter;
    m_SpeckleSize   =_SpeckleSize;
    m_procesOnChip  =_procesOnChip;
    m_calDisparity  =_calDisparity;

}

