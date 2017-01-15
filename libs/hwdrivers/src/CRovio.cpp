/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/hwdrivers/CRovio.h>
#include <mrpt/hwdrivers/CFFMPEG_InputStream.h>
#include <mrpt/utils/net_utils.h>
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/obs/CObservationImage.h>

using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils::net;
using namespace std;


CRovio::TOptions::TOptions() :
	IP("150.214.109.134"),
	user("admin"),
	password("investigacion")
	//We could fill the camera matrices here instead of in initialization
{
	double cam_params[] = {
		1650.740234375,		0,						805.33190917968750,
		0,					1640.6497802734375,		675.1627197265625,
		0,					0,						1					};
	cameraParams.intrinsicParams.loadFromArray(cam_params);
	cameraParams.setDistortionParamsFromValues(0.12792350351810455, -0.4786585867404937, 0.011172077618539333, -0.0037264013662934303);
	//cameraParams.focalLengthMeters =
}

/*-------------------------------------------------
                     INITIALIZE
  -----------------------------------------------*/
void CRovio::initialize() //string &errormsg, string url_out, string user_out, string password_out)
{
	string response, errormsg;
	http_get (format("http://%s/rev.cgi?Cmd=nav&action=1",options.IP.c_str()), response, errormsg, 80, options.user, options.password);

	if (!response.empty())
		cout<<"[CRovio::Initialize] Response:\n"<<response<<endl;

	if (!errormsg.empty())
		THROW_EXCEPTION_CUSTOM_MSG1("Error initializing Rovio: %s",errormsg.c_str() );
}

/*-------------------------------------------------
        SEND MOVEMENT COMMAND (Manual Drive)
  -----------------------------------------------*/
bool CRovio::send_cmd_action(int direction, int speed)
{
	string response, errormsg;
	string command = format("http://%s/rev.cgi?Cmd=nav&action=18&drive=%i&speed=%i", options.IP.c_str(), direction, speed);
	http_get (command, response, errormsg, 80, options.user, options.password);
	return  errormsg.empty();
}

/*-------------------------------------------------
                  PATH MANAGEMENT
  -----------------------------------------------*/
bool CRovio::path_management(int act)
{
	string response, errormsg;
	string command;
	command = format("http://%s/rev.cgi?Cmd=nav&action=%i", options.IP.c_str(), act);
	http_get (command, response, errormsg, 80, options.user, options.password);
	return  errormsg.empty();
}

bool CRovio::path_management(int act, const string &path_name)
{
	string response, errormsg;
	string command;
	command = format("http://%s/rev.cgi?Cmd=nav&action=%i&name=%s", options.IP.c_str(), act, path_name.c_str());
	http_get (command, response, errormsg, 80, options.user, options.password);
	return  errormsg.empty();
}

/*-------------------------------------------------
                  GENERAL COMMAND
  -----------------------------------------------*/
bool CRovio::general_command(int act, string &response, string &errormsg)
{
	string command;
	command = format("http://%s/rev.cgi?Cmd=nav&action=%i", options.IP.c_str(), act);
	http_get (command, response, errormsg, 80, options.user, options.password);
	return  errormsg.empty();
}


/*-------------------------------------------------
                    MOVE ROBOT
  -----------------------------------------------*/
bool CRovio::move( char direction, int speed )
{
	switch(direction)
	{
		case 'f':	//Forward
			return send_cmd_action(1,speed);
		case 'b':	//Backward
			return send_cmd_action(2,speed);
		case 'l':	//Left
			return send_cmd_action(3,speed);
		case 'r':	//Right
			return send_cmd_action(4,speed);
		default:
			cout << "Error in parameter of move()";
			return false;
	}
}

bool CRovio::rotate( char direction, int speed )
{
	switch(direction)
	{
		case 'l':	//Left
			return send_cmd_action(5,speed);
		case 'r':	//Right
			return send_cmd_action(6,speed);
		default:
			cout << "Error in parameter of rotate()";
			return false;
	}

}
/*-------------------------------------------------
                    MOVE HEAD
  -----------------------------------------------*/
bool CRovio::takeHeadUp()
{
	return send_cmd_action(11,5);
}
bool CRovio::takeHeadMiddle()
{
	return send_cmd_action(13,5);
}
bool CRovio::takeHeadDown()
{
	return send_cmd_action(12,5);
}

/*-------------------------------------------------
                  PATH COMMAND
  -----------------------------------------------*/
bool CRovio::pathRecord()
{
	return path_management(2);
}
bool CRovio::pathRecordAbort()
{
	return path_management(3);
}
bool CRovio::pathRecordSave(const string &path_name)
{
	return path_management(4,path_name);
}
bool CRovio::pathDelete(const string &path_name)
{
	return path_management(5,path_name);
}
bool CRovio::pathGetList(string &path_list)
{
	string error;
	general_command(6, path_list, error);
	return error.empty();
}
bool CRovio::pathRunForward()
{
	return path_management(7);
}
bool CRovio::pathRunBackward()
{
	return path_management(8);
}
bool CRovio::pathRunStop()
{
	return path_management(9);
}
bool CRovio::pathRunPause()
{
	return path_management(10);
}
bool CRovio::pathRename(const string &current_name, const string &new_name)
{
	string response, errormsg;
	string command = format("http://%s/rev.cgi?Cmd=nav&action=11&name=%s&newname=%s", options.IP.c_str(), current_name.c_str(), new_name.c_str());
	http_get (command, response, errormsg, 80, options.user, options.password);
	return errormsg.empty();
}

/*-------------------------------------------------
                     GO HOME
  -----------------------------------------------*/
bool CRovio::goHome(bool dock,int speed )
{
	if(dock)
		return send_cmd_action(13, speed );
	else
		return send_cmd_action(12, speed );
}

/*-------------------------------------------------
                  CAMERA FUNCTIONS
  -----------------------------------------------*/
void CRovio::loadConfig(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string			&section )
{
	options.cameraParams.loadFromConfigFile(section,configSource);

	// Any other params??
}
/*-------------------------------------------------
                  VIDEO STREAMING
  -----------------------------------------------*/
void CRovio::thread_video()	//This function takes a frame and waits until getLastImage() ask for it, and so on.
{
	try
	{
		// obj -> this
		CFFMPEG_InputStream	in_video;
		string video_url=format("rtsp://%s/webcam", options.IP.c_str() );

		const bool open_ok = in_video.openURL(video_url, false /*grayscale*/, false /* verbose */ );

		m_videothread_initialized_error = !open_ok;
		m_videothread_initialized_done = true;

		if (m_videothread_initialized_error)
		{
			std::cerr << "[CRovio] Error opening video stream: " << video_url << std::endl;
			return; // Error!
		}

		while(!m_videothread_must_exit)
		{
			CObservationImagePtr obs = CObservationImage::Create();

			if (in_video.retrieveFrame(obs->image) )
			{
				obs->cameraParams = options.cameraParams;

				//Critical section
				{
					mrpt::synch::CCriticalSectionLocker cs( &this->buffer_img_cs );
					this->buffer_img = obs;
					//cout<<"[CRovio::threadVideo] Image grabbed\n";
				}
			}
			else
			{
				//obs.clear();	//If no image was copied, destroy the thisect.
				cout<<"[CRovio::thread_video] Warning: the program doesn't receive any image\n";
			}
			mrpt::system::sleep(10);
		}//end while

		in_video.close();

		m_videothread_finished = true;
	}
	catch(std::exception &e)//que hace eactamente esto?
	{
		m_videothread_initialized_done = true;  // Just in case...
		m_videothread_finished = true;
		cout<<"Error in thread_video thread";
		cerr << e.what() << endl;
	}
	catch(...)
	{
		m_videothread_initialized_done = true;  // Just in case...
		m_videothread_finished = true;
		cout<<"Error in thread_video thread";
	}
}

bool CRovio::retrieve_video()
{
	if(m_videoThread.isClear())
	{
		m_videothread_initialized_done  = false;
		m_videothread_initialized_error = false;
		m_videothread_must_exit         = false;
		m_videothread_finished          = false;

		m_videoThread = mrpt::system::createThreadFromObjectMethod(this,&mrpt::hwdrivers::CRovio::thread_video);

		while (!m_videothread_initialized_done) {
			mrpt::system::sleep(10);
		}

		// Ok or error?
		if (m_videothread_initialized_error)
		{
			m_videoThread.clear();
			return false;
		}
		else return true; // Grabbing video
	}
	else
	return true;
}

bool CRovio::isVideoStreamming() const
{
	return (!m_videoThread.isClear() && !m_videothread_finished);
}

bool CRovio::stop_video()
{
	bool was_already_stop = true;
	m_videothread_must_exit = true;
	if (isVideoStreamming())
	{
		joinThread(m_videoThread);
		was_already_stop = false;
	}
	m_videoThread.clear();

	return !was_already_stop;
}

bool CRovio::getNextImageSync(CObservationImagePtr& lastImage )		//This function grabbes the images taken by thread_video
{
	if (!isVideoStreamming())
		return false;

	{
		mrpt::synch::CCriticalSectionLocker cs( &buffer_img_cs );
		if(!buffer_img)
			return false;

		lastImage = buffer_img;
	}

	return true;
}

/*-------------------------------------------------
                  CAPTURE PICTURE
  -----------------------------------------------*/
bool CRovio::captureImageAsync( CImage & picture, bool rectified)
{
	try
	{
		vector_byte resp;
		string errormsg;
		string MF=format("http://%s/Jpeg/CamImg[0000].jpg",options.IP.c_str());
		http_get (MF, resp, errormsg, 80, options.user, options.password);

		CMemoryStream stream( &resp[0],  resp.size()  );
		picture.loadFromStreamAsJPEG(stream);
		if( rectified )//Comprobar que las matrices existen y son correctas********************
			picture.rectifyImageInPlace(options.cameraParams);
		//picture.saveToFile("0000.jpg");
		//cout<<"Response:\n"<<response<<endl;
		return true;
	}
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
		return false;
	}
}

/*-------------------------------------------------
                     STATE
  -----------------------------------------------*/
bool CRovio::getRovioState(CRovio::TRovioState &status)
{
	MRPT_UNUSED_PARAM(status);
	size_t x_pos, /*y_pos, theta_pos,*/ lenght;
	string x_value, response, errormsg;
	mrpt::math::TPose2D pose;
	general_command(1, response, errormsg);		//Get report from Rovio to response

	//Getting x value
	x_pos=response.find("x=");
	x_value=response.substr((x_pos+2),8);
	lenght=x_value.find('|');
	x_value=x_value.substr(0,lenght);
	char* x_char=new char [lenght];
	strcpy(x_char, x_value.c_str());
	pose.x=atof(x_char);

	string error;
	string state;
	general_command(1, state, error);

	return error.empty();
}

/*-------------------------------------------------
                     GET ENCODERS
  -----------------------------------------------*/
long convertToLong(char *sLong)
{
	char * result=strpbrk(sLong,"-0123456789ABCDEF");
	char * stop;
	return strtol(result,&stop,16);
}

bool CRovio::getEncoders(CRovio::TEncoders &encoders)//Revisar esto
{
	MRPT_UNUSED_PARAM(encoders);
	string resp, error, field;
	//string field_name[12]={"Packet length","Not Used","Left Wheel:Dir Rotation","Left Wheel:Ticks","Right Wheel:Dir Rotation","Right Wheel:Ticks","Rear Wheel:Dir Rotation","Rear Wheel:Ticks","Not used","Head Position","Batery","Config Status"};
	size_t length;
	int *a_enc = new int[11];	//12 encoder's fields
	long l_value;

	general_command(20, resp, error);	//get Encoders string to resp
	if(error.empty())
	{
		size_t pos=(resp.find("responses =")+12);
		for(int i=0;i<=11;i++)
		{
			if ( (i==3)||(i==5)||(i==7) )
				length = 4;
			else
				length = 2;

			field = resp.substr(pos,length);
			pos+=length;

			/*---------- String to binary conv------------------*/
			char* cstr = new char [field.size()+1];
			strcpy (cstr, field.c_str());
			l_value = convertToLong(cstr);

			if ( (i==2)||(i==4)||(i==6) )	//just interested in bit(2)-> "0000X0"
				l_value=( (l_value>>1) & 0x1); //This extracts the last but one bit of l_value which sustitutes l_value
			a_enc[i] = l_value;
		}
		//Upload the encoders value
		if(a_enc[2])
			this->encoders.left += a_enc[3];	//Esta esto bien asi? o deberia usar encoders como parametro de entrada
		else
			this->encoders.left -= a_enc[3];
		if(a_enc[4])
			this->encoders.left += a_enc[5];
		else
			this->encoders.left -= a_enc[5];
		if(a_enc[6])
			this->encoders.left += a_enc[7];
		else
			this->encoders.left -= a_enc[7];

		return true;

	}else{
		cout <<"\n---------------------------------------------------" << endl;
		cout << "ERROR->" <<error <<endl;

		return false;
	}
}

/*-------------------------------------------------
        GET POSITION WITH NORTHSTAR SYSTEM
  -----------------------------------------------*/
bool CRovio::getPosition(mrpt::math::TPose2D &pose)
{
	size_t x_pos, y_pos, theta_pos, lenght;
	string x_value, y_value, theta_value, response, errormsg;
	general_command(1, response, errormsg);		//Get report from Rovio to response

	//Getting x value
	x_pos=response.find("x=");
	x_value=response.substr((x_pos+2),8);
	lenght=x_value.find('|');
	x_value=x_value.substr(0,lenght);
	char* x_char=new char [lenght];
	strcpy(x_char, x_value.c_str());
	pose.x=atof(x_char);

	//Getting y value
	y_pos=response.find("y=");
	y_value=response.substr((y_pos+2),8);
	lenght=y_value.find('|');
	y_value=y_value.substr(0,lenght);
	char* y_char=new char [lenght];
	strcpy(y_char, y_value.c_str());
	pose.y=atof(y_char);

	//Getting theta value
	theta_pos=response.find("theta=");
	theta_value=response.substr((theta_pos+6),8);
	lenght=theta_value.find('|');
	theta_value=theta_value.substr(0,lenght);
	char* theta_char=new char [lenght];
	strcpy(theta_char, theta_value.c_str());
	pose.phi=atof(theta_char);

	return errormsg.empty();
}

CRovio::CRovio()
{

}

CRovio::~CRovio()
{
	if (isVideoStreamming())
		stop_video();
}
