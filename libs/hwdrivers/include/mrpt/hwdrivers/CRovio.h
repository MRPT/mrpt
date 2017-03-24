/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CROVIO_H
#define CROVIO_H

#include <mrpt/utils/TCamera.h>
#include <mrpt/hwdrivers/link_pragmas.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/synch/CThreadSafeVariable.h>
#include <mrpt/obs/CObservationImage.h>

#include <mrpt/hwdrivers/CGenericSensor.h>


namespace mrpt
{
	namespace hwdrivers
	{
		/** A class to interface a Rovio robot (manufactured by WowWee).
		  *  Supports: Simple motion commands, video streaming. 
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CRovio
		{
		private:
			mrpt::system::TThreadHandle		m_videoThread;
			bool	m_videothread_must_exit; 
			bool	m_videothread_initialized_done;
			bool	m_videothread_initialized_error;
			bool    m_videothread_finished;

			mrpt::obs::CObservationImagePtr buffer_img;
			mrpt::synch::CCriticalSection buffer_img_cs;


			/** This function takes a frame and waits until getLastImage ask for it, and so on.
			  */
			void thread_video();

			bool send_cmd_action(int act, int speed);

			bool path_management(int act);

			bool path_management(int act, const std::string &path_name);

			bool general_command(int act, std::string &response, std::string &errormsg);


		public:
			struct TOptions
			{
				std::string IP;
				std::string user;
				std::string password;

				mrpt::utils::TCamera cameraParams;		// Mat. cam. preguntar paco

				TOptions();
			} options;

			enum status {idle, driving_home, docking, executing_path, recording_path};

			struct TRovioState{
				status state;
				unsigned int nss;	//Navigation Signal Strength
				unsigned int wss;	//Wifi Signal Strength
			};

			struct TEncoders{
				int left;
				int right;
				int rear;
				TEncoders()
				{
					left = 0;
					right = 0;
					rear = 0;
				}
			}encoders;


			/** Establish conection with Rovio and log in its system: Important, fill out "options" members *BEFORE* calling this method.
			  *  \exception std::runtime On errors
			  */ 
			void initialize(); //string &errormsg_out, std::string url_out="150.214.109.134", std::string user_out="admin", std::string password_out="investigacion");

			/**	move send Rovio the command to move in the specified direcction
			  * \param direction 'f'->forward, 'b'->backward, 'r'->right, 'l'->left
			  * \return False on error
			  */
			bool move(char direction, int speed=5 );

			/** rotate send Rovio the command to rotate in the specified direcction
			  * 'r'->right, 'l'->left
			  * \return False on error
			  */
			bool rotate(char direction, int speed=5 );

			/**  Head positions 
			  * \return False on error
			  */ 
			bool takeHeadUp();
			bool takeHeadMiddle();
			bool takeHeadDown();


			/*  Path commands */ 
			bool pathRecord();
			bool pathRecordAbort();
			bool pathRecordSave(const std::string &path_name);//Repasar const
			bool pathDelete(const std::string &path_name);
			/** Get list of saved paths
			  */ 
			bool pathGetList(std::string &path_list);
			bool pathRunForward();
			bool pathRunBackward();
			bool pathRunStop();
			bool pathRunPause();
			bool pathRename(const std::string &old_name, const std::string &new_name);


			/** goHome(bool dock) drives Rovio in front of charging station if the paremeter dock is set to false, otherwise it also docks
			  * \return False on error
			  */
			bool goHome(bool dock, int speed = 5);

			/** Loads the rovio camera calibration parameters (of leave the default ones if not found) (See CGenericSensor), then call to "loadConfig_sensorSpecific"
			  *  \exception This method throws an exception with a descriptive message if some critical parameter is missing or has an invalid value.
			  */
			void  loadConfig(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string			&section );

			/** This function launchs a thread with the function "thread_video()" which gets frames into a buffer.
			  * After calling this method, images can be obtained with getNextImageSync()
			  * \return False on error
			  * \sa getNextImageSync
			  */
			bool retrieve_video();//como la protejo para que no se llame dos veces??????????????????????????????????????????????

			/** This function stops and joins the thread launched by "retrieve_video()".
			  * \return False on error
			  */
			bool stop_video();

			/** Returns the next frame from Rovio's live video stream, after starting the live streaming with retrieve_video()
			  * \return False on error
			  * \sa retrieve_video, captureImageAsync
			  */
			bool getNextImageSync(mrpt::obs::CObservationImagePtr& lastImage );

			/** Returns a snapshot from Rovio, if rectified is set true, the returned image is rectified with the parameters of intrinsic_matrix and distortion_matrix.
			  * This function works asynchronously and does not need to have enabled the live video streaming.
			  * \return False on error
			  * \sa captureImageSync
			  */
			bool captureImageAsync( mrpt::utils::CImage&out_img, bool recttified);//string pict_name, 

			bool isVideoStreamming() const; //!< Return true if video is streaming correctly \sa retrieve_video


//Rovio State
			/** Returns a TRovioState with internal information of Rovio (State, Navigation Signal Strength, Wifi Signal Strength)
			  * \return False on error
			  */
			bool getRovioState(TRovioState &state);

			/** Returns a TEncoders with information of Rovio encoders (since last read, it seems Rovio is continuously reading with unknown sample time)
			  * \return False on error
			  */
			bool getEncoders(TEncoders &encoders);

			/** Returns the Rovio's pose
			  * \return False on error
			  */
			bool getPosition(mrpt::math::TPose2D &out_pose);



			CRovio();
			virtual ~CRovio();

		};	// End of class

	} // End of namespace

} // End of namespace

#endif
