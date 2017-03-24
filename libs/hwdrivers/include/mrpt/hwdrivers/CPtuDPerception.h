/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CPtuDPerception_H
#define CPtuDPerception_H

#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/hwdrivers/CPtuBase.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** This class implements initialization and comunication methods to
		  * control a Pan and Tilt Unit model PTU-46-17.5, working in radians .
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CPtuDPerception : public CPtuBase
		{

		public:

			/** Default constructor */

			CPtuDPerception() {};

			/** Destructor */

			virtual ~CPtuDPerception(){ close(); }

		/*************************** Commands ***************************/

		public:

			/** Search limit forward */

			virtual bool rangeMeasure();

			/** Specification of positions in absolute terms */

			virtual bool moveToAbsPos(char axis,double nRad);

			/** Query position in absolute terms */

			virtual bool absPosQ(char axis,double &nRad);

			/** Specify desired axis position as an offset from the current position. \n
			*	This method recives the number of radians to move.
			*	\code
			*	Example of use:
			*		TT-500 *
			*		A *
			*		TO * Current Tilt position is -500
			*		TO500 *
			*		A *
			*		TT * Current Pan position is 1000
			*	\endcode
			*/

			virtual bool moveToOffPos(char axis,double nRad);

			/** Query position in relative terms */

			virtual bool offPosQ(char axis,double &nRad);

			/** Query max movement limit of a axis in absolute terms */

			virtual bool maxPosQ(char axis,double &nRad);

			/** Query min movement limit of a axis in absolute terms */

			virtual bool minPosQ(char axis,double &nRad);

			/** Query if exist movement limits */

			virtual bool enableLimitsQ(bool &enable); // Query if exist some limit

			/** Enable/Disable movement limits */

			virtual bool enableLimits(bool set);

			/** With I mode (default) instructs pan-tilt unit to immediately
			*	execute positional commands. \n
			*	In S mode instructs pan-tilt unit to execute positional commands
			*	only when an Await Position Command Completion command is executed
			*	or when put into Immediate Execution Mode. \n
			*	\code
			*	Example of use of S mode:
			*		DR *
			*		S *
			*		PP1500 *
			*		TP-900 *
			*		PP * Current Pan position is 0
			*		TP * Current Tilt position is 0
			*		A *
			*		PP * Current Pan position is 1500
			*		TP * Current Tilt position is -900
			*	\endcode
			*/

			virtual bool inmediateExecution(bool set);

			/** Wait the finish of the last position command to
			*	continue accept commands
			*/

			virtual bool aWait(void);

			/** Inmediately stop all */

			virtual bool haltAll();

			/** Inmediately stop */

			virtual bool halt(char axis);

			/** Specification of turn speed */

			virtual bool  speed(char axis,double radSec);

			/** Query turn speed */

			virtual bool  speedQ(char axis,double &radSec);

			/** Specification (de/a)celeration in turn */

			virtual bool  aceleration(char axis,double radSec2);

			/** Query (de/a)celeration in turn */

			virtual bool  acelerationQ(char axis,double &radSec2);

			/** Specification of velocity to which start and finish
			*	the (de/a)celeration
			*/

			virtual bool  baseSpeed(char axis,double radSec);

			/** Query velocity to which start and finish
			*	the (de/a)celeration
			*/

			virtual bool  baseSpeedQ(char axis,double &radSec);

			/** Specification of velocity upper limit */

			virtual bool upperSpeed(char axis,double radSec);

			/** Query velocity upper limit */

			virtual bool upperSpeedQ(char axis,double &radSec);

			/** Specification of velocity lower limit */

			virtual bool lowerSpeed(char axis,double radSec);

			/** Query velocity lower limit */

			virtual bool lowerSpeedQ(char axis,double &radSec);

			/** Reset PTU to initial state */

			virtual bool reset(void);

			/** Save or restart default values */

			virtual bool save(void);

			/** Restore default values */

			virtual bool restoreDefaults(void);

			/** Restore factory default values */

			virtual bool restoreFactoryDefaults(void);

			/** Version and CopyRights */

			virtual bool version(char * nVersion);

			/** Number of version */

			virtual void nversion(double &nVersion);

			/** Query power mode */

			virtual bool powerModeQ(bool transit,char &mode);

			/** Specification of power mode */

			virtual bool powerMode(bool transit,char mode);

			/** Check if ptu is moving */

			virtual double status(double &rad){
				MRPT_UNUSED_PARAM(rad);
				return 1;
			}

			/** Set limits of movement */

			virtual bool setLimits(char axis, double &l, double &u);

			/* Change motion direction */

			virtual bool changeMotionDir();


		/**************************** State Queries ********************/

			/** Check errors, returns 0 if there are not errors or error code in otherwise
			*	Error codes:
			*	\code
			*	1: Com error
			*	2: Time out error
			*	3: Init error
			*	4: Pan tilt hit error
			*	5: Pan hit error
			*	6: Tilt hit error
			*	7: Max limit error
			*	8: Min limit error
			*	9: Out of range
			*	10: Illegal command error
			*	11: Unexpected error
			*   \endcode
			**/

			virtual int checkErrors();

			inline bool noError() { return nError==1; }
			inline bool comError() { return (nError % CPtuDPerception::ComError)==0; }
			inline bool timeoutError() { return (nError % CPtuDPerception::TimeoutError)==0; }
			inline bool initError() { return (nError % CPtuDPerception::InitError)==0; }
			inline bool panTiltHitError() { return (nError % CPtuDPerception::PanTiltHitError)==0; }
			inline bool panHitError() { return (nError % CPtuDPerception::PanHitError)==0; }
			inline bool tiltHitError() { return (nError % CPtuDPerception::TiltHitError)==0; }
			inline bool maxLimitError() { return (nError % CPtuDPerception::MaxLimitError)==0; }
			inline bool minLimitError () { return (nError % CPtuDPerception::MinLimitError)==0; }
			inline bool outOfRange() { return (nError % CPtuDPerception::OutOfRange)==0; }
			inline bool illegalCommandError() { return (nError % CPtuDPerception::IllegalCommandError)==0; }
			inline bool unExpectedError() { return (nError % CPtuDPerception::UnExpectedError)==0; }

			/** Clear errors **/

			virtual void clearErrors() { nError=NoError; }


		/*************************** Other member methods *****************/

		public:

			/** PTU and serial port initialization */

			virtual bool init(const std::string &port);

			/** Close conection with serial port */

			virtual void close();

			/** To obtains the mistake for use discrete values when the movement
			*	is expressed in radians. Parameters are the absolute position in
			*	radians and the axis desired
			*/

			virtual double radError(char axis,double nRadMoved);

			/**  To obtain the discrete value for a number of radians */

			virtual long radToPos(char axis,double nRad);

			/** To obtain the number of radians for a discrete value */

			virtual double posToRad(char axis,long nPos);

			/** Performs a scan in the axis indicated and whit the precision desired. \n
			*		\param <axis> {Pan or Till} \n
			*		\param <tWait> {Wait time betwen commands} \n
			*		\param <initial> {initial position}
			*		\param <final> {final position}
			*		\param <radPre> {radians precision for the scan}
			*/

			virtual bool scan(char axis, int wait, float initial, float final, double radPre);

			/** Query verbose mode */

			virtual bool verboseQ(bool &modo);

			/** Set verbose. \n
			*	\conde
			*	Example of response with FV (verbose) active:
			*		FV *
			*		PP * Current pan position is 0
			*		Example of response with FT (terse) active:
			*		FT *
			*		PP * 0
			*	\endcode
			*/

			virtual bool verbose(bool set);

			/** Query echo mode */

			virtual bool echoModeQ(bool &mode);

			/** Enable/Disable echo response with command. \n
			*	\code
			*	Example of use (EE supposed):
			*		PP * 22
			*		ED *
			*		<pp entered again, but not echoed>* 22
			*	\endcode
			*/

			virtual bool echoMode(bool mode);

			/** Query the pan and tilt resolution per position moved
			*	and initialize local atributes
			*/

			virtual bool resolution(void);


		/*************************** Methods for internal use ****************/

		private:

			/** To transmition commands to the PTU */

			virtual bool transmit(const char * command);

			/** To receive the responseof the PTU */

			virtual bool receive(const char * command,char * response);

			/** Used to obtains a number of radians */

			virtual bool radQuerry(char axis,char command,double &nRad);

			/** Method used for asign a number of radians with a command */

			virtual bool radAsign(char axis,char command,double nRad);

			/** Convert string to double */

			virtual double convertToDouble(char *sDouble);

			/** Convert string to long */

			virtual long convertToLong(char *sLong);

		/**************************** Atributes ********************/

		public:

			enum { NoError = 1, ComError = 2, TimeoutError = 3,
						InitError = 5,PanHitError = 7, TiltHitError = 11, PanTiltHitError=13,
						MaxLimitError = 17, MinLimitError = 19, OutOfRange = 23,
						IllegalCommandError = 29, UnExpectedError =31 };

			/** TimeoutError: Only occurs if the communication is cut with PTU
			*		so it is advisable to check the connection and initialize
			*		again the comunication.
			*/

			int nError;

			enum { Pan = 'P', Tilt = 'T' };
			enum { Regular = 'R', High = 'H', Low = 'L', Off = 'O' };
			enum { Com1 = 1, Com2 = 2, Com3 = 3, Com4 = 4 };


		};	// End of class

	} // End of namespace

} // End of namespace

#endif
