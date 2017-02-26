/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CPtuBase_H
#define CPtuBase_H

#include <mrpt/hwdrivers/CSerialPort.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** This class implements initialization and comunication methods to
		  * control a generic Pan and Tilt Unit, working in radians.
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CPtuBase
		{

			/*************************** Atributes **********************/

		public:

			double tiltResolution,panResolution;

		protected:

			CSerialPort serPort;

			/**************************** Methods ***********************/

		public:

			/** Destructor */

			virtual ~CPtuBase() {};

			/** Search limit forward */

			virtual bool rangeMeasure()=0;

			/** Specification of positions in absolute terms */

			virtual bool moveToAbsPos(char axis,double nRad)=0;

			/** Query position in absolute terms */

			virtual bool absPosQ(char axis,double &nRad)=0;

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

			virtual bool moveToOffPos(char axis,double nRad)=0;

			/** Query position in relative terms */

			virtual bool offPosQ(char axis,double &nRad)=0;

			/** Query max movement limit of a axis in absolute terms */

			virtual bool maxPosQ(char axis,double &nRad)=0;

			/** Query min movement limit of a axis in absolute terms */

			virtual bool minPosQ(char axis,double &nRad)=0;

			/** Query if exist movement limits */

			virtual bool enableLimitsQ(bool &enable)=0; // Query if exist some limit

			/** Enable/Disable movement limits */

			virtual bool enableLimits(bool set)=0;

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

			virtual bool inmediateExecution(bool set)=0;

			/** Wait the finish of the last position command to
			*	continue accept commands
			*/

			virtual bool aWait(void)=0;

			/** Inmediately stop all */

			virtual bool haltAll()=0;

			/** Inmediately stop */

			virtual bool halt(char axis)=0;

		    /** Specification of turn speed */

			virtual bool  speed(char axis,double RadSec)=0;

			/** Query turn speed */

			virtual bool  speedQ(char axis,double &RadSec)=0;

			/** Specification (de/a)celeration in turn */

			virtual bool  aceleration(char axis,double RadSec2)=0;

			/** Query (de/a)celeration in turn */

			virtual bool  acelerationQ(char axis,double &RadSec2)=0;

			/** Specification of velocity to which start and finish
			*	the (de/a)celeration
			*/

			virtual bool  baseSpeed(char axis,double RadSec)=0;

			/** Query velocity to which start and finish
			*	the (de/a)celeration
			*/

			virtual bool  baseSpeedQ(char axis,double &RadSec)=0;

			/** Specification of velocity upper limit */

			virtual bool upperSpeed(char axis,double RadSec)=0;

			/** Query velocity upper limit */

			virtual bool upperSpeedQ(char axis,double &RadSec)=0;

			/** Specification of velocity lower limit */

			virtual bool lowerSpeed(char axis,double RadSec)=0;

			/** Query velocity lower limit */

			virtual bool lowerSpeedQ(char axis,double &RadSec)=0;

			/** Reset PTU to initial state */

			virtual bool reset(void)=0;

			/** Save or restart default values */

			virtual bool save(void)=0;

			/** Restore default values */

			virtual bool restoreDefaults(void)=0;

			/** Restore factory default values */

			virtual bool restoreFactoryDefaults(void)=0;

			/** Version and CopyRights */

			virtual bool version(char * nVersion)=0;

			/** Number of version */

			virtual void nversion(double &nVersion)=0;

			/** Query power mode */

			virtual bool powerModeQ(bool transit,char &mode)=0;

			/** Specification of power mode */

			virtual bool powerMode(bool transit,char mode)=0;

			/** Check if ptu is moving */

			virtual double status(double &rad)=0;

			/** Set limits of movement */

			virtual bool setLimits(char axis, double &l, double &u)=0;

			/* Change motion direction */

			virtual bool changeMotionDir()=0;


		/**************************** State Queries ********************/

			/** Check errors, returns 0 if there are not errors or error code otherwise **/

			virtual int checkErrors()=0;

			/** Clear errors **/

			virtual void clearErrors()=0;


		/*************************** Other member methods *****************/

			/** PTU and serial port initialization */

			virtual bool init(const std::string &port)=0;

			/** Close conection with serial port */

			virtual void close()=0;

			/** To obtains the mistake for use discrete values when the movement
			*	is expressed in radians. Parameters are the absolute position in
			*	radians and the axis desired
			*/

			virtual double radError(char axis,double nRadMoved)=0;

			/**  To obtain the discrete value for a number of radians */

			virtual long radToPos(char axis,double nRad)=0;

			/** To obtain the number of radians for a discrete value */

			virtual double posToRad(char axis,long nPos)=0;

			/** Performs a scan in the axis indicated and whit the precision desired.
			*		\param <axis> {Pan or Till}
			*		\param <tWait> {Wait time betwen commands}
			*		\param <initial> {initial position}
			*		\param <final> {final position}
			*		\param <RadPre> {radians precision for the scan}
			*/

			virtual bool scan(char axis, int wait, float initial, float final, double RadPre)=0;

			/** Query verbose mode */

			virtual bool verboseQ(bool &modo)=0;

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

			virtual bool verbose(bool set)=0;

			/** Query echo mode */

			virtual bool echoModeQ(bool &mode)=0;

			/** Enable/Disable echo response with command. \n
			*	\code
			*	Example of use (EE supposed):
			*		PP * 22
			*		ED *
			*		<pp entered again, but not echoed>* 22
			*	\endcode
			*/

			virtual bool echoMode(bool mode)=0;

			/** Query the pan and tilt resolution per position moved
			*	and initialize local atributes
			*/

			virtual bool resolution(void)=0;


		/*************************** Methods for internal use ****************/

		private:

			/** To transmition commands to the PTU */

			virtual bool transmit(const char * command)=0;

			/** To receive the responseof the PTU */

			virtual bool receive(const char * command,char * response)=0;

			/** Used to obtains a number of radians */

			virtual bool radQuerry(char axis,char command,double &nRad)=0;

			/** Method used for asign a number of radians with a command */

			virtual bool radAsign(char axis,char command,double nRad)=0;


		};	// End of class

	} // End of namespace

} // End of namespace

#endif
