/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/comms/CSerialPort.h>
#include <mrpt/hwdrivers/CPtuBase.h>

namespace mrpt::hwdrivers
{
/** This class implements initialization and comunication methods to
 * control a Pan and Tilt Unit model PTU-46-17.5, working in radians .
 * \ingroup mrpt_hwdrivers_grp
 */
class CPtuDPerception : public CPtuBase
{
   public:
	/** Default constructor */

	CPtuDPerception() = default;

	/** Destructor */

	~CPtuDPerception() override { close(); }
	/*************************** Commands ***************************/

   public:
	/** Search limit forward */

	bool rangeMeasure() override;

	/** Specification of positions in absolute terms */

	bool moveToAbsPos(char axis, double nRad) override;

	/** Query position in absolute terms */

	bool absPosQ(char axis, double& nRad) override;

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

	bool moveToOffPos(char axis, double nRad) override;

	/** Query position in relative terms */

	bool offPosQ(char axis, double& nRad) override;

	/** Query max movement limit of a axis in absolute terms */

	bool maxPosQ(char axis, double& nRad) override;

	/** Query min movement limit of a axis in absolute terms */

	bool minPosQ(char axis, double& nRad) override;

	/** Query if exist movement limits */

	bool enableLimitsQ(bool& enable) override;  // Query if exist some limit

	/** Enable/Disable movement limits */

	bool enableLimits(bool set) override;

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

	bool inmediateExecution(bool set) override;

	/** Wait the finish of the last position command to
	 *	continue accept commands
	 */

	bool aWait() override;

	/** Inmediately stop all */

	bool haltAll() override;

	/** Inmediately stop */

	bool halt(char axis) override;

	/** Specification of turn speed */

	bool speed(char axis, double radSec) override;

	/** Query turn speed */

	bool speedQ(char axis, double& radSec) override;

	/** Specification (de/a)celeration in turn */

	bool aceleration(char axis, double radSec2) override;

	/** Query (de/a)celeration in turn */

	bool acelerationQ(char axis, double& radSec2) override;

	/** Specification of velocity to which start and finish
	 *	the (de/a)celeration
	 */

	bool baseSpeed(char axis, double radSec) override;

	/** Query velocity to which start and finish
	 *	the (de/a)celeration
	 */

	bool baseSpeedQ(char axis, double& radSec) override;

	/** Specification of velocity upper limit */

	bool upperSpeed(char axis, double radSec) override;

	/** Query velocity upper limit */

	bool upperSpeedQ(char axis, double& radSec) override;

	/** Specification of velocity lower limit */

	bool lowerSpeed(char axis, double radSec) override;

	/** Query velocity lower limit */

	bool lowerSpeedQ(char axis, double& radSec) override;

	/** Reset PTU to initial state */

	bool reset() override;

	/** Save or restart default values */

	bool save() override;

	/** Restore default values */

	bool restoreDefaults() override;

	/** Restore factory default values */

	bool restoreFactoryDefaults() override;

	/** Version and CopyRights */

	bool version(char* nVersion) override;

	/** Number of version */

	void nversion(double& nVersion) override;

	/** Query power mode */

	bool powerModeQ(bool transit, char& mode) override;

	/** Specification of power mode */

	bool powerMode(bool transit, char mode) override;

	/** Check if ptu is moving */

	double status(double& rad) override
	{
		MRPT_UNUSED_PARAM(rad);
		return 1;
	}

	/** Set limits of movement */

	bool setLimits(char axis, double& l, double& u) override;

	/* Change motion direction */

	bool changeMotionDir() override;

	/**************************** State Queries ********************/

	/** Check errors, returns 0 if there are not errors or error code in
	 *otherwise
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

	int checkErrors() override;

	inline bool noError() { return nError == 1; }
	inline bool comError() { return (nError % CPtuDPerception::ComError) == 0; }
	inline bool timeoutError()
	{
		return (nError % CPtuDPerception::TimeoutError) == 0;
	}
	inline bool initError()
	{
		return (nError % CPtuDPerception::InitError) == 0;
	}
	inline bool panTiltHitError()
	{
		return (nError % CPtuDPerception::PanTiltHitError) == 0;
	}
	inline bool panHitError()
	{
		return (nError % CPtuDPerception::PanHitError) == 0;
	}
	inline bool tiltHitError()
	{
		return (nError % CPtuDPerception::TiltHitError) == 0;
	}
	inline bool maxLimitError()
	{
		return (nError % CPtuDPerception::MaxLimitError) == 0;
	}
	inline bool minLimitError()
	{
		return (nError % CPtuDPerception::MinLimitError) == 0;
	}
	inline bool outOfRange()
	{
		return (nError % CPtuDPerception::OutOfRange) == 0;
	}
	inline bool illegalCommandError()
	{
		return (nError % CPtuDPerception::IllegalCommandError) == 0;
	}
	inline bool unExpectedError()
	{
		return (nError % CPtuDPerception::UnExpectedError) == 0;
	}

	/** Clear errors **/

	void clearErrors() override { nError = NoError; }
	/*************************** Other member methods *****************/

   public:
	/** PTU and serial port initialization */

	bool init(const std::string& port) override;

	/** Close Connection with serial port */

	void close() override;

	/** To obtains the mistake for use discrete values when the movement
	 *	is expressed in radians. Parameters are the absolute position in
	 *	radians and the axis desired
	 */

	double radError(char axis, double nRadMoved) override;

	/**  To obtain the discrete value for a number of radians */

	long radToPos(char axis, double nRad) override;

	/** To obtain the number of radians for a discrete value */

	double posToRad(char axis, long nPos) override;

	/** Performs a scan in the axis indicated and whit the precision desired. \n
	 *		\param <axis> {Pan or Till} \n
	 *		\param <tWait> {Wait time betwen commands} \n
	 *		\param <initial> {initial position}
	 *		\param <final> {final position}
	 *		\param <radPre> {radians precision for the scan}
	 */

	bool scan(char axis, int wait, float initial, float final, double radPre)
		override;

	/** Query verbose mode */

	bool verboseQ(bool& modo) override;

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

	bool verbose(bool set) override;

	/** Query echo mode */

	bool echoModeQ(bool& mode) override;

	/** Enable/Disable echo response with command. \n
	 *	\code
	 *	Example of use (EE supposed):
	 *		PP * 22
	 *		ED *
	 *		<pp entered again, but not echoed>* 22
	 *	\endcode
	 */

	bool echoMode(bool mode) override;

	/** Query the pan and tilt resolution per position moved
	 *	and initialize local atributes
	 */

	bool resolution() override;

	/*************************** Methods for internal use ****************/

   private:
	/** To transmition commands to the PTU */

	bool transmit(const char* command) override;

	/** To receive the responseof the PTU */

	bool receive(const char* command, char* response) override;

	/** Used to obtains a number of radians */

	bool radQuerry(char axis, char command, double& nRad) override;

	/** Method used for asign a number of radians with a command */

	bool radAsign(char axis, char command, double nRad) override;

	/** Convert string to double */

	virtual double convertToDouble(char* sDouble);

	/** Convert string to long */

	virtual long convertToLong(char* sLong);

	/**************************** Atributes ********************/

   public:
	enum
	{
		NoError = 1,
		ComError = 2,
		TimeoutError = 3,
		InitError = 5,
		PanHitError = 7,
		TiltHitError = 11,
		PanTiltHitError = 13,
		MaxLimitError = 17,
		MinLimitError = 19,
		OutOfRange = 23,
		IllegalCommandError = 29,
		UnExpectedError = 31
	};

	/** TimeoutError: Only occurs if the communication is cut with PTU
	 *		so it is advisable to check the connection and initialize
	 *		again the comunication.
	 */

	int nError;

	enum
	{
		Pan = 'P',
		Tilt = 'T'
	};
	enum
	{
		Regular = 'R',
		High = 'H',
		Low = 'L',
		Off = 'O'
	};
	enum
	{
		Com1 = 1,
		Com2 = 2,
		Com3 = 3,
		Com4 = 4
	};

};  // End of class

}  // namespace mrpt::hwdrivers
