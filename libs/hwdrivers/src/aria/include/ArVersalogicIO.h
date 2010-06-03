/*
MobileRobots Advanced Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007 MobileRobots Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
MobileRobots Inc, 19 Columbia Drive, Amherst, NH 03031; 800-639-9481
*/

#ifndef ARVERSALOGICIO_H
#define ARVERSALOGICIO_H

#ifndef SWIG

#include "ariaTypedefs.h"
#include "ArRobot.h"

/** @brief Interface to integrated digital and analog I/O interfaces on Versalogic motherboards.
 
  This class is a basic set of calls to use the Linux device driver, <code>amrio</code>,
  which reads and writes data to VersaLogic's Opto22 and analog interfaces.
  The <code>amrio</code> driver must be built into the Linux kernel or its
  module loaded. Contact MobileRobots for information about obtaining this
  driver.
  It currently supports the Versalogic VSBC-8d, VSBC-8k, and EBX12 (Cobra) motherboards.

  The digital portion of the Opto22 consists of two banks of 8 pins on the VSBC8
  and four banks of 8 pins on the EBX12.  Each bank can be set as either inputs
  or outputs.  The banks are zero-indexed, so bank0 is the first one.

  The analog inputs require a separate chip.  There are 8 inputs, only
  one of which can be read at a time.  It currently returns a value between
  0 and 4096 or a decimal value in the range of 0-5V.  The constructor will attempt
  an analog conversion, and if it fails will assume that the chip is not present
  and will disable the analog function.

  See the motherboard manual for information about physical connections and specifications of the analog input and Opto22 digital IO.
  Computer motherboard manuals are available at http://robots.mobilerobots.com/docs .

  The SPECIAL_CONTROL_REGISTER contains a few bits of information, the one of
  importance at the moment is the CPU_OVERTEMPERATURE bit, which will be set
  high if the CPU temp is over the warning temp as set in the BIOS.  Bitwise
  AND the special_control_register output with 0x20 to find the temperature
  bit.

  The destructor closes the device, so just delete the ArVersalogicIO instance
  to close the device.

  @swigomit
  @notwindows
*/
 
class ArVersalogicIO
{
public:

  enum Direction
  {
    DIGITAL_INPUT,
    DIGITAL_OUTPUT
  };

  /// Constructor
  AREXPORT ArVersalogicIO(const char * dev = "/dev/amrio");
  /// Destructor
  AREXPORT virtual ~ArVersalogicIO(void);

  /// tries to close the device.  Returns false if operation failed
  AREXPORT bool closeIO(void);

  /// returns true if the device is opened and operational
  /*AREXPORT*/ bool isEnabled(void) { return myEnabled; }

  /// returns true if analog values are supported
  /*AREXPORT*/ bool isAnalogSupported(void) { return myAnalogEnabled; }

  /// Take an analog reading from a port number from 0-7.
  /// This returns a conversion of the bits to a decimal value,
  /// currently assumed to be in the 0-5V range
  AREXPORT bool getAnalogValue(int port, double *val);

  /// Take an analog reading from a port number from 0-7.
  /// This returns the actual reading from the chip, which is 12-bits
  AREXPORT bool getAnalogValueRaw(int port, int *val);

  /// returns the direction (input or output) for the given bank
  AREXPORT Direction getDigitalBankDirection(int bank);

  /// set direction for a particular digital I/O bank
  AREXPORT bool setDigitalBankDirection(int bank, Direction dir);

  /// get the current value of the digital inputs on a certain bank
  AREXPORT bool getDigitalBankInputs(int bank, unsigned char *val);

  /// get the current value of the digital outputs bits on a certain bank
  AREXPORT bool getDigitalBankOutputs(int bank, unsigned char *val);

  /// set the value of the digital outputs bits
  AREXPORT bool setDigitalBankOutputs(int bank, unsigned char val);

  /// gets the special register of the motherboard.
  AREXPORT bool getSpecialControlRegister(unsigned char *val);

  /// lock the amrio device instance
  AREXPORT int lock(void){ return(myMutex.lock()); }
  /// unlock the amrio device instance
  AREXPORT int unlock(void){ return(myMutex.unlock()); }

  /// Try to lock the device instance without blocking
  /*AREXPORT*/ int tryLock() {return(myMutex.tryLock());}

protected:

  static ArMutex myMutex;
  int myFD;

  bool myEnabled;
  bool myAnalogEnabled;

  int myNumBanks;

  unsigned char myDigitalBank0;
  unsigned char myDigitalBank1;
  unsigned char myDigitalBank2;
  unsigned char myDigitalBank3;

  ArRetFunctorC<bool, ArVersalogicIO> myDisconnectCB;
};

#endif // SWIG

#endif // ARVERSALOGICIO_H
