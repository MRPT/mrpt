/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#ifndef ARSYSTEMSTATUS_H
#define ARSYSTEMSTATUS_H

#include "Aria.h"

class ArSystemStatusRefreshThread;

/** @brief Utility to get statistics about the  host operating system
 *  (CPU usage, wireless link data, etc).
 *
 *  This class is only implemented for Linux; on Windows you will get invalid
 *  information.
 *  @todo Add a function and functor that formats uptime like "X years, 
 *  X months, X days, X hours, X min, X sec." (omitting 0 values).
 */
class ArSystemStatus {
public:

  /** Create a new thread which periodically invalidates cached data,
   *  causing it to be recalculated when next accessed. Use this if you
   *  will be accessing the data too frequently to require it to be 
   *  recacluted on each access.
   */
  AREXPORT static void startPeriodicUpdate(int refreshFrequency = 5000);

  /** Stop periodic update thread. Henceforth any access of data will
   *  cause it to be re-read and recalculated. */
  AREXPORT static void stopPeriodicUpdate();

  /** @deprecated use startPeriodicUpdate() which has a better name. */
  /*AREXPORT*/ static void runRefreshThread(int refreshFrequency = 5000) {
    startPeriodicUpdate(refreshFrequency);
  }

  /** Get CPU work to idle ratio since last refresh.
   *  This is a value ranging from (0 .. 1) X (Num. CPUs). (Therefore
   *  if you have two CPUs, the maximum value will be 2.0, or 200%.)
   *  This value is calculated as the percentage 
   *  of time the CPU spent doing work (not in "idle" state) since the 
   *  previous calculation.
   *  @return CPU usage value, or -1 if unable to determine
   */
  AREXPORT static double getCPU();

  /** Get CPU usage as percentage since last refresh. This is a value ranging from
   *  (0..100) X (Num. CPUs). (Therefore if you have two CPUs, the maximum value
   *  will be 200%).
   *  @sa getCPU()
   *  @return CPU usage as percentage, or -1 if not able to determine
   */
  AREXPORT static double getCPUPercent();

  /// Get CPU percentage in a string
  AREXPORT static std::string getCPUPercentAsString();

  /// Get total system uptime (seconds)
  AREXPORT static unsigned long getUptime();

  /// Get total system uptime (hours)
  AREXPORT static double getUptimeHours();

  /// Get total system uptime in a string (hours)
  AREXPORT static std::string getUptimeHoursAsString();

  /** @return Pointer to a functor which can be used to retrieve the current CPU percentage */
  AREXPORT static ArRetFunctor<double>* getCPUPercentFunctor();

  /** @return Pointer to a functor which can be used to retrieve the current uptime (hours) */
  AREXPORT static ArRetFunctor<double>* getUptimeHoursFunctor();



  /** Get wireless network general link quality heuristic (for first configured
   * wireless device). */
  AREXPORT static int getWirelessLinkQuality();

  /** Get wireless network signal level (for first configured
   * wireless device). */
  AREXPORT static int getWirelessLinkSignal();

  /** Get wireless network noise level (for first configured
   * wireless device). */
  AREXPORT static int getWirelessLinkNoise();

  /** Get wireless network total discarded packets (for first configured
   * wireless device). */
  AREXPORT static int getWirelessDiscardedPackets();

  /** Get wireless network packets discarded because of a conflict with another
   * network (for first configured
   * wireless device). */
  AREXPORT static int getWirelessDiscardedPacketsBecauseNetConflict();

  AREXPORT static ArRetFunctor<int>* getWirelessLinkQualityFunctor();
  AREXPORT static ArRetFunctor<int>* getWirelessLinkNoiseFunctor();
  AREXPORT static ArRetFunctor<int>* getWirelessLinkSignalFunctor();

  /** @internal */
  AREXPORT static void invalidate();

  /** @deprecated Calling this function is no longer neccesary. */
  /*AREXPORT*/ static void refresh() { } 
private:
  

  static ArMutex ourCPUMutex;
  static double ourCPU;
  static unsigned long ourUptime;
  static unsigned long ourLastCPUTime;
  static ArTime ourLastCPURefreshTime;
  static ArGlobalRetFunctor<double> ourGetCPUPercentCallback;
  static ArGlobalRetFunctor<double> ourGetUptimeHoursCallback;

  static ArMutex ourWirelessMutex;
  static int ourLinkQuality, ourLinkSignal, ourLinkNoise,
        ourDiscardedTotal, ourDiscardedDecrypt, ourDiscardedConflict;
  static ArGlobalRetFunctor<int> ourGetWirelessLinkQualityCallback;
  static ArGlobalRetFunctor<int> ourGetWirelessLinkNoiseCallback;
  static ArGlobalRetFunctor<int> ourGetWirelessLinkSignalCallback;

  static void refreshCPU(); ///< Refresh CPU, if neccesary
  static void refreshWireless(); ///< Refresh Wireless stats, if neccesary

  
  static ArSystemStatusRefreshThread* ourPeriodicUpdateThread;
  static bool ourShouldRefreshWireless;
  static bool ourShouldRefreshCPU;

};

#endif
