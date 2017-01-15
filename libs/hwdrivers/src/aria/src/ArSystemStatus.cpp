/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include "ArExport.h"
#include "ariaOSDef.h"
#include "ariaUtil.h"
#include "ArSystemStatus.h"
#include <stdio.h>

double ArSystemStatus::ourCPU = -1.0;
unsigned long ArSystemStatus::ourUptime = 0;
unsigned long ArSystemStatus::ourLastCPUTime = 0;
ArTime ArSystemStatus::ourLastCPURefreshTime;
ArGlobalRetFunctor<double> ArSystemStatus::ourGetCPUPercentCallback(&ArSystemStatus::getCPUPercent);
ArGlobalRetFunctor<double> ArSystemStatus::ourGetUptimeHoursCallback(&ArSystemStatus::getUptimeHours);
int ArSystemStatus::ourLinkQuality = -1;
int ArSystemStatus::ourLinkSignal = -1;
int ArSystemStatus::ourLinkNoise = -1;
int ArSystemStatus::ourDiscardedTotal = -1;
int ArSystemStatus::ourDiscardedConflict = -1;
int ArSystemStatus::ourDiscardedDecrypt = -1;
ArMutex ArSystemStatus::ourCPUMutex;
ArMutex ArSystemStatus::ourWirelessMutex;
ArGlobalRetFunctor<int> ArSystemStatus::ourGetWirelessLinkQualityCallback(&ArSystemStatus::getWirelessLinkQuality);
ArGlobalRetFunctor<int> ArSystemStatus::ourGetWirelessLinkNoiseCallback(&ArSystemStatus::getWirelessLinkNoise);
ArGlobalRetFunctor<int> ArSystemStatus::ourGetWirelessLinkSignalCallback(&ArSystemStatus::getWirelessLinkSignal);
ArSystemStatusRefreshThread* ArSystemStatus::ourPeriodicUpdateThread = 0;
bool ArSystemStatus::ourShouldRefreshWireless = true;
bool ArSystemStatus::ourShouldRefreshCPU = true;


class ArScopedLock {
private:
  ArMutex& mtx;
public:
  ArScopedLock(ArMutex& m) : mtx(m) {
    mtx.lock();
  }
  ~ArScopedLock() {
    mtx.unlock();
  }
};


void ArSystemStatus::refreshCPU()
{
#ifndef WIN32
  if(ourPeriodicUpdateThread && !ourShouldRefreshCPU) return;
  unsigned long interval = ourLastCPURefreshTime.mSecSince();
  FILE* statfp = fopen("/proc/stat", "r");
  FILE* uptimefp = fopen("/proc/uptime", "r");
  if(!statfp) {
    ArLog::log(ArLog::Terse, "ArSystemStatus: Error: Failed to open /proc/stat!");
  }
  if(!uptimefp) {
    ArLog::log(ArLog::Terse, "ArSystemStatus: Error: Failed to open /proc/uptime!");
  }
  if(!statfp || !uptimefp)
  {
    ourCPU = -1.0;
    ourLastCPUTime = ourUptime = 0;
    ourShouldRefreshCPU = false;
    return;
  }
  double uptime = 0, idle_uptime = 0;
  //char line[512];
  //fgets(line,  512, uptimefp);
  //printf("read uptime file: %s\n", line);
  int dum_ret = fscanf(uptimefp, "%lf %lf", &uptime, &idle_uptime);
  fclose(uptimefp);
  unsigned long user, nice, sys, idle, total;
  char tag[32];
  dum_ret = fscanf(statfp, "%s %lu %lu %lu %lu", tag, &user, &nice, &sys, &idle);
  MRPT_UNUSED_PARAM(dum_ret);
  fclose(statfp);
  ourUptime = (unsigned long)uptime;
  total = user+nice+sys; // total non-idle cpu time in 100ths of a sec
  if(ourLastCPUTime == 0 || interval == 0)
  {
    // no time has past since last refresh
    ourLastCPUTime = total;
    ourShouldRefreshCPU = false;
    return;
  }
  ourCPU = (double)(total - ourLastCPUTime) / ((double)interval/10.0); // convert 'interval' to 1/100 sec units
  ourLastCPUTime = total;
  ourLastCPURefreshTime.setToNow();
  ourShouldRefreshCPU = false;
#endif // WIN32
}



/** @cond INTERNAL_CLASSES */
class ArSystemStatusRefreshThread : public virtual ArASyncTask {
public:
  ArSystemStatusRefreshThread(int refreshFrequency) :
    myRefreshFrequency(refreshFrequency)
  {
  }
  void runAsync() { create(false); }
  void setRefreshFreq(int freq) { myRefreshFrequency = freq; }
private:
  int myRefreshFrequency;
  virtual void* runThread(void* arg)
  {
	MRPT_UNUSED_PARAM(arg);
    while(getRunning())
    {
      ArSystemStatus::invalidate();
      ArUtil::sleep(myRefreshFrequency);
    }
    return NULL;
  }
};
/** @endcond INTERNAL_CLASSES */


AREXPORT void ArSystemStatus::startPeriodicUpdate(int refreshFrequency)
{
  if(ourPeriodicUpdateThread) {
    printf("***** Setting refresh freq to %d ms.\n", refreshFrequency);
    ourPeriodicUpdateThread->setRefreshFreq(refreshFrequency);
    return;
  }
  ourPeriodicUpdateThread = new ArSystemStatusRefreshThread(refreshFrequency);
  ourPeriodicUpdateThread->runAsync();
}

AREXPORT void ArSystemStatus::stopPeriodicUpdate()
{
  if(!ourPeriodicUpdateThread) return;
  ourPeriodicUpdateThread->stopRunning();
  delete ourPeriodicUpdateThread;
  ourPeriodicUpdateThread = 0;
}



AREXPORT double ArSystemStatus::getCPU() {
  ArScopedLock lock(ourCPUMutex);
  refreshCPU();
  return ourCPU;
}

AREXPORT double ArSystemStatus::getCPUPercent() {
  ArScopedLock lock(ourCPUMutex);
  refreshCPU();
  if(ourCPU < 0)
  {
    return ourCPU;  // invalid value indicator
  }
  return ourCPU * 100.0;
}

// Get CPU percentage in a string
AREXPORT std::string ArSystemStatus::getCPUPercentAsString() {
  ArScopedLock lock(ourCPUMutex);
  refreshCPU();
  if(ourCPU < 0)
  {
    return std::string("n/a");
  }
  char tmp[32];
  snprintf(tmp, 31, "%.2f", getCPUPercent());
  return std::string(tmp);
}

// Get total system uptime (seconds)
AREXPORT unsigned long ArSystemStatus::getUptime() {
  ArScopedLock lock(ourCPUMutex);
  refreshCPU();
  return ourUptime;
}

// Get total system uptime (hours)
AREXPORT double ArSystemStatus::getUptimeHours() {
  ArScopedLock lock(ourCPUMutex);
  refreshCPU();
  return ourUptime / 3600.0;
}

// Get total system uptime in a string (hours)
AREXPORT std::string ArSystemStatus::getUptimeHoursAsString() {
  ArScopedLock lock(ourCPUMutex);
  refreshCPU();
  char tmp[32];
  snprintf(tmp, 31, "%.2f", getUptimeHours());
  return std::string(tmp);
}

// return Pointer to a functor which can be used to retrieve the current CPU percentage
AREXPORT ArRetFunctor<double>* ArSystemStatus::getCPUPercentFunctor() {
  return &ourGetCPUPercentCallback;
}

// return Pointer to a functor which can be used to retrieve the current uptime (hours)
AREXPORT ArRetFunctor<double>* ArSystemStatus::getUptimeHoursFunctor() {
  return &ourGetUptimeHoursCallback;
}

AREXPORT ArRetFunctor<int>* ArSystemStatus::getWirelessLinkQualityFunctor() {
  return &ourGetWirelessLinkQualityCallback;
}
AREXPORT ArRetFunctor<int>* ArSystemStatus::getWirelessLinkNoiseFunctor() {
  return &ourGetWirelessLinkNoiseCallback;
}
AREXPORT ArRetFunctor<int>* ArSystemStatus::getWirelessLinkSignalFunctor() {
  return &ourGetWirelessLinkSignalCallback;
}

// Get wireless stats from /proc/net/wireless:

void ArSystemStatus::refreshWireless()
{
#ifndef WIN32
  if(ourPeriodicUpdateThread && !ourShouldRefreshWireless) return;
  FILE* fp = fopen("/proc/net/wireless", "r");
  if(!fp)
  {
    ArLog::log(ArLog::Terse, "ArSystemStatus: Error: Failed to open /proc/net/wireless!");
    ourShouldRefreshWireless = false;
    return;
  }

  // first two lines are header info
  char line[256];
  if(! (fgets(line, 256, fp) && fgets(line, 256, fp)) )
  {
    fclose(fp);
    ourLinkQuality = ourLinkSignal = ourLinkNoise =
      ourDiscardedTotal = ourDiscardedDecrypt = -1;
    ourShouldRefreshWireless = false;
    return;
  }


  // next line is info for first device
  char id[32];
  unsigned int stat;
  int disc_frag, disc_retry, disc_misc, missed;
  disc_frag = disc_retry = disc_misc = missed = 0;
  int r = fscanf(fp, "%31s %x %d. %d. %d. %d %d %d %d %d %d",
      id, &stat,
      &ourLinkQuality, &ourLinkSignal, &ourLinkNoise,
      &ourDiscardedConflict, &ourDiscardedDecrypt,
      &disc_frag, &disc_retry, &disc_misc, &missed) ;
  fclose(fp);
  if(r < 11)
    ArLog::log(ArLog::Verbose, "ArSystemStatus: Warning: Failed to parse /proc/net/wireless (only %d out of 11 values parsed).", r);
  if(ourDiscardedConflict == -1 || ourDiscardedDecrypt == -1)
    ourDiscardedTotal = -1;
  else
    ourDiscardedTotal = ourDiscardedConflict + ourDiscardedDecrypt
      + disc_frag + disc_retry + disc_misc;
  ourShouldRefreshWireless = false;
#endif // WIN32
}


AREXPORT int ArSystemStatus::getWirelessLinkQuality() {
  ArScopedLock lock(ourWirelessMutex);
  refreshWireless();
  return ourLinkQuality;
}

AREXPORT int ArSystemStatus::getWirelessLinkSignal() {
  ArScopedLock lock(ourWirelessMutex);
  refreshWireless();
  return ourLinkSignal;
}

AREXPORT int ArSystemStatus::getWirelessLinkNoise() {
  ArScopedLock lock(ourWirelessMutex);
  refreshWireless();
  return ourLinkNoise;
}

AREXPORT int ArSystemStatus::getWirelessDiscardedPackets() {
  ArScopedLock lock(ourWirelessMutex);
  refreshWireless();
  return ourDiscardedTotal;
}

AREXPORT int ArSystemStatus::getWirelessDiscardedPacketsBecauseNetConflict() {
  ArScopedLock lock(ourWirelessMutex);
  refreshWireless();
  return ourDiscardedConflict;
}



AREXPORT void ArSystemStatus::invalidate()
{
  ArScopedLock lockc(ourCPUMutex);
  ArScopedLock lockw(ourWirelessMutex);
  ourShouldRefreshCPU = ourShouldRefreshWireless = true;
}
