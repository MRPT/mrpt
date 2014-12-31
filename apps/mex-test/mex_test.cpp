/*-----------------------------------------------------------------------------
    APPLICATION: mextest
    FILE: mex_test.cpp
    AUTHORS: Jesus Briales Garcia <jesusbriales@gmail.com>

    For instructions and details, see:
     http://
  -----------------------------------------------------------------------------*/

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/round.h>
#include <mrpt/slam/CActionCollection.h>
#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>

// Matlab MEX interface headers
//#include <mex.h>
//#include <matrix.h>
#include <mexplus.h>

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace std;

// Global variable
int counter = 0;
int timer   = 0;

bool allThreadsMustExit = false;

// Thread handler stored as global
TThreadHandle thre;

void timerThread( );
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    plhs[0] = mexplus::MxArray::from(counter);

    allThreadsMustExit = false;

    // Create independent thread
    if (thre.idThread == 0) // Unitialized
    {
        printf("Thread was unitialized, launching new timerThread\n");
        thre = createThread(timerThread);
    }

    printf("Counter: %d\n", counter);
    printf("Timer: %d\n",timer);

    counter++;
    // Set exit value for threads
    //allThreadsMustExit = true;

    //exitThread(); // To finish mexFunction without finishing the other threads
}

void timerThread( )
{
    int process_period_ms = 3000;

    uint timerCounter = 0;
    while (! allThreadsMustExit )
    {
        printf("Inside loop counter: %d\n", timerCounter++);

        sleep(1000);
        timer++;
    }
    //printf("timerThread loop exited\n");
}
