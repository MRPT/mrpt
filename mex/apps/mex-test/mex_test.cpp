/*-----------------------------------------------------------------------------
	APPLICATION: mextest
	FILE: mex_test.cpp
	AUTHORS: Jesus Briales Garcia <jesusbriales@gmail.com>

	For instructions and details, see:
	 http://
  -----------------------------------------------------------------------------*/

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/img/CImage.h>
#include <mrpt/core/round.h>
#include <mrpt/slam/CActionCollection.h>
#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>

// Matlab MEX interface headers
#include <mexplus.h>

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;
using namespace mrpt::slam;
using namespace std;
using namespace mexplus;

// Global variable
int counter = 0;
int timer = 0;

bool allThreadsMustExit = false;

// Thread handler stored as global
std::thread thre;

void timerThread();
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
	plhs[0] = mexplus::from(counter);

	allThreadsMustExit = false;

	// Create independent thread
	if (thre.idThread == 0)  // Unitialized
	{
		printf("Thread was unitialized, launching new timerThread\n");
		thre = std::thread(timerThread);
	}

	printf("Counter: %d\n", counter);
	printf("Timer: %d\n", timer);

	counter++;
	// Set exit value for threads
	// allThreadsMustExit = true;

	// Test nested structs in Matlab
	const char* fields[] = {"field1", "test2", "what3"};
	const char* fields2[] = {"a", "b"};
	MxArray struct_array(MxArray::Struct(3, fields, 1, 2));
	MxArray other_struct(MxArray::Struct(2, fields2, 1, 2));
	other_struct.set("a", 1, 0);
	other_struct.set("b", 2, 0);
	//    struct_array.set("field1",other_struct.release(), 0);
	struct_array.set("field1", other_struct.release(), 0);
	struct_array.set("test2", "testing nested structs", 1);

	plhs[1] = struct_array.release();
	//    plhs[2] = other_struct.release();

	// exitThread(); // To finish mexFunction without finishing the other
	// threads
}

void timerThread()
{
	int process_period_ms = 3000;

	uint timerCounter = 0;
	while (!allThreadsMustExit)
	{
		printf("Inside loop counter: %d\n", timerCounter++);

		std::this_thread::sleep_for(1000ms);
		timer++;
	}
	// printf("timerThread loop exited\n");
}
