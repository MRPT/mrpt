/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/system/CObserver.h>
#include <iostream>

using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace std;

// Load a example laser scan:
CObservation2DRangeScan getScan()
{
	CObservation2DRangeScan scan1;
	stock_observations::example2DRangeScan(scan1);
	return scan1;
}

class MyObserver : public mrpt::system::CObserver
{
   protected:
	void OnEvent(const mrptEvent& e) override
	{
		if (e.isOfType<mrptEventMetricMapClear>())
			cout << "[MyObserver] Event received: mrptEventMetricMapClear\n";
		else if (e.isOfType<mrptEventMetricMapInsert>())
			cout << "[MyObserver] Event received: mrptEventMetricMapInsert\n";
		else if (e.isOfType<mrptEventOnDestroy>())
			cout << "[MyObserver] Event received: mrptEventOnDestroy\n";
		else
			cout << "[MyObserver] Event received: Generic mrptEvent \n";
	}
};

// ------------------------------------------------------
//				TestObserverPattern
// ------------------------------------------------------
void TestObserverPattern()
{
	CSimplePointsMap::Ptr map1 = mrpt::make_aligned_shared<CSimplePointsMap>();

	CObservation2DRangeScan obs = getScan();

	// Start observer:
	MyObserver observer;
	observer.observeBegin(*map1);

	// Insert:
	for (int i = 0; i < 3; i++) map1->insertObservation(&obs);

	// Clear:
	map1->clear();

	// Insert again:
	map1->insertObservation(&obs);

	// Destroy
	map1.reset();
}

int main()
{
	try
	{
		TestObserverPattern();

		return 0;
	}
	catch (exception& e)
	{
		cout << "MRPT exception caught: " << e.what() << endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}
}
