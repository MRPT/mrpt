/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */

#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/system/CControlledRateTimer.h>
#include <iostream>

// Declare the supported command line switches ===========
TCLAP::CmdLine cmd("system_control_rate_timer_example");

TCLAP::ValueArg<double> argRate1(
	"1", "rate1", "rate1 (Hz)", false, 500.0, "rate (Hz)", cmd);
TCLAP::ValueArg<double> argRate2(
	"2", "rate2", "rate2 (Hz)", false, 1500.0, "rate (Hz)", cmd);

void rateTest()
{
	const double TOTAL_EXECUTION_TIME = 5.0;  // seconds
	const double NOMINAL_RATE1 = argRate1.getValue();
	const double NOMINAL_RATE2 = argRate2.getValue();
	const unsigned int N = TOTAL_EXECUTION_TIME * NOMINAL_RATE1;
	const unsigned int STEP_TIME1 = N / 2;

	mrpt::system::CControlledRateTimer rate;
	rate.setRate(NOMINAL_RATE1);  // Hz

	mrpt::math::CVectorDouble estimatedRates, rawRates, actionRates, controlref;
	estimatedRates.resize(N);
	rawRates.resize(N);
	actionRates.resize(N);
	controlref.resize(N);

	printf("Running for %f seconds...\n", TOTAL_EXECUTION_TIME);

	for (unsigned int i = 0; i < N; i++)
	{
		// ---- here starts a regular user loop with a rate timer -----

		// Do the main loop job here:
		// ......

		// Wait so we start over exactly when the next period is about to start:
		rate.sleep();

		// ---- here ends a regular user loop with a rate timer -----

		// Graphs, for this example only, don't use in production! ;-)
		// Reference:
		const double desiredRate =
			(i < STEP_TIME1) ? NOMINAL_RATE1 : NOMINAL_RATE2;

		rate.setRate(desiredRate);

		estimatedRates[i] = rate.estimatedRate();
		rawRates[i] = rate.estimatedRateRaw();
		actionRates[i] = rate.actualControlledRate();
		controlref[i] = desiredRate;
	}

	mrpt::gui::CDisplayWindowPlots win(
		"Measured rates (Hz) [black] / estimated "
		"rates [red] / control action [blue]",
		600, 600);
	win.plot(rawRates, "k.3");
	win.hold_on();
	win.plot(estimatedRates, "r-");
	win.plot(actionRates, "b.2");
	win.plot(controlref, "m-");

	win.axis(-0.15 * N, N, -500, 2.0 * std::max(NOMINAL_RATE1, NOMINAL_RATE2));
	win.waitForKey();
}

int main(int argc, char** argv)
{
	try
	{
		// Parse arguments:
		if (!cmd.parse(argc, argv))
			throw std::runtime_error("");  // should exit.

		rateTest();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}
