/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */

#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/system/CControlledRateTimer.h>

#include <CLI/CLI.hpp>
#include <iostream>

void rateTest(double rate1, double rate2)
{
  const double TOTAL_EXECUTION_TIME = 5.0;  // seconds
  const unsigned int N = static_cast<unsigned int>(TOTAL_EXECUTION_TIME * rate1);
  const unsigned int STEP_TIME1 = N / 2;

  mrpt::system::CControlledRateTimer rate;
  rate.setRate(rate1);  // Hz

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
    const double desiredRate = (i < STEP_TIME1) ? rate1 : rate2;

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

  win.axis(-0.15 * N, N, -500, 2.0 * std::max(rate1, rate2));
  win.waitForKey();
}

int main(int argc, char** argv)
{
  CLI::App app{"system_control_rate_timer_example"};

  double rate1 = 500.0;
  double rate2 = 1500.0;
  app.add_option("-1,--rate1", rate1, "rate1 (Hz)")->capture_default_str();
  app.add_option("-2,--rate2", rate2, "rate2 (Hz)")->capture_default_str();

  CLI11_PARSE(app, argc, argv);

  try
  {
    rateTest(rate1, rate2);
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
    return -1;
  }
}
