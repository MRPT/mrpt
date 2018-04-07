///////////////////////////////////////////////////////
// 
// A timer class designed for dealing with timestamps
// CK Nov 2002
//
///////////////////////////////////////////////////////

#include "cvd/timer.h"
#include <iostream>
#include <chrono>

using namespace  std;
using namespace  std::chrono;

namespace CVD {


long long get_time_of_day_ns()
{
	auto time = high_resolution_clock::now();

	return time_point_cast<chrono::nanoseconds>(time).time_since_epoch().count();
}



cvd_timer::cvd_timer()
{
	start = high_resolution_clock::now();
}

double cvd_timer::reset() 
{
	auto now = high_resolution_clock::now();
	double r = duration<float>(now - start).count();
	start = now;

	return r;
}

double cvd_timer::get_time() 
{
  auto now = high_resolution_clock::now();
  return duration<float>(now - start).count();
}

double get_time_of_day() 
{
  return get_time_of_day_ns()/1e9;
}


double cvd_timer::conv_ntime(const double & time) const 
{
	double start_seconds = time_point_cast<duration<float>>(start).time_since_epoch().count();
	return time - start_seconds;
}


cvd_timer timer;

}
