////////////////////////////////////////////////////////
//
// A timer class designed for dealing with timestamps
// CK Nov 2002
//
////////////////////////////////////////////////////////

#ifndef __CVD_TIMER_H
#define __CVD_TIMER_H
#include <algorithm>
#include <iostream>
#include <string>
#include <cassert>
#include <deque>
#include <chrono>

namespace CVD {


/// Provides the time elapsed in seconds. This predominantly a wrapper for
/// the system call
/// gettimeofday(), but can also provides a convenient way of converting from other
/// time units. In all cases, the time is given relative to the time the class
/// was created, or the last time reset() was called.
/// @ingroup gCPP
class cvd_timer
{
	public:
		/// Create a timer, and set the start time to be now
		cvd_timer();

		/// How many seconds have elapsed since the start time?
		double get_time();


		/// Convert current time given as double by correcting for the start time
		/// @param time current time as double
		double conv_ntime(const double & time)const;

		/// Sets the start time to the current time
		double reset();
		
	private:
		std::chrono::high_resolution_clock::time_point start;
};

extern cvd_timer timer;

/// Same as the system call gettimeofday, but returns seconds since
/// the epoch as a double.
double get_time_of_day();


/// Same as the system call gettimeofday, but returns nanoseconds seconds since
/// the epoch
long long get_time_of_day_ns();


/// Provides a simple timer class which uses cvd_timer internally.
/// Statistics (average, max and min cycle time) are kept and output for a number of timing cycles.
/// Cycle times are stored in a std::deque<double> and statistics only calculated when the get or print functions are called.
/// @ingroup gCPP
class SimpleTimer
{
    public:
        /// Create a simple timer object.
        /// @param description A string description of what is being timed (used in output)
        /// @param cycles_to_time How many cycles to time before the times are averaged and output (default 1).
        /// @param output Whether or not to output timing information. Default is true.
        /// @param out std::ostream to send output information to. Default std::cout.
        SimpleTimer(const std::string &description, const int &cycles_to_time=1, bool output=true, std::ostream &out=std::cout) : output_info(output), max(0), min(0), average(0), text(description), period(cycles_to_time), increment(0), timings(0), cumulative_time(0), time_at_start_of_timing(0), timing_started(false), sout(out)
        {
            assert(period>0);
            internal_cvd_timer=new cvd_timer();
        }

        /// Destructor. Deletes the internal cvd_timer
        ~SimpleTimer()
        {
            delete internal_cvd_timer;
        }

        /// Begin or end a timing cycle. Automatically calls print() when cycles_to_time cycles are reached.
        void click()
        {
                if (!timing_started)
                {
                    timing_started=true;
                    time_at_start_of_timing=internal_cvd_timer->get_time();
                }
                else if (timing_started)
                {
                    increment=internal_cvd_timer->get_time()-time_at_start_of_timing;
                    time_buffer.push_back(increment);
                    if((int)time_buffer.size()>period&&period>0)
                        time_buffer.pop_front();
                    timings++;
                    timing_started=false;
                    if (timings%period==0 && output_info)
                        print();
                }
        }

        /// Output timing information (average, maximum and minimum times for a set of cycles).
        /// Automatically called after cycles_to_time cycles but can be called manually.
        void print()
        {
            if(timings>0)
            {
                if((int)time_buffer.size()==1)
                    sout<<text<<" takes: "<<get_average()<<"s over 1 timing"<<std::endl;
                else
                    sout<<text<<" takes : av "<<get_average()<<"s , max "<<get_max()<<"s, min "<<get_min()<<"s, over "<<time_buffer.size()<<" timings"<<std::endl;
            }
            else
                sout<<text<<" section : error. No timed cycles. Use click() to start and stop timing."<<std::endl;
        }

        /// Calculate the max cycle time as double
        double get_max(){
            max=-1;
            if (time_buffer.size()>0)
                max=time_buffer[0];
            if(time_buffer.size()>1)
                for(int i=0; i<(int)time_buffer.size(); ++i)
                    max=std::max(time_buffer[i], max);

            return max;

        }

        /// Calculate the min cycle time as double
        double get_min(){
            min=-1;
            if (time_buffer.size()>0)
                min=time_buffer[0];
            if(time_buffer.size()>1)
                for(int i=0; i<(int)time_buffer.size(); ++i)
                    min=std::min(time_buffer[i], min);
            return min;
        }

        /// Calculate the average cycle time as double
        double get_average(){
            average=-1;
            if(time_buffer.size()>0){
            cumulative_time=0;
            for(int i=0; i<(int)time_buffer.size(); ++i)
                cumulative_time+=time_buffer[i];
            average=cumulative_time/time_buffer.size();
            }
            return average;
        }

    private:
        bool output_info;
        double max, min, average;
        std::string text;
        int period;
        double increment;
        int timings;
        double cumulative_time;
        double time_at_start_of_timing;
        bool timing_started;
        cvd_timer* internal_cvd_timer;
        std::ostream& sout;
        std::deque<double> time_buffer;


};

}

#endif
