// timer.h
//
// A cross-platform monotonic wallclock timer.
// Currently supports nanoseconds resolution.
//
//
// @author: dharabor
//
// @created: September 2012
//

#ifndef WARTHOG_TIMER_H
#define WARTHOG_TIMER_H

#ifdef OS_MAC
#include <CoreServices/CoreServices.h>
#include <mach/mach.h>
#include <mach/mach_time.h>

#else
#include <time.h>
#endif

namespace warthog
{

class timer
{

#ifdef OS_MAC
  uint64_t start_time;
  uint64_t stop_time;
#else
	timespec stop_time;
	timespec start_time;
#endif

public:
	timer();
	void reset();
	void start();
	void stop();
	double elapsed_time_nano();
	double elapsed_time_micro();
};

}

#endif 
