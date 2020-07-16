/*
 * fm_stopwatch.h
 *
 *  Created on: Jul 8, 2012
 *      Author: demian
 */

#include <ros/ros.h>


using namespace std;

namespace fmutil
{
struct Stopwatch
{
	string msg_;
	uint64_t time_;
	uint64_t total_;

	Stopwatch()
	{
		
		total_ = 0;
	}

  Stopwatch(string msg, bool start=true)
	{
		msg_ = msg;
		total_ = 0;
		if(start) time_ = GetTimeMs64();
	}
	void reset()
	{
		total_ = 0;
	}
	uint64_t GetTimeMs64()
	{
		struct timeval tv;

		gettimeofday(&tv, NULL);

		uint64_t ret = tv.tv_usec;
		/* Convert from micro seconds (10^-6) to milliseconds (10^-3) */
		//ret /= 1000;

		/* Adds the seconds (10^0) after converting them to milliseconds (10^-3) */
		ret += (tv.tv_sec * 1000000);

		return ret;
	}

	void start(string msg)
	{
        msg_ = msg;
		time_ = GetTimeMs64();
	}

  void start()
	{
		time_ = GetTimeMs64();
	}
	void end(bool print_msg)
	{
		uint64_t time_diff = GetTimeMs64()-time_;
		if(print_msg)	cout<<msg_<<": "<<time_diff/1000<<" ms"<<endl;
		total_+=time_diff;
	}

    void end()
    {
        end(true);
    }

};
}
