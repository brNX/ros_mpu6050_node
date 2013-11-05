////////////////////////////////////////////////////////////////////////////
//
//  This file is part of linux-mpu9150
//
//  Copyright (c) 2013 Pansenti, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <time.h>
#include <sys/time.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include "ros_glue.h"



void __no_operation(void) { }

int delay_ms(unsigned long num_ms)
{
	struct timespec ts;

	ts.tv_sec = num_ms / 1000;
	ts.tv_nsec = (num_ms % 1000) * 1000000;

	return nanosleep(&ts, NULL);
}

int get_ms(unsigned long *count)
{
	struct timeval t;

	if (!count)
		return -1;

	if (gettimeofday(&t, NULL) < 0) {
		perror("gettimeofday");
		return -1;
	}

	*count = (t.tv_sec * 1000) + (t.tv_usec / 1000);	

	return 0;
}

