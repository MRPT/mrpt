
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "xsdebugcounters.h"
#include <atomic>

#ifdef XSENS_USE_DEBUG_COUNTERS
volatile std::atomic_int XsVector_allocCount_value(0);
volatile std::atomic_int XsVector_freeCount_value(0);

volatile std::atomic_int XsMatrix_allocCount_value(0);
volatile std::atomic_int XsMatrix_freeCount_value(0);

volatile std::atomic_int XsArray_allocCount_value(0);
volatile std::atomic_int XsArray_freeCount_value(0);

extern "C"
{

int XsVector_resetDebugCounts()
{
	return XsVector_allocCount_value.exchange(0) + XsVector_freeCount_value.exchange(0);
}

int XsVector_allocCount()
{
	return XsVector_allocCount_value.load();
}

int XsVector_freeCount()
{
	return XsVector_freeCount_value.load();
}

int XsVector_incAllocCount()
{
	return ++XsVector_allocCount_value;
}
int XsVector_incFreeCount()
{
	return ++XsVector_freeCount_value;
}

int XsMatrix_resetDebugCounts()
{
	return XsMatrix_allocCount_value.exchange(0) + XsMatrix_freeCount_value.exchange(0);
}

int XsMatrix_allocCount()
{
	return XsMatrix_allocCount_value.load();
}

int XsMatrix_freeCount()
{
	return XsMatrix_freeCount_value.load();
}

int XsMatrix_incAllocCount()
{
	return ++XsMatrix_allocCount_value;
}

int XsMatrix_incFreeCount()
{
	return ++XsMatrix_freeCount_value;
}

int XsArray_resetDebugCounts()
{
	return XsArray_allocCount_value.exchange(0) + XsArray_freeCount_value.exchange(0);
}

int XsArray_allocCount()
{
	return XsArray_allocCount_value.load();
}

int XsArray_freeCount()
{
	return XsArray_freeCount_value.load();
}

int XsArray_incAllocCount()
{
	return ++XsArray_allocCount_value;
}

int XsArray_incFreeCount()
{
	return ++XsArray_freeCount_value;
}

} // extern "C"

#endif
