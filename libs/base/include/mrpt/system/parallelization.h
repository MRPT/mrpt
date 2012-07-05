/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef  __MRPT_PARALLELIZATION_H
#define  __MRPT_PARALLELIZATION_H

#include <mrpt/config.h>

// This file declares helper structs for usage with TBB
//  Refer to http://threadingbuildingblocks.org/
// (The following code blocks are based on OpenCV code - BSD license)

#if MRPT_HAS_TBB
    #include <tbb/tbb_stddef.h>
    #if TBB_VERSION_MAJOR*100 + TBB_VERSION_MINOR >= 202
        #include <tbb/tbb.h>
        #include <tbb/task.h>
        #undef min
        #undef max
    #else
        #undef MRPT_HAS_TBB
        #define MRPT_HAS_TBB 0
    #endif
#endif


// Define a common interface so if we don't have TBB it falls back to a good-old for loop:
namespace mrpt
{
	namespace system
	{
#if MRPT_HAS_TBB
        typedef tbb::blocked_range<int> BlockedRange;

        template<typename Body> static inline
        void parallel_for( const BlockedRange& range, const Body& body )
        {
            tbb::parallel_for(range, body);
        }

        template<typename Iterator, typename Body> static inline
        void parallel_do( Iterator first, Iterator last, const Body& body )
        {
            tbb::parallel_do(first, last, body);
        }

        typedef tbb::split Split;

        template<typename Body> static inline
        void parallel_reduce( const BlockedRange& range, Body& body )
        {
            tbb::parallel_reduce(range, body);
        }

        //typedef tbb::concurrent_vector<Rect> ConcurrentRectVector;
#else
		// Emulate TBB-like classes which fall back to an old "for"
        class BlockedRange
        {
        public:
            BlockedRange() : _begin(0), _end(0), _grainsize(0) {}
            BlockedRange(int b, int e, int g=1) : _begin(b), _end(e), _grainsize(g) {}
            int begin() const { return _begin; }
            int end() const { return _end; }
            int grainsize() const { return _grainsize; }
        protected:
            int _begin, _end, _grainsize;
        };

        template<typename Body> static inline
        void parallel_for( const BlockedRange& range, const Body& body )
        {
            body(range);
        }
        typedef std::vector<Rect> ConcurrentRectVector;

        template<typename Iterator, typename Body> static inline
        void parallel_do( Iterator first, Iterator last, const Body& body )
        {
            for( ; first != last; ++first )
                body(*first);
        }

        class Split {};

        template<typename Body> static inline
        void parallel_reduce( const BlockedRange& range, Body& body )
        {
            body(range);
        }

#endif // MRPT_HAS_TBB

    }  // end NS
}  // end NS

#endif
