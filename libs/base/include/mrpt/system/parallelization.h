/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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
