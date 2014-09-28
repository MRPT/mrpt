/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef TPspaceHybridNavigationSystem_H
#define TPspaceHybridNavigationSystem_H

#include <mrpt/utils.h>

namespace mrpt
{
  namespace hybridnav
  {

        /** This class contains methods for the TP-Space Hybrid Navigation algorithm
         *
         *  <b>Usage:</b><br>
         *		- write me
         *
         *
         *  <b>About the algorithm:</b><br>
         *
         *
         * <b>Changes history</b>
         *      - 21/FEB/2014: Creation (MB)
         *  \ingroup mrpt_hybrid_grp
         */
        class TPspaceHybridNavigationSystem //: public PTRRT_Navigator inheritance??
        {
            public:
                TPspaceHybridNavigationSystem();
                virtual ~TPspaceHybridNavigationSystem();


            protected:

            private:
        };

  }
}
#endif // TPspaceHybridNavigationSystem_H
