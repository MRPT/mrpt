/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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
#ifndef CPOSEORPOINT_DETAIL_H
#define CPOSEORPOINT_DETAIL_H

namespace mrpt
{
	namespace poses
	{
		class CPoint2D;
		class CPoint3D;
		class CPose2D;
		class CPose3D;
		class CPose3DQuat;
		class CPose3DRotVec;

		/** Internal, auxiliary templates for MRPT classes */
		namespace detail
		{
			template <class POSEORPOINT>  struct T3DTypeHelper; // generic version. Specialized below.

			template <>  struct T3DTypeHelper<CPoint2D> { enum { is_3D_val = 0 }; };
			template <>  struct T3DTypeHelper<CPoint3D> { enum { is_3D_val = 1 }; };
			template <>  struct T3DTypeHelper<CPose2D> { enum { is_3D_val = 0 }; };
			template <>  struct T3DTypeHelper<CPose3D> { enum { is_3D_val = 1 }; };
			template <>  struct T3DTypeHelper<CPose3DQuat> { enum { is_3D_val = 1 }; };
			template <>  struct T3DTypeHelper<CPose3DRotVec> { enum { is_3D_val = 1 }; };


			template <class DERIVEDCLASS, int IS3D> struct pose_point_impl;  // generic template, specialized below:

			// Extra members for 3D implementation:
			template <class DERIVEDCLASS> struct pose_point_impl<DERIVEDCLASS,1>
			{
				inline double z() const /*!< Get Z coord. */ { return static_cast<const DERIVEDCLASS*>(this)->m_coords[2]; }
				inline double &z() /*!< Get ref to Z coord. */ { return static_cast<DERIVEDCLASS*>(this)->m_coords[2]; }
				inline void z(const double v) /*!< Set Z coord. */ { static_cast<DERIVEDCLASS*>(this)->m_coords[2]=v; }
				inline void z_incr(const double v) /*!< Z+=v */ { static_cast<DERIVEDCLASS*>(this)->m_coords[2]+=v; }
			};

			// Extra members for 2D implementation:
			template <class DERIVEDCLASS> struct pose_point_impl<DERIVEDCLASS,0>
			{
			};

		} // End of namespace
	} // End of namespace
} // End of namespace

#endif
