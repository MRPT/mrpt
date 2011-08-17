/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef  TMatchingPair_H
#define  TMatchingPair_H

#include <mrpt/utils/utils_defs.h>

namespace mrpt
{
	namespace utils
	{
		using namespace mrpt::poses;

	// Pragma defined to ensure no structure packing, so we can use SSE2 vectorization on parts of this struture
#pragma pack(push,1)

		/** A structure for holding correspondences between two sets of points or points-like entities in 2D or 3D.
		 * \ingroup mrpt_base_grp
		  */
		struct BASE_IMPEXP TMatchingPair
		{
			TMatchingPair() :
				this_idx(0), other_idx(0),
				this_x(0),this_y(0),this_z(0),
				other_x(0),other_y(0),other_z(0),
				errorSquareAfterTransformation(0)
			{
			}

			TMatchingPair( unsigned int _this_idx,unsigned int _other_idx, float _this_x, float _this_y,float _this_z, float _other_x,float _other_y,float _other_z ) :
					this_idx(_this_idx), other_idx(_other_idx),
					this_x(_this_x),this_y(_this_y),this_z(_this_z),
					other_x(_other_x),other_y(_other_y),other_z(_other_z),
					errorSquareAfterTransformation(0)
			{
			}

			unsigned int	this_idx;
			unsigned int	other_idx;
			float			this_x,this_y,this_z;
			float			other_x,other_y,other_z;
			float			errorSquareAfterTransformation;

		};

#pragma pack(pop) // End of pack = 1


		typedef TMatchingPair*  TMatchingPairPtr;

		/** A list of TMatchingPair
		 * \ingroup mrpt_base_grp
		  */
		class BASE_IMPEXP TMatchingPairList : public std::vector<TMatchingPair>
		{
		public:

			/** Checks if the given index from the "other" map appears in the list.
			  */
			bool  indexOtherMapHasCorrespondence(unsigned int idx);

			/** Saves the correspondences to a text file
			  */
			void  dumpToFile(const std::string &fileName);

			/** Saves the correspondences as a MATLAB script which draws them.
			  */
			void saveAsMATLABScript( const std::string &filName );

			/** Computes the overall square error between the 2D points in the list of correspondences, given the 2D transformation "q"
			  *    \f[ \sum\limits_i e_i  \f]
			  *  Where \f$ e_i \f$ are the elements of the square error vector as computed by computeSquareErrorVector
			  * \sa squareErrorVector, overallSquareErrorAndPoints
			  */
			float overallSquareError( const CPose2D &q ) const;

			/** Computes the overall square error between the 2D points in the list of correspondences, given the 2D transformation "q", and return the transformed points as well.
			  *    \f[ \sum\limits_i e_i  \f]
			  *  Where \f$ e_i \f$ are the elements of the square error vector as computed by computeSquareErrorVector
			  * \sa squareErrorVector
			  */
			float overallSquareErrorAndPoints(
				const CPose2D &q,
				vector_float &xs,
				vector_float &ys ) const;


			/**  Returns a vector with the square error between each pair of correspondences in the list, given the 2D transformation "q"
			  *    Each element \f$ e_i \f$ is the square distance between the "this" (global) point and the "other" (local) point transformed through "q":
			  *    \f[ e_i = | x_{this} -  q \oplus x_{other}  |^2 \f]
			  * \sa overallSquareError
			  */
			void  squareErrorVector(const CPose2D &q, vector_float &out_sqErrs ) const;

			/**  Returns a vector with the square error between each pair of correspondences in the list and the transformed "other" (local) points, given the 2D transformation "q"
			  *    Each element \f$ e_i \f$ is the square distance between the "this" (global) point and the "other" (local) point transformed through "q":
			  *    \f[ e_i = | x_{this} -  q \oplus x_{other}  |^2 \f]
			  * \sa overallSquareError
			  */
			void  squareErrorVector(
				const CPose2D &q,
				vector_float &out_sqErrs,
				vector_float &xs,
				vector_float &ys ) const;

			/** Test whether the given pair "p" is within the pairings */
			bool contains (const TMatchingPair &p) const;
		};

		/** A comparison operator, for sorting lists of TMatchingPair's, first order by this_idx, if equals, by other_idx   */
		bool BASE_IMPEXP operator < (const TMatchingPair& a, const TMatchingPair& b);

		/** A comparison operator  */
		bool BASE_IMPEXP operator == (const TMatchingPair& a,const TMatchingPair& b);

		/** A comparison operator */
		bool BASE_IMPEXP operator == (const TMatchingPairList& a,const TMatchingPairList& b);


	} // End of namespace
} // end of namespace
#endif
