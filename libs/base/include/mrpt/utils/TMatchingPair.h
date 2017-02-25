/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  TMatchingPair_H
#define  TMatchingPair_H

#include <string>
#include <vector>
#include <mrpt/base/link_pragmas.h>
#include <mrpt/poses/poses_frwds.h>

namespace mrpt
{
	namespace utils
	{

	// Pragma defined to ensure no structure packing, so we can use SSE2 vectorization on parts of this struture
#if defined(MRPT_IS_X86_AMD64)
#pragma pack(push,1)
#endif

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

#if defined(MRPT_IS_X86_AMD64)
#pragma pack(pop) // End of pack = 1
#endif


		typedef TMatchingPair*  TMatchingPairPtr;

		/** A list of TMatchingPair
		 * \ingroup mrpt_base_grp
		  */
		class BASE_IMPEXP TMatchingPairList : public std::vector<TMatchingPair>
		{
		public:
			/** Checks if the given index from the "other" map appears in the list. */
			bool  indexOtherMapHasCorrespondence(size_t idx)  const;
			/** Saves the correspondences to a text file */
			void  dumpToFile(const std::string &fileName) const;
			/** Saves the correspondences as a MATLAB script which draws them. */
			void saveAsMATLABScript( const std::string &filName ) const;

			/** Computes the overall square error between the 2D points in the list of correspondences, given the 2D transformation "q"
			  *    \f[ \sum\limits_i e_i  \f]
			  *  Where \f$ e_i \f$ are the elements of the square error vector as computed by computeSquareErrorVector
			  * \sa squareErrorVector, overallSquareErrorAndPoints
			  */
			float overallSquareError( const mrpt::poses::CPose2D &q ) const;

			/** Computes the overall square error between the 2D points in the list of correspondences, given the 2D transformation "q", and return the transformed points as well.
			  *    \f[ \sum\limits_i e_i  \f]
			  *  Where \f$ e_i \f$ are the elements of the square error vector as computed by computeSquareErrorVector
			  * \sa squareErrorVector
			  */
			float overallSquareErrorAndPoints(
				const mrpt::poses::CPose2D &q,
				std::vector<float> &xs,
				std::vector<float> &ys ) const;


			/**  Returns a vector with the square error between each pair of correspondences in the list, given the 2D transformation "q"
			  *    Each element \f$ e_i \f$ is the square distance between the "this" (global) point and the "other" (local) point transformed through "q":
			  *    \f[ e_i = | x_{this} -  q \oplus x_{other}  |^2 \f]
			  * \sa overallSquareError
			  */
			void  squareErrorVector(const mrpt::poses::CPose2D &q, std::vector<float> &out_sqErrs ) const;

			/**  Returns a vector with the square error between each pair of correspondences in the list and the transformed "other" (local) points, given the 2D transformation "q"
			  *    Each element \f$ e_i \f$ is the square distance between the "this" (global) point and the "other" (local) point transformed through "q":
			  *    \f[ e_i = | x_{this} -  q \oplus x_{other}  |^2 \f]
			  * \sa overallSquareError
			  */
			void  squareErrorVector(
				const mrpt::poses::CPose2D &q,
				std::vector<float> &out_sqErrs,
				std::vector<float> &xs,
				std::vector<float> &ys ) const;

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
