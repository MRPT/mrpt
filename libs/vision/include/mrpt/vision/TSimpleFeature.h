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
#ifndef _mrpt_vision_TSimpleFeature_H
#define _mrpt_vision_TSimpleFeature_H

#include <mrpt/utils/TPixelCoord.h>
#include <mrpt/math/KDTreeCapable.h>
#include <mrpt/vision/types.h>

#include <mrpt/vision/link_pragmas.h>

namespace mrpt
{
	namespace vision
	{
		/** A simple structure for representing one image feature (without descriptor nor patch) - This is
		  *  the template which allows you to select if pixels are represented as integers or floats.
		  *  \sa TSimpleFeature, TSimpleFeaturef
		  */
		template <typename PIXEL_COORD_TYPE>
		struct TSimpleFeature_templ
		{
			/** Constructor that only sets the pt.{x,y} values, leaving all other values to *undefined values*. */
			template <typename COORD_TYPE>
			inline TSimpleFeature_templ(const COORD_TYPE x, const COORD_TYPE y) : pt(x,y) { }

			PIXEL_COORD_TYPE    pt;             //!< Coordinates in the image
			TFeatureID          ID;             //!< ID of the feature
			TFeatureTrackStatus	track_status;	//!< Status of the feature tracking process
			float				response;		//!< A measure of the "goodness" of the feature (typically, the KLT_response value)
			int					octave;			//!< The image octave the image was found in: 0=original image, 1=1/2 image, 2=1/4 image, etc.
		};

		/** A simple structure for representing one image feature (without descriptor nor patch).
		  *  \sa TSimpleFeaturef, CFeature, TSimpleFeatureList
		  */
		typedef TSimpleFeature_templ<mrpt::utils::TPixelCoord>   TSimpleFeature;

		/** A version of  TSimpleFeature with subpixel precision */
		typedef TSimpleFeature_templ<mrpt::utils::TPixelCoordf>  TSimpleFeaturef;


		/** A list of image features using the structure TSimpleFeature for each feature - capable of KD-tree computations
		  *  Users normally use directly the typedef's: TSimpleFeatureList & TSimpleFeaturefList
		  */
		template <typename FEATURE>
		struct TSimpleFeatureList_templ : public mrpt::math::KDTreeCapable
		{
		public:
			typedef std::vector<FEATURE> TFeatureVector;

			/** Returns a const ref to the actual std::vector<> container */
			const TFeatureVector& getVector() const { return m_feats; }

			/** Get a reference to the nearest feature to the a given 2D point (version returning distance to closest feature in "max_dist")
			*   \param x [IN] The query point x-coordinate
			*   \param y [IN] The query point y-coordinate
			*   \param max_dist [IN/OUT] At input: The maximum distance to search for. At output: The actual distance to the feature.
			*  \return The 0-based index of the found feature, or std::string::npos if none found.
			*  \note See also all the available KD-tree search methods, listed in mrpt::math::KDTreeCapable
			*/
			size_t nearest( const float x, const float y, double &max_dist ) const
			{
				if (!this->empty()) {
					float closest_x,closest_y, closest_sqDist;
					// Look for the closest feature using KD-tree look up:
					const size_t closest_idx = this->kdTreeClosestPoint2D(x,y,closest_x,closest_y,closest_sqDist);
					float closest_dist = std::sqrt(closest_sqDist);
					if (closest_dist<=max_dist) {
						max_dist = closest_dist;
						return closest_idx;
					}
				}
				return std::string::npos;
			} // end nearest

			/** Call this when the list of features has been modified so the KD-tree is marked as outdated. */
			inline void mark_kdtree_as_outdated() const { kdtree_mark_as_outdated(); }

			/** @name Method and datatypes to emulate a STL container
			    @{ */
			typedef typename TFeatureVector::iterator iterator;
			typedef typename TFeatureVector::const_iterator const_iterator;

			typedef typename TFeatureVector::reverse_iterator reverse_iterator;
			typedef typename TFeatureVector::const_reverse_iterator const_reverse_iterator;

			inline iterator begin() { return m_feats.begin(); }
			inline iterator end() { return m_feats.end(); }
			inline const_iterator begin() const { return m_feats.begin(); }
			inline const_iterator end() const { return m_feats.end(); }

			inline reverse_iterator rbegin() { return m_feats.rbegin(); }
			inline reverse_iterator rend() { return m_feats.rend(); }
			inline const_reverse_iterator rbegin() const { return m_feats.rbegin(); }
			inline const_reverse_iterator rend() const { return m_feats.rend(); }

			inline iterator erase(const iterator it)  { mark_kdtree_as_outdated(); return m_feats.erase(it); }

			inline bool empty() const  { return m_feats.empty(); }
			inline size_t size() const { return m_feats.size(); }

			inline void clear() { m_feats.clear(); mark_kdtree_as_outdated(); }
			inline void resize(size_t N) { m_feats.resize(N); mark_kdtree_as_outdated(); }
			inline void reserve(size_t N) { m_feats.reserve(N); }

			inline void push_front(const FEATURE &f) { mark_kdtree_as_outdated();  m_feats.push_front(f); }
			inline void push_back(const FEATURE &f) { mark_kdtree_as_outdated();  m_feats.push_back(f); }

			inline void push_front_fast(const FEATURE &f) { m_feats.push_front(f); }
			inline void push_back_fast (const FEATURE &f) { m_feats.push_back(f); }

			inline void push_front_fast(const int x, const int y) { m_feats.push_front(FEATURE(x,y)); }
			inline void push_back_fast (const int x, const int y) { m_feats.push_back (FEATURE(x,y)); }

			inline       FEATURE & operator [](const unsigned int index)        { return m_feats[index]; }
			inline const FEATURE & operator [](const unsigned int index) const  { return m_feats[index]; }

			/** @} */

			/** @name Virtual methods that MUST be implemented by children classes of KDTreeCapable
			    @{ */
			/** Must return the number of data points */
			virtual size_t kdtree_get_point_count() const { return size(); }
			/** Must fill out the data points in "data", such as the i'th point will be stored in (data[i][0],...,data[i][nDims-1]). */
			virtual void kdtree_fill_point_data(ANNpointArray &data, const int nDims) const
			{
				ASSERTMSG_(nDims==2, "TSimpleFeatureList_templ only supports 2D KD-trees.")
				const size_t N=m_feats.size();
				for (size_t i=0;i<N;i++)
				{
					data[i][0] = m_feats[i].pt.x;
					data[i][1] = m_feats[i].pt.y;
				}
			}
			/** @} */
		private:
			TFeatureVector  m_feats; //!< The actual container with the list of features

		}; // end of class

		/** A list of image features using the structure TSimpleFeature for each feature - capable of KD-tree computations */
		typedef TSimpleFeatureList_templ<TSimpleFeature>  TSimpleFeatureList;

		/** A list of image features using the structure TSimpleFeaturef for each feature - capable of KD-tree computations */
		typedef TSimpleFeatureList_templ<TSimpleFeaturef> TSimpleFeaturefList;




		/** A helper struct to sort keypoints by their response: It can be used with these types:
		  *	  - std::vector<cv::KeyPoint>
		  *	  - mrpt::vision::TSimpleFeatureList
		  */
		template <typename FEATURE_LIST>
		struct KeypointResponseSorter : public std::binary_function<size_t,size_t,bool>
		{
			const FEATURE_LIST &m_data;
			KeypointResponseSorter( const FEATURE_LIST &data ) : m_data(data) { }
			bool operator() (size_t k1, size_t k2 ) const {
				return (m_data[k1].response > m_data[k2].response);
			}
		};

	} // end of namespace

} // end of namespace

#endif

