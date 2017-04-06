/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/system/datetime.h>
#include <mrpt/utils/TEnumType.h>
#include <mrpt/poses/SE_traits.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/poses/poses_frwds.h>
#include <mrpt/base/link_pragmas.h>

namespace mrpt {
namespace poses {

	/** Type to select the interpolation method in CPoseInterpolatorBase derived classes.
	  *  - imSpline: Spline interpolation using 4 points (2 before + 2 after the query point).
	  *  - imLinear2Neig: Linear interpolation between the previous and next neightbour.
	  *  - imLinear4Neig: Linear interpolation using the linear fit of the 4 closer points (2 before + 2 after the query point).
	  *  - imSSLLLL : Use Spline for X and Y, and Linear Least squares for Z, yaw, pitch and roll.
	  *  - imSSLSLL : Use Spline for X, Y and yaw, and Linear Lesat squares for Z, pitch and roll.
	  *  - imLinearSlerp: Linear for X,Y,Z, Slerp for 3D angles.
	  *  - imSplineSlerp: Spline for X,Y,Z, Slerp for 3D angles.
	  * \ingroup interpolation_grp poses_grp
	  */
	enum TInterpolatorMethod
	{
		imSpline = 0,
		imLinear2Neig,
		imLinear4Neig,
		imSSLLLL,
		imSSLSLL,
		imLinearSlerp,
		imSplineSlerp
	};


	/** Base class for SE(2)/SE(3) interpolators. See docs for derived classes.
	* \ingroup interpolation_grp poses_grp
	*/
	template <int DIM>
	class BASE_IMPEXP CPoseInterpolatorBase
	{
	public:
		CPoseInterpolatorBase(); //!< Default ctor: empty sequence of poses

		/** @name Type definitions and STL-like container interface
		 * @{ */

		typedef typename mrpt::poses::SE_traits<DIM>::lightweight_pose_t pose_t;  //!< TPose2D or TPose3D
		typedef typename mrpt::poses::SE_traits<DIM>::pose_t             cpose_t; //!< CPose2D or CPose3D
		typedef typename mrpt::poses::SE_traits<DIM>::point_t            point_t; //!< TPoint2D or TPoint3D

		typedef std::pair<mrpt::system::TTimeStamp,pose_t> TTimePosePair;
		typedef std::map<mrpt::system::TTimeStamp,pose_t>  TPath;
		typedef typename TPath::iterator       iterator;
		typedef typename TPath::const_iterator const_iterator;
		typedef typename TPath::reverse_iterator       reverse_iterator;
		typedef typename TPath::const_reverse_iterator const_reverse_iterator;

		inline iterator begin() { return m_path.begin(); }
		inline const_iterator begin() const { return m_path.begin(); }
		inline const_iterator cbegin() const {
#if MRPT_HAS_CXX11
			return m_path.cbegin();
#else
			return m_path.begin();
#endif
		}

		inline iterator end() { return m_path.end(); }
		inline const_iterator end() const { return m_path.end(); }
		inline const_iterator cend() const {
#if MRPT_HAS_CXX11
			return m_path.cend();
#else
			return m_path.end();
#endif
		}

		inline reverse_iterator rbegin() { return m_path.rbegin(); }
		inline const_reverse_iterator rbegin() const { return m_path.rbegin(); }

		inline reverse_iterator rend() { return m_path.rend(); }
		inline const_reverse_iterator rend() const { return m_path.rend(); }

		iterator lower_bound( const mrpt::system::TTimeStamp & t) { return m_path.lower_bound(t); }
		const_iterator lower_bound( const mrpt::system::TTimeStamp & t) const { return m_path.lower_bound(t); }

		iterator upper_bound( const mrpt::system::TTimeStamp & t) { return m_path.upper_bound(t); }
		const_iterator upper_bound( const mrpt::system::TTimeStamp & t) const { return m_path.upper_bound(t); }

		iterator erase(iterator element_to_erase) { m_path.erase(element_to_erase++); return element_to_erase; }

		size_t size() const { return m_path.size(); }
		bool empty() const { return m_path.empty(); }

		iterator find(const mrpt::system::TTimeStamp & t) { return m_path.find(t); }
		const_iterator find(const mrpt::system::TTimeStamp & t) const { return m_path.find(t); }
		/** @} */

		/** Inserts a new pose in the sequence.
		  *  It overwrites any previously existing pose at exactly the same time.
		  */
		void insert( mrpt::system::TTimeStamp t, const pose_t  &p);
		void insert( mrpt::system::TTimeStamp t, const cpose_t &p); //!< Overload (slower)

		/** Returns the pose at a given time, or interpolates using splines if there is not an exact match.
		  * \param t The time of the point to interpolate.
		  * \param out_interp The output interpolated pose.
		  * \param out_valid_interp Whether there was information enough to compute the interpolation.
		  * \return A reference to out_interp
		  */
		pose_t  &interpolate(mrpt::system::TTimeStamp t, pose_t  &out_interp, bool &out_valid_interp ) const;
		cpose_t &interpolate(mrpt::system::TTimeStamp t, cpose_t &out_interp, bool &out_valid_interp) const; //!< \overload (slower)

		void clear(); //!< Clears the current sequence of poses

		/** Set value of the maximum time to consider interpolation.
		 * If set to a negative value, the check is disabled (default behavior). */
		void setMaxTimeInterpolation( double time );
		double getMaxTimeInterpolation(); //!< Set value of the maximum time to consider interpolation

		/** Get the previous CPose3D in the map with a minimum defined distance.
		* \return true if pose was found, false otherwise */
		bool getPreviousPoseWithMinDistance( const mrpt::system::TTimeStamp &t, double distance, pose_t  &out_pose );
		bool getPreviousPoseWithMinDistance( const mrpt::system::TTimeStamp &t, double distance, cpose_t &out_pose ); //!< \overload (slower)

		/** Saves the points in the interpolator to a text file, with this format:
		  *  Each row contains these elements separated by spaces:
		  *	- Timestamp: As a "double time_t" (see mrpt::system::timestampTotime_t).
		  *	- x y z: The 3D position in meters.
		  *	- yaw pitch roll: The angles, in radians
		  * \sa loadFromTextFile
		  * \return true on success, false on any error.
		  */
		bool saveToTextFile(const std::string &s) const;

		/** Saves the points in the interpolator to a text file, with the same format that saveToTextFile, but interpolating the path with the given period in seconds.
		  * \sa loadFromTextFile
		  * \return true on success, false on any error.
		  */
		bool saveInterpolatedToTextFile(const std::string &s, double period) const;

		/** Loads from a text file, in the format described by saveToTextFile.
		  * \return true on success, false on any error.
		  * \exception std::exception On invalid file format
		  */
		bool loadFromTextFile(const std::string &s);

		/** Computes the bounding box in all Euclidean coordinates of the whole path. \exception std::exception On empty path */
		void getBoundingBox(point_t &minCorner, point_t &maxCorner) const;

		void setInterpolationMethod( TInterpolatorMethod method); //!< Change the method used to interpolate the robot path. The default method at construction is "imSpline". \sa getInterpolationMethod()
		TInterpolatorMethod getInterpolationMethod() const; //!< Returns the currently set interpolation method.  \sa setInterpolationMethod()

		/** Filters by averaging one of the components of the pose data within the interpolator. The width of the filter is set by the number of samples.
		  * \param component	[IN]	The index of the component to filter: 0 (x), 1 (y), 2 (z), 3 (yaw), 4 (pitch) or 5 (roll)
		  * \param samples		[IN]	The width of the average filter.
		  */
		void filter( unsigned int component, unsigned int samples );


	protected:
		TPath m_path; //!< The sequence of poses
		double               maxTimeInterpolation; //!< Maximum time considered to interpolate. If the difference between the desired timestamp where to interpolate and the next timestamp stored in the map is bigger than this value, the interpolation will not be done.
		TInterpolatorMethod  m_method;

		void impl_interpolation(
			const mrpt::math::CArrayDouble<4> &ts,
			const TTimePosePair p1,const TTimePosePair p2,const TTimePosePair p3,const TTimePosePair p4,
			const TInterpolatorMethod method,double td,pose_t &out_interp) const;

	}; // End of class def.

} // End of namespace

// Specializations MUST occur at the same namespace:
namespace utils
{
	template <>
	struct TEnumTypeFiller<mrpt::poses::TInterpolatorMethod>
	{
		typedef mrpt::poses::TInterpolatorMethod enum_t;
		static void fill(bimap<enum_t,std::string>  &m_map)
		{
			m_map.insert(poses::imSpline,          "imSpline");
			m_map.insert(poses::imLinear2Neig,     "imLinear2Neig");
			m_map.insert(poses::imLinear4Neig,     "imLinear4Neig");
			m_map.insert(poses::imSSLLLL,          "imSSLLLL");
			m_map.insert(poses::imLinearSlerp,     "imLinearSlerp");
			m_map.insert(poses::imSplineSlerp,     "imSplineSlerp");
		}
	};
} // End of namespace
} // End of namespace
