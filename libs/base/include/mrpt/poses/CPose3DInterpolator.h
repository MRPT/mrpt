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
#ifndef CPose3DInterpolator_H
#define CPose3DInterpolator_H

#include <mrpt/poses/CPose.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/stl_extensions.h>
#include <mrpt/utils/TEnumType.h>

namespace mrpt
{
	namespace poses
	{
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPose3DInterpolator, mrpt::utils::CSerializable )

		typedef std::pair<mrpt::system::TTimeStamp, mrpt::poses::CPose3D> TTimePosePair;

		/** A trajectory in time and in 6D (CPose3D) that interpolates using splines the intervals between a set of given time-referenced poses.
		  *   To insert new points into the sequence, use the "insert" method, and for getting an interpolated point, use "interpolate" method. For example:
		  * \code
		  * CPose3DInterpolator		path;
		  *
		  * path.setInterpolationMethod( CPose3DInterpolator::imSplineSlerp );
		  *
		  * path.insert( t0, CPose3D(...) );
		  * path.insert( t1, CPose3D(...) );
		  * path.insert( t2, CPose3D(...) );
		  * path.insert( t3, CPose3D(...) );
		  *
		  * CPose3D p;
		  * bool valid;
		  *
		  * cout << "Pose at t: " << path.interpolate(t,p,valid) << endl;
		  * \endcode
		  *
		  *  Time is represented with mrpt::system::TTimeStamp. See mrpt::system for methods and utilities to manage these time references.
		  *
		  *  See TInterpolatorMethod for the list of interpolation methods. The default method at constructor is "imLinearSlerp".
		  *
		  * \sa CPoseOrPoint
		 * \ingroup interpolation_grp poses_grp
		 */
		class BASE_IMPEXP CPose3DInterpolator : public mrpt::utils::CSerializable
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CPose3DInterpolator )

		 private:
			 typedef std::map< mrpt::system::TTimeStamp, CPose3D > TPath;
			 TPath	m_path;		//!< The sequence of poses

		 public:
			 typedef TPath::iterator		iterator;
			 typedef TPath::const_iterator const_iterator;
			 typedef TPath::reverse_iterator		reverse_iterator;
			 typedef TPath::const_reverse_iterator const_reverse_iterator;

			 /** Type to select the interpolation method in CPose3DInterpolator::setInterpolationMethod
			   *  - imSpline: Spline interpolation using 4 points (2 before + 2 after the query point).
			   *  - imLinear2Neig: Linear interpolation between the previous and next neightbour.
			   *  - imLinear4Neig: Linear interpolation using the linear fit of the 4 closer points (2 before + 2 after the query point).
			   *  - imSSLLLL : Use Spline for X and Y, and Linear Least squares for Z, yaw, pitch and roll.
			   *  - imSSLSLL : Use Spline for X, Y and yaw, and Linear Lesat squares for Z, pitch and roll.
			   *  - imLinearSlerp: Linear for X,Y,Z, Slerp for 3D angles.
			   *  - imSplineSlerp: Spline for X,Y,Z, Slerp for 3D angles.
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

			 inline iterator begin() { return m_path.begin(); }
			 inline const_iterator begin() const { return m_path.begin(); }

			 inline iterator end() { return m_path.end(); }
			 inline const_iterator end() const { return m_path.end(); }

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

			 /** Creates an empty interpolator (with no points).
			  */
			 CPose3DInterpolator();

			 /** Inserts a new pose in the sequence.
			   *  It overwrites any previously existing pose at exactly the same time.
			   */
			 void insert( mrpt::system::TTimeStamp t, const CPose3D &p);

			 /** Returns the pose at a given time, or interpolates using splines if there is not an exact match.
			   * \param t The time of the point to interpolate.
			   * \param out_interp The output interpolated pose.
			   * \param out_valid_interp Whether there was information enough to compute the interpolation.
			   * \return A reference to out_interp
			   */
			 CPose3D &interpolate( mrpt::system::TTimeStamp t, CPose3D &out_interp, bool &out_valid_interp ) const;

			 /** Clears the current sequence of poses */
			 void clear();

			 /** Set value of the maximum time to consider interpolation.
			   *  If set to a negative value, the check is disabled (default behavior).
			   */
			 void setMaxTimeInterpolation( double time );

			 /** Set value of the maximum time to consider interpolation */
			 double getMaxTimeInterpolation( );

			 /** Get the previous CPose3D in the map with a minimum defined distance
			   * \return true if pose was found, false otherwise.
			   */
			 bool getPreviousPoseWithMinDistance( const mrpt::system::TTimeStamp &t, double distance, CPose3D &out_pose );

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

			 /**  Computes the bounding box in X,Y,Z of the whole vehicle path.
			   * \exception std::exception On empty path
			   */
			 void getBoundingBox(CPoint3D &minCorner, CPoint3D &maxCorner) const;

			 /**  Computes the bounding box in X,Y,Z of the whole vehicle path.
			   * \exception std::exception On empty path
			   */
			 void getBoundingBox(mrpt::math::TPoint3D &minCorner, mrpt::math::TPoint3D &maxCorner) const;

			 /** Change the method used to interpolate the robot path.
			   *  The default method at construction is "imSpline".
			   * \sa getInterpolationMethod
			   */
			 void setInterpolationMethod( TInterpolatorMethod method);

			 /** Returns the currently set interpolation method.
			   * \sa setInterpolationMethod
			   */
			 TInterpolatorMethod getInterpolationMethod() const;

			 /** Filters by averaging one of the components of the CPose3D data within the interpolator. The width of the filter is set by the number of samples.
			   * \param component	[IN]	The index of the component to filter: 0 (x), 1 (y), 2 (z), 3 (yaw), 4 (pitch) or 5 (roll).
			   * \param samples		[IN]	The width of the average filter.
			   */
			 void filter( unsigned int component, unsigned int samples );


		private:
			 double maxTimeInterpolation; //!< Maximum time considered to interpolate. If the difference between the desired timestamp where to interpolate and the next timestamp stored in the map is bigger than this value, the interpolation will not be done.

			 TInterpolatorMethod 	m_method;

		}; // End of class def.

	} // End of namespace

	// Specializations MUST occur at the same namespace:
	namespace utils
	{
		template <>
		struct TEnumTypeFiller<poses::CPose3DInterpolator::TInterpolatorMethod>
		{
			typedef poses::CPose3DInterpolator::TInterpolatorMethod enum_t;
			static void fill(bimap<enum_t,std::string>  &m_map)
			{
				m_map.insert(poses::CPose3DInterpolator::imSpline,          "imSpline");
				m_map.insert(poses::CPose3DInterpolator::imLinear2Neig,     "imLinear2Neig");
				m_map.insert(poses::CPose3DInterpolator::imLinear4Neig,     "imLinear4Neig");
				m_map.insert(poses::CPose3DInterpolator::imSSLLLL,          "imSSLLLL");
				m_map.insert(poses::CPose3DInterpolator::imLinearSlerp,     "imLinearSlerp");
				m_map.insert(poses::CPose3DInterpolator::imSplineSlerp,     "imSplineSlerp");
			}
		};
	} // End of namespace
} // End of namespace

#endif
