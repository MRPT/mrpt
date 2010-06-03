/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
#ifndef CPOINT3D_H
#define CPOINT3D_H

#include <mrpt/poses/CPoint.h>

namespace mrpt
{
namespace poses
{
    class CPose3D;

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPoint3D, CPoint )

	/** A class used to store a 3D point.
	 *
	 *  For a complete description of Points/Poses, see mrpt::poses::CPoseOrPoint, or refer
	 *    to the <a href="http://www.mrpt.org/2D_3D_Geometry">2D/3D Geometry tutorial</a> in the wiki.
	 *

<div align=center>

<table class=MsoTableGrid border=1 cellspacing=0 cellpadding=0
 style='border-collapse:collapse;border:none'>
 <tr>
  <td width=576 colspan=2 style='width:432.2pt;border:solid windowtext 1.0pt;
  background:#E6E6E6;padding:0cm 5.4pt 0cm 5.4pt'>
  <p   align=center style='text-align:center'>poses::CPoint3D</p>
  </td>
 </tr>
 <tr>
  <td width=288 style='width:216.1pt;border:solid windowtext 1.0pt;border-top:
  none;padding:0cm 5.4pt 0cm 5.4pt'>
  <p   align=center style='text-align:center'>Homogeneous
  transfomation matrix</p>
  </td>
  <td width=288 style='width:216.1pt;border-top:none;border-left:none;
  border-bottom:solid windowtext 1.0pt;border-right:solid windowtext 1.0pt;
  padding:0cm 5.4pt 0cm 5.4pt'>
  <p   align=center style='text-align:center'>Spatial
  representation</p>
  </td>
 </tr>
 <tr style='height:108.3pt'>
  <td width=288 style='width:216.1pt;border:solid windowtext 1.0pt;border-top:
  none;padding:0cm 5.4pt 0cm 5.4pt;height:108.3pt'>
  <div align=center>
  <table  Table border=0 cellspacing=0 cellpadding=0 width="46%"
   style='width:46.84%;border-collapse:collapse'>
   <tr style='height:16.5pt'>
    <td width=32 style='width:24.0pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
    <p   align=center style='text-align:center'>1</p>
    </td>
    <td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
    <p   align=center style='text-align:center'>0</p>
    </td>
    <td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
    <p   align=center style='text-align:center'>0</p>
    </td>
    <td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
    <p   align=center style='text-align:center'>x</p>
    </td>
   </tr>
   <tr style='height:16.5pt'>
    <td width=32 style='width:24.0pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
    <p   align=center style='text-align:center'>0</p>
    </td>
    <td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
    <p   align=center style='text-align:center'>1</p>
    </td>
    <td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
    <p   align=center style='text-align:center'>0</p>
    </td>
    <td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
    <p   align=center style='text-align:center'>y</p>
    </td>
   </tr>
   <tr style='height:16.5pt'>
    <td width=32 style='width:24.0pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
    <p   align=center style='text-align:center'>0</p>
    </td>
    <td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
    <p   align=center style='text-align:center'>0</p>
    </td>
    <td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
    <p   align=center style='text-align:center'>1</p>
    </td>
    <td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
    <p   align=center style='text-align:center'>z</p>
    </td>
   </tr>
   <tr style='height:16.5pt'>
    <td width=32 style='width:24.0pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
    <p   align=center style='text-align:center'>0</p>
    </td>
    <td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
    <p   align=center style='text-align:center'>0</p>
    </td>
    <td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
    <p   align=center style='text-align:center'>0</p>
    </td>
    <td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
    <p   align=center style='text-align:center'>1</p>
    </td>
   </tr>
  </table>
  </div>
  <p   align=center style='text-align:center'></p>
  </td>
  <td width=288 style='width:216.1pt;border-top:none;border-left:none;
  border-bottom:solid windowtext 1.0pt;border-right:solid windowtext 1.0pt;
  padding:0cm 5.4pt 0cm 5.4pt;height:108.3pt'>
  <p   align=center style='text-align:center'><img src="CPoint3D.gif"></p>
  </td>
 </tr>
</table>

</div>


	  * \sa CPoseOrPoint,CPose, CPoint
	 */
	class BASE_IMPEXP  CPoint3D : public CPoint
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CPoint3D )

	 public:
		 /** Constructor for initializing point coordinates.
		  */
		 CPoint3D(const double x=0,const double y=0,const double z=0);

		 /** Constructor from an CPoint2D object.
		  */
		 CPoint3D( const CPoint2D &);

		 /** Constructor from an CPose3D object.
		  */
		 explicit CPoint3D( const CPose3D &);

		 /** Constructor from an CPose2D object.
		  */
		 explicit CPoint3D( const CPose2D &);

		 /** Constructor from lightweight object.
		  */
		 CPoint3D(const mrpt::math::TPoint3D &);

		 /** Returns this point as seen from "b", i.e. result = this - b
		   */
		 CPoint3D  operator - (const CPose3D& b) const;

		 /** Returns this point minus point "b", i.e. result = this - b
		   */
		 CPoint3D  operator - (const CPoint3D& b) const;

		 /** Returns this point plus point "b", i.e. result = this + b
		   */
		 CPoint3D  operator + (const CPoint3D& b) const;

		 /** Returns this point plus pose "b", i.e. result = this + b
		   */
		 CPose3D	operator + (const CPose3D& b) const;

		 /** Return the pose or point as a 1x3 vector: [x y z] */
		 void getAsVector(vector_double &v) const;

		 /** Returns the corresponding 4x4 homogeneous transformation matrix for the point(translation) or pose (translation+orientation).
		   * \sa getInverseHomogeneousMatrix
		   */
		 void  getHomogeneousMatrix(CMatrixDouble44 & out_HM ) const
		 {
			out_HM.unit();
			out_HM.get_unsafe(0,3)=m_x;
			out_HM.get_unsafe(1,3)=m_y;
			out_HM.get_unsafe(2,3)=m_z;
		 }

		 /** Returns a human-readable textual representation of the object (eg: "[0.02 1.04 -0.8]" )
		   * \sa fromString
		   */
		void asString(std::string &s) const { s = mrpt::format("[%f %f %f]",m_x,m_y,m_z); }
		inline std::string asString() const { std::string s; asString(s); return s; }

		 /** Set the current object value from a string generated by 'asString' (eg: "[0.02 1.04 -0.8]" )
		   * \sa asString
		   * \exception std::exception On invalid format
		   */
		 void fromString(const std::string &s) {
		 	CMatrixDouble  m;
		 	if (!m.fromMatlabStringFormat(s)) THROW_EXCEPTION("Malformed expression in ::fromString");
		 	ASSERTMSG_(mrpt::math::size(m,1)==1 && mrpt::math::size(m,2)==3, "Wrong size of vector in ::fromString");
		 	m_x = m.get_unsafe(0,0);
		 	m_y = m.get_unsafe(0,1);
		 	m_z = m.get_unsafe(0,2);
		 }

		 inline const double &operator[](unsigned int i)const
		 {
		 	switch(i)
		 	{
		 		case 0:return m_x;
		 		case 1:return m_y;
		 		case 2:return m_z;
		 		default:
		 		throw std::runtime_error("CPoint3D::operator[]: Index of bounds.");
		 	}
		 }
		 inline double &operator[](unsigned int i)
		 {
		 	switch(i)
		 	{
		 		case 0:return m_x;
		 		case 1:return m_y;
		 		case 2:return m_z;
		 		default:
		 		throw std::runtime_error("CPoint3D::operator[]: Index of bounds.");
		 	}
		 }
		/** @name STL-like methods and typedefs 
		   @{   */
		typedef double         value_type;		//!< The type of the elements
		typedef double&        reference;
		typedef const double&  const_reference;
		typedef std::size_t    size_type;
		typedef std::ptrdiff_t difference_type;


		// size is constant
		enum { static_size = 3 };
		static inline size_type size() { return static_size; }
		static inline bool empty() { return false; }
		static inline size_type max_size() { return static_size; }
		static inline void resize(const size_t n) { if (n!=static_size) throw std::logic_error(format("Try to change the size of CPoint3D to %u.",static_cast<unsigned>(n))); }
		/** @} */

	}; // End of class def.


	std::ostream BASE_IMPEXP & operator << (std::ostream& o, const CPoint3D& p);

	bool BASE_IMPEXP operator==(const CPoint3D &p1,const CPoint3D &p2);
	bool BASE_IMPEXP operator!=(const CPoint3D &p1,const CPoint3D &p2);

	} // End of namespace
} // End of namespace

#endif
