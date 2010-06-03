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

#ifndef opengl_CSetOfLines_H
#define opengl_CSetOfLines_H

#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/utils/stl_extensions.h>

namespace mrpt
{
	namespace opengl
	{
		using mrpt::math::TPoint3D;
		using mrpt::math::TSegment3D;
		class OPENGL_IMPEXP CSetOfLines;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CSetOfLines, CRenderizable, OPENGL_IMPEXP )

		/** A set of independent lines (or segments), one line with its own start and end positions (X,Y,Z).
		  *  \sa opengl::COpenGLScene
		  */
		class OPENGL_IMPEXP CSetOfLines : public CRenderizable
		{
			DEFINE_SERIALIZABLE( CSetOfLines )
		protected:
			std::vector<TSegment3D> mSegments;
            float			mLineWidth;

		public:
			/**
			  * Clear the list of segments
			  */
			inline void clear()	{
				mSegments.clear();
			}
			/**
			  * Sets the width with which lines will be drawn.
			  */
			inline void setLineWidth(float w) {
				mLineWidth=w;
			}
			/**
			  * Gets the width with which lines are drawn.
			  */
			float getLineWidth() const {
				return mLineWidth;
			}
			/**
			  * Appends a line to the set.
			  */
			inline void appendLine(const mrpt::math::TSegment3D &sgm)	{
				mSegments.push_back(sgm);
			}
			/**
			  * Appends a line to the set, given the coordinates of its bounds.
			  */
			inline void appendLine(float x0,float y0,float z0,float x1,float y1,float z1)	{
				appendLine(TSegment3D(TPoint3D(x0,y0,z0),TPoint3D(x1,y1,z1)));
			}
			/**
			  * Appends any iterable collection of lines to the set. Note that this includes another CSetOfLines.
			  * \sa appendLine
			  */
			template<class T> inline void appendLines(const T &sgms)	{
				mSegments.insert(mSegments.end(),sgms.begin(),sgms.end());
			}
			/**
			  * Appends certain amount of lines, located between two iterators, into the set.
			  * \sa appendLine
			  */
			template<class T_it> inline void appendLines(const T_it &begin,const T_it &end)	{
				mSegments.reserve(mSegments.size()+(end-begin));
				mSegments.insert(mSegments.end(),begin,end);
			}
			/**
			  * Resizes the set.
			  * \sa reserve
			  */
			void resize(size_t nLines)	{
				mSegments.resize(nLines);
			}
			/**
			  * Reserves an amount of lines to the set. This method should be used when some known amount of lines is going to be inserted, so that only a memory allocation is needed.
			  * \sa resize
			  */
			void reserve(size_t r)	{
				mSegments.reserve(r);
			}
			/**
			  * Inserts a line, given its bounds. Works with any pair of objects with access to x, y and z members.
			  */
			template<class T,class U> inline void appendLine(T p0,U p1)	{
				appendLine(p0.x,p0.y,p0.z,p1.x,p1.y,p1.z);
			}
			/**
			  * Returns the total count of lines in this set.
			  */
			inline size_t getLineCount() const	{
				return mSegments.size();
			}
			/**
			  * Sets a specific line in the set, given its index.
			  * \sa appendLine
			  */
			void setLineByIndex(size_t index,const TSegment3D &segm);
			/**
			  * Sets a specific line in the set, given its index.
			  * \sa appendLine
			  */
			inline void setLineByIndex(size_t index,float x0,float y0,float z0,float x1,float y1,float z1)	{
				setLineByIndex(index,TSegment3D(TPoint3D(x0,y0,z0),TPoint3D(x1,y1,z1)));
			}
			/**
			  * Class factory
			  */
			inline static CSetOfLinesPtr Create(const std::vector<TSegment3D> &sgms)	{
				return CSetOfLinesPtr(new CSetOfLines(sgms));
			}
			/** Render
			  */
			void  render() const;

			//Iterator management
			/**
			  * Const iterator to the set.
			  */
			typedef std::vector<TSegment3D>::const_iterator const_iterator;
			/**
			  * Const reverse iterator to the set.
			  */
			typedef std::vector<TSegment3D>::const_reverse_iterator const_reverse_iterator;
			/**
			  * Beginning const iterator.
			  * \sa end,rbegin,rend
			  */
			inline const_iterator begin() const	{
				return mSegments.begin();
			}
			/**
			  * Ending const iterator.
			  * \sa begin,rend,rbegin
			  */
			inline const_iterator end() const	{
				return mSegments.end();
			}
			/**
			  * Beginning const reverse iterator (actually, accesses the end of the set).
			  * \sa rend,begin,end
			  */
			inline const_reverse_iterator rbegin() const	{
				return mSegments.rbegin();
			}
			/**
			  * Ending const reverse iterator (actually, refers to the starting point of the set).
			  * \sa rbegin,end,begin
			  */
			inline const_reverse_iterator rend() const	{
				return mSegments.rend();
			}

		private:
			/** Constructor
			  */
			CSetOfLines():mSegments(),mLineWidth(1.0)	{}
			/**
			  * Constructor with a initial set of lines.
			  */
			CSetOfLines(const std::vector<TSegment3D> &sgms):mSegments(sgms),mLineWidth(1.0)	{}
			/**
			  * Private, virtual destructor: only can be deleted from smart pointers.
			  */
			virtual ~CSetOfLines() { }
		};
		/** Inserts a set of segments into the list. Allows call chaining.
		  * \sa mrpt::opengl::CSetOfLines::appendLines
		  */
		template<class T> inline CSetOfLinesPtr &operator<<(CSetOfLinesPtr &l,const T &s)	{
			l->appendLines(s.begin(),s.end());
			return l;
		}
		/** Inserts a segment into the list. Allows call chaining.
		  * \sa mrpt::opengl::CSetOfLines::appendLine(const TSegment &)
		  */
		template<> inline CSetOfLinesPtr &operator<<(CSetOfLinesPtr &l,const mrpt::math::TSegment3D &s)	{
			l->appendLine(s);
			return l;
		}
	} // end namespace

} // End of namespace


#endif
