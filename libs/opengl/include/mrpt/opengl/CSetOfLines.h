/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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

#ifndef opengl_CSetOfLines_H
#define opengl_CSetOfLines_H

#include <mrpt/opengl/CRenderizableDisplayList.h>
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
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CSetOfLines, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** A set of independent lines (or segments), one line with its own start and end positions (X,Y,Z).
		  *  \sa opengl::COpenGLScene
		  *  
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *   <tr> <td> mrpt::opengl::CSetOfLines </td> <td> \image html preview_CSetOfLines.png </td> </tr>
		  *  </table>
		  *  </div>
		  *  
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CSetOfLines : public CRenderizableDisplayList
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
				CRenderizableDisplayList::notifyChange();
			}
			/**
			  * Sets the width with which lines will be drawn.
			  */
			inline void setLineWidth(float w) {
				mLineWidth=w;
				CRenderizableDisplayList::notifyChange();
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
				CRenderizableDisplayList::notifyChange();
			}
			/**
			  * Appends a line to the set, given the coordinates of its bounds.
			  */
			inline void appendLine(float x0,float y0,float z0,float x1,float y1,float z1)	{
				appendLine(TSegment3D(TPoint3D(x0,y0,z0),TPoint3D(x1,y1,z1)));
				CRenderizableDisplayList::notifyChange();
			}
			/**
			  * Appends any iterable collection of lines to the set. Note that this includes another CSetOfLines.
			  * \sa appendLine
			  */
			template<class T> inline void appendLines(const T &sgms)	{
				mSegments.insert(mSegments.end(),sgms.begin(),sgms.end());
				CRenderizableDisplayList::notifyChange();
			}
			/**
			  * Appends certain amount of lines, located between two iterators, into the set.
			  * \sa appendLine
			  */
			template<class T_it> inline void appendLines(const T_it &begin,const T_it &end)	{
				mSegments.reserve(mSegments.size()+(end-begin));
				mSegments.insert(mSegments.end(),begin,end);
				CRenderizableDisplayList::notifyChange();
			}
			/**
			  * Resizes the set.
			  * \sa reserve
			  */
			void resize(size_t nLines)	{
				mSegments.resize(nLines);
				CRenderizableDisplayList::notifyChange();
			}
			/**
			  * Reserves an amount of lines to the set. This method should be used when some known amount of lines is going to be inserted, so that only a memory allocation is needed.
			  * \sa resize
			  */
			void reserve(size_t r)	{
				mSegments.reserve(r);
				CRenderizableDisplayList::notifyChange();
			}
			/**
			  * Inserts a line, given its bounds. Works with any pair of objects with access to x, y and z members.
			  */
			template<class T,class U> inline void appendLine(T p0,U p1)	{
				appendLine(p0.x,p0.y,p0.z,p1.x,p1.y,p1.z);
				CRenderizableDisplayList::notifyChange();
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
			inline void setLineByIndex(size_t index,double x0,double y0,double z0,double x1,double y1,double z1)	{
				setLineByIndex(index,TSegment3D(TPoint3D(x0,y0,z0),TPoint3D(x1,y1,z1)));
				CRenderizableDisplayList::notifyChange();
			}
			/**
			  * Gets a specific line in the set, given its index.
			  * \sa getLineByIndex
			  */
			inline void getLineByIndex(size_t index,double &x0,double &y0,double &z0,double &x1,double &y1,double &z1) const	{
				ASSERT_(index<mSegments.size())
				x0 = mSegments[index].point1.x;
				y0 = mSegments[index].point1.y;
				z0 = mSegments[index].point1.z;
				x1 = mSegments[index].point2.x;
				y1 = mSegments[index].point2.y;
				z1 = mSegments[index].point2.z;
			}
			/**
			  * Class factory
			  */
			inline static CSetOfLinesPtr Create(const std::vector<TSegment3D> &sgms)	{
				return CSetOfLinesPtr(new CSetOfLines(sgms));
			}
			/** Render
			  */
			void  render_dl() const;

			//Iterator management
			typedef std::vector<TSegment3D>::iterator iterator;	//!< Iterator to the set.
			typedef std::vector<TSegment3D>::reverse_iterator reverse_iterator;	//!< Iterator to the set.

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
			inline iterator begin() { CRenderizableDisplayList::notifyChange();  return mSegments.begin(); }
			/**
			  * Ending const iterator.
			  * \sa begin,rend,rbegin
			  */
			inline const_iterator end() const	{
				return mSegments.end();
			}
			inline iterator end() { CRenderizableDisplayList::notifyChange(); return mSegments.end(); }
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
