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
#ifndef opengl_CSetOfObjects_H
#define opengl_CSetOfObjects_H

#include <mrpt/opengl/CRenderizable.h>

// All these are needed for the auxiliary methods posePDF2opengl()
#include <mrpt/poses/CPointPDF.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DQuatPDF.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CSetOfObjects;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CSetOfObjects, CRenderizable, OPENGL_IMPEXP )

		/** A set of object, which are referenced to the coordinates framework established in this object.
		  *  It can be established a hierarchy of "CSetOfObjects", where the coordinates framework of each
		  *   one will be referenced to the parent's one.
		  *	The list of child objects is accessed directly as in the class "COpenGLScene"
		  *  \sa opengl::COpenGLScene
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CSetOfObjects : public CRenderizable
		{
			DEFINE_SERIALIZABLE( CSetOfObjects )

		protected:
			/** The list of child objects.
			  *  Objects are automatically deleted when calling "clear" or in the destructor.
			  */
			CListOpenGLObjects		m_objects;

		public:

			typedef CListOpenGLObjects::const_iterator 	const_iterator;
			typedef CListOpenGLObjects::iterator 		iterator;

			inline const_iterator begin() const { return  m_objects.begin(); }
			inline const_iterator end() const { return  m_objects.end(); }
			inline iterator begin() { return  m_objects.begin(); }
			inline iterator end() { return  m_objects.end(); }

			/** Inserts a set of objects into the list.
			  */
			template<class T> inline void insertCollection(const T &objs)	{
				insert(objs.begin(),objs.end());
			}
			/** Insert a new object to the list.
			  */
			void insert( const CRenderizablePtr &newObject );

			/** Inserts a set of objects, bounded by iterators, into the list.
			  */
			template<class T_it> inline void insert(const T_it &begin,const T_it &end)	{
				for (T_it it=begin;it!=end;it++) insert(*it);
			}

			/** Render child objects.
			  */
			void  render() const;

			/** Clear the list of objects in the scene, deleting objects' memory.
			  */
			void  clear();

			/** Returns number of objects.  */
			size_t size()  {  return m_objects.size(); }

			/** Returns true if there are no objects.  */
			inline bool empty() const { return m_objects.empty(); }

			/** Initializes all textures in the scene (See opengl::CTexturedPlane::loadTextureInOpenGL)
			  */
			void  initializeAllTextures();

			/** Returns the first object with a given name, or a NULL pointer if not found.
			  */
			CRenderizablePtr getByName( const std::string &str );

			 /** Returns the i'th object of a given class (or of a descendant class), or NULL (an empty smart pointer) if not found.
			   *  Example:
			   * \code
					CSpherePtr obs = myscene.getByClass<CSphere>();
			   * \endcode
			   * By default (ith=0), the first observation is returned.
			   */
			 template <typename T>
			 typename T::SmartPtr getByClass( const size_t &ith = 0 ) const
			 {
				MRPT_START
				size_t  foundCount = 0;
				const mrpt::utils::TRuntimeClassId*	class_ID = T::classinfo;
				for (CListOpenGLObjects::const_iterator it = m_objects.begin();it!=m_objects.end();++it)
					if (  (*it).present() && (*it)->GetRuntimeClass()->derivedFrom( class_ID ) )
						if (foundCount++ == ith)
							return typename T::SmartPtr(*it);

				// If not found directly, search recursively:
				for (CListOpenGLObjects::const_iterator it=m_objects.begin();it!=m_objects.end();++it)
				{
					if ( (*it).present() && (*it)->GetRuntimeClass() == CLASS_ID_NAMESPACE(CSetOfObjects,mrpt::opengl))
					{
						typename T::SmartPtr o = CSetOfObjectsPtr(*it)->getByClass<T>(ith);
						if (o) return o;
					}
				}

				return typename T::SmartPtr();	// Not found: return empty smart pointer
				MRPT_END
			 }


			/** Removes the given object from the scene (it also deletes the object to free its memory).
			  */
			void removeObject( const CRenderizablePtr &obj );

			/** Retrieves a list of all objects in text form.
			  */
			void dumpListOfObjects( utils::CStringList  &lst );

			/** Ray tracing
			  */
			virtual bool traceRay(const mrpt::poses::CPose3D &o,double &dist) const;

			virtual CRenderizable& setColor_u8(const mrpt::utils::TColor &c);
			virtual CRenderizable& setColorR_u8(const uint8_t r);
			virtual CRenderizable& setColorG_u8(const uint8_t g);
			virtual CRenderizable& setColorB_u8(const uint8_t b);
			virtual CRenderizable& setColorA_u8(const uint8_t a);

			bool contains(const CRenderizablePtr &obj) const;


			/** @name pose_pdf -> 3d objects auxiliary templates
			    @{ */
			// The reason this code is here is to exploit C++'s "T::template function()" in order to
			//  define the members getAs3DObject() in several classes in mrpt-base with its argument
			//  being a class (CSetOfObjects) which is actually declared here, in mrpt-opengl.
			//  Implementations are in "pose_pdfs.cpp", not in "CSetOfObjects" (historic reasons...)

			/** Returns a representation of a the PDF - this is just an auxiliary function, it's more natural to call
			  *    mrpt::poses::CPosePDF::getAs3DObject     */
			static CSetOfObjectsPtr posePDF2opengl(const mrpt::poses::CPosePDF &o);

			/** Returns a representation of a the PDF - this is just an auxiliary function, it's more natural to call
			  *    mrpt::poses::CPointPDF::getAs3DObject     */
			static CSetOfObjectsPtr posePDF2opengl(const mrpt::poses::CPointPDF &o);

			/** Returns a representation of a the PDF - this is just an auxiliary function, it's more natural to call
			  *    mrpt::poses::CPose3DPDF::getAs3DObject     */
			static CSetOfObjectsPtr posePDF2opengl(const mrpt::poses::CPose3DPDF &o);

			/** Returns a representation of a the PDF - this is just an auxiliary function, it's more natural to call
			  *    mrpt::poses::CPose3DQuatPDF::getAs3DObject     */
			static CSetOfObjectsPtr posePDF2opengl(const mrpt::poses::CPose3DQuatPDF &o);

			/** @} */

		private:
			/** Default constructor
			  */
			CSetOfObjects( );

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CSetOfObjects();
		};
		/** Inserts an object into the list. Allows call chaining.
		  * \sa mrpt::opengl::CSetOfObjects::insert
		  */
		inline CSetOfObjectsPtr &operator<<(CSetOfObjectsPtr &s,const CRenderizablePtr &r)	{
			s->insert(r);
			return s;
		}
		/** Inserts a set of objects into the list. Allows call chaining.
		  * \sa mrpt::opengl::CSetOfObjects::insert
		  */
		template<class T> inline CSetOfObjectsPtr &operator<<(CSetOfObjectsPtr &o,const std::vector<T> &v)	{
			o->insertCollection(v);
			return o;
		}


	} // end namespace

} // End of namespace


#endif
