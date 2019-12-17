/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/poses/poses_frwds.h>  // All these are needed for the auxiliary methods posePDF2opengl()

namespace mrpt::opengl
{
/** A set of object, which are referenced to the coordinates framework
 *established in this object.
 *  It can be established a hierarchy of "CSetOfObjects", where the coordinates
 *framework of each
 *   one will be referenced to the parent's one.
 *	The list of child objects is accessed directly as in the class
 *"COpenGLScene"
 *  \sa opengl::COpenGLScene
 * \ingroup mrpt_opengl_grp
 */
class CSetOfObjects : public CRenderizable
{
	DEFINE_SERIALIZABLE(CSetOfObjects, mrpt::opengl)

   protected:
	/** The list of child objects.
	 *  Objects are automatically deleted when calling "clear" or in the
	 * destructor.
	 */
	CListOpenGLObjects m_objects;

   public:
	using const_iterator = CListOpenGLObjects::const_iterator;
	using iterator = CListOpenGLObjects::iterator;

	inline const_iterator begin() const { return m_objects.begin(); }
	inline const_iterator end() const { return m_objects.end(); }
	inline iterator begin() { return m_objects.begin(); }
	inline iterator end() { return m_objects.end(); }
	/** Inserts a set of objects into the list.
	 */
	template <class T>
	inline void insertCollection(const T& objs)
	{
		insert(objs.begin(), objs.end());
	}
	/** Insert a new object to the list.
	 */
	void insert(const CRenderizable::Ptr& newObject);

	/** Inserts a set of objects, bounded by iterators, into the list.
	 */
	template <class T_it>
	inline void insert(const T_it& begin, const T_it& end)
	{
		for (T_it it = begin; it != end; it++) insert(*it);
	}

	/** Render child objects.
	 */
	void render() const override;
	void renderUpdateBuffers() const override;

	/** Clear the list of objects in the scene, deleting objects' memory.
	 */
	void clear();

	/** Returns number of objects.  */
	size_t size() { return m_objects.size(); }
	/** Returns true if there are no objects.  */
	inline bool empty() const { return m_objects.empty(); }
	/** Initializes all textures in the scene (See
	 * opengl::CTexturedPlane::loadTextureInOpenGL)
	 */
	void initializeAllTextures();

	/** Returns the first object with a given name, or a nullptr pointer if not
	 * found.
	 */
	CRenderizable::Ptr getByName(const std::string& str);

	/** Returns the i'th object of a given class (or of a descendant class), or
	  nullptr (an empty smart pointer) if not found.
	  *  Example:
	  * \code
		   CSphere::Ptr obs = myscene.getByClass<CSphere>();
	  * \endcode
	  * By default (ith=0), the first observation is returned.
	  */
	template <typename T>
	typename T::Ptr getByClass(size_t ith = 0) const;

	/** Removes the given object from the scene (it also deletes the object to
	 * free its memory).
	 */
	void removeObject(const CRenderizable::Ptr& obj);

	/** Retrieves a list of all objects in text form  */
	void dumpListOfObjects(std::vector<std::string>& lst);

	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;

	CRenderizable& setColor_u8(const mrpt::img::TColor& c) override;
	CRenderizable& setColorR_u8(const uint8_t r) override;
	CRenderizable& setColorG_u8(const uint8_t g) override;
	CRenderizable& setColorB_u8(const uint8_t b) override;
	CRenderizable& setColorA_u8(const uint8_t a) override;
	bool contains(const CRenderizable::Ptr& obj) const;
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	/** @name pose_pdf -> 3d objects auxiliary templates
		@{ */
	// The reason this code is here is to exploit C++'s "T::template function()"
	// in order to
	//  define the members getAs3DObject() in several classes in mrpt-base with
	//  its argument
	//  being a class (CSetOfObjects) which is actually declared here, in
	//  mrpt-opengl.
	//  Implementations are in "pose_pdfs.cpp", not in "CSetOfObjects" (historic
	//  reasons...)

	/** Returns a representation of a the PDF - this is just an auxiliary
	 * function, it's more natural to call
	 *    mrpt::poses::CPosePDF::getAs3DObject     */
	static CSetOfObjects::Ptr posePDF2opengl(const mrpt::poses::CPosePDF& o);

	/** Returns a representation of a the PDF - this is just an auxiliary
	 * function, it's more natural to call
	 *    mrpt::poses::CPointPDF::getAs3DObject     */
	static CSetOfObjects::Ptr posePDF2opengl(const mrpt::poses::CPointPDF& o);

	/** Returns a representation of a the PDF - this is just an auxiliary
	 * function, it's more natural to call
	 *    mrpt::poses::CPose3DPDF::getAs3DObject     */
	static CSetOfObjects::Ptr posePDF2opengl(const mrpt::poses::CPose3DPDF& o);

	/** Returns a representation of a the PDF - this is just an auxiliary
	 * function, it's more natural to call
	 *    mrpt::poses::CPose3DQuatPDF::getAs3DObject     */
	static CSetOfObjects::Ptr posePDF2opengl(
		const mrpt::poses::CPose3DQuatPDF& o);

	/** @} */

	/** Default constructor
	 */
	CSetOfObjects();

	/** Private, virtual destructor: only can be deleted from smart pointers */
	~CSetOfObjects() override;
};
/** Inserts an object into the list. Allows call chaining.
 * \sa mrpt::opengl::CSetOfObjects::insert
 */
inline CSetOfObjects::Ptr& operator<<(
	CSetOfObjects::Ptr& s, const CRenderizable::Ptr& r)
{
	s->insert(r);
	return s;
}
/** Inserts a set of objects into the list. Allows call chaining.
 * \sa mrpt::opengl::CSetOfObjects::insert
 */
template <class T>
inline CSetOfObjects::Ptr& operator<<(
	CSetOfObjects::Ptr& o, const std::vector<T>& v)
{
	o->insertCollection(v);
	return o;
}

// Implementation: (here because it needs the _POST macro defining the
// Smart::Ptr)
template <typename T>
typename T::Ptr CSetOfObjects::getByClass(size_t ith) const
{
	MRPT_START
	size_t foundCount = 0;
	const auto class_ID = &T::GetRuntimeClassIdStatic();
	for (const auto& o : m_objects)
		if (o && o->GetRuntimeClass()->derivedFrom(class_ID))
			if (foundCount++ == ith) return std::dynamic_pointer_cast<T>(o);

	// If not found directly, search recursively:
	for (const auto& o : m_objects)
	{
		if (o && o->GetRuntimeClass() ==
					 CLASS_ID_NAMESPACE(CSetOfObjects, mrpt::opengl))
		{
			typename T::Ptr obj = std::dynamic_pointer_cast<CSetOfObjects>(o)
									  ->template getByClass<T>(ith);
			if (obj) return obj;
		}
	}

	return typename T::Ptr();  // Not found: return empty smart pointer
	MRPT_END
}
}  // namespace mrpt::opengl
