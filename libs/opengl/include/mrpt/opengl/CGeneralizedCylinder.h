/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/geometry.h>
#include <mrpt/opengl/CPolyhedron.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfTriangles.h>
#include <vector>

namespace mrpt::opengl
{
class CGeneralizedCylinder;
/**
 * This object represents any figure obtained by extruding any profile along a
 * given axis. The profile should lie over a x=0 plane, and the axis must be
 * roughly perpendicular to this plane. In particular, it should be almost
 * perpendicular to the Z axis.
 * \ingroup mrpt_opengl_grp
 */
class CGeneralizedCylinder : public CRenderizable
{
	DEFINE_SERIALIZABLE(CGeneralizedCylinder, mrpt::opengl)
   public:
	/**
	 * Auxiliary struct holding any quadrilateral, represented by foour points.
	 */
	struct TQuadrilateral
	{
	   private:
		/**
		 * Automatically compute a vector normal to this quadrilateral.
		 */
		void calculateNormal();

	   public:
		/**
		 * Quadrilateral`'s points.
		 */
		mrpt::math::TPoint3D points[4];
		/**
		 * Normal vector.
		 */
		double normal[3];
		/**
		 * Given a polygon with 4 already positions allocated, this method
		 * fills it with the quadrilateral points.
		 * \sa mrpt::math::TPolygon3D
		 */
		inline void getAsPolygonUnsafe(mrpt::math::TPolygon3D& vec) const
		{
			vec[0] = points[0];
			vec[1] = points[1];
			vec[2] = points[2];
			vec[3] = points[3];
		}
		/**
		 * Constructor from 4 points.
		 */
		TQuadrilateral(
			const mrpt::math::TPoint3D& p1, const mrpt::math::TPoint3D& p2,
			const mrpt::math::TPoint3D& p3, const mrpt::math::TPoint3D& p4)
		{
			points[0] = p1;
			points[1] = p2;
			points[2] = p3;
			points[3] = p4;
			calculateNormal();
		}
		/**
		 * Construction from any array of four compatible objects.
		 */
		template <class T>
		TQuadrilateral(const T (&p)[4])
		{
			for (int i = 0; i < 4; i++) points[i] = p[i];
			calculateNormal();
		}
		/**
		 * Empty constructor. Initializes to garbage.
		 */
		TQuadrilateral() = default;
		/**
		 * Destructor.
		 */
		~TQuadrilateral() = default;
	};

   protected:
	/** Cylinder's axis. It's represented as a pose because it holds the angle
	 * to get to the next pose. */
	std::vector<mrpt::poses::CPose3D> axis;
	/**  Object's generatrix, that is, profile which will be extruded. */
	std::vector<mrpt::math::TPoint3D> generatrix;
	/** Mutable object with mesh information, used to avoid repeated
	 * computations.  */
	mutable std::vector<TQuadrilateral> mesh;
	/** Mutable object with the cylinder's points, used to avoid repeated
	 * computations. */
	mutable mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_data<double>>
		pointsMesh;
	/**  Mutable flag which tells if recalculations are needed. */
	mutable bool meshUpToDate{false};
	/**
	 * Mutable set of data used in ray tracing.
	 * \sa mrpt::math::TPolygonWithPlane
	 */
	mutable std::vector<mrpt::math::TPolygonWithPlane> polys;
	/** Mutable flag telling whether ray tracing temporary data must be
	 * recalculated or not. */
	mutable bool polysUpToDate{false};
	/** Boolean variable which determines if the profile is closed at each
	 * section. */
	bool closed{false};
	/** Flag to determine whether the object is fully visible or only some
	 * sections are. */
	bool fullyVisible{true};
	/**
	 * First visible section, if fullyVisible is set to false.
	 * \sa fullyVisible,lastSection
	 */
	size_t firstSection;
	/**
	 * Last visible section, if fullyVisible is set to false.
	 * \sa fullyVisible,firstSection
	 */
	size_t lastSection;

   public:
	void render(const mrpt::opengl::TRenderMatrices& state, mrpt::opengl::Program& shaders) const override;
	void renderUpdateBuffers() const override;
	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;

	/**
	 * Get axis's spatial coordinates.
	 */
	inline void getAxis(std::vector<mrpt::math::TPoint3D>& a) const
	{
		// a=axis;
		size_t N = axis.size();
		a.resize(N);
		for (size_t i = 0; i < N; i++)
		{
			a[i].x = axis[i].x();
			a[i].y = axis[i].y();
			a[i].z = axis[i].z();
		}
	}
	/**
	 * Get axis, including angular coordinates.
	 */
	inline void getAxis(std::vector<mrpt::poses::CPose3D>& a) const
	{
		a = axis;
	}
	/**
	 * Set the axis points.
	 */
	inline void setAxis(const std::vector<mrpt::math::TPoint3D>& a)
	{
		generatePoses(a, axis);
		meshUpToDate = false;
		fullyVisible = true;
		CRenderizable::notifyChange();
	}
	/**
	 * Get cylinder's profile.
	 */
	inline void getGeneratrix(std::vector<mrpt::math::TPoint3D>& g) const
	{
		g = generatrix;
	}
	/**
	 * Set cylinder's profile.
	 */
	inline void setGeneratrix(const std::vector<mrpt::math::TPoint3D>& g)
	{
		generatrix = g;
		meshUpToDate = false;
		CRenderizable::notifyChange();
	}
	/**
	 * Returns true if each section is a closed polygon.
	 */
	inline bool isClosed() const { return closed; }
	/**
	 * Set whether each section is a closed polygon or not.
	 */
	inline void setClosed(bool c = true)
	{
		closed = c;
		meshUpToDate = false;
		CRenderizable::notifyChange();
	}
	/**
	 * Get a polyhedron containing the starting point of the cylinder (its
	 * "base").
	 * \sa getEnd,mrpt::opengl::CPolyhedron
	 */
	void getOrigin(CPolyhedron::Ptr& poly) const;
	/**
	 * Get a polyhedron containing the ending point of the cylinder (its
	 * "base").
	 * \sa getOrigin,mrpt::opengl::CPolyhedron
	 */
	void getEnd(CPolyhedron::Ptr& poly) const;
	/**
	 * Get the cylinder as a set of polygons in 3D.
	 * \sa mrpt::math::TPolygon3D
	 */
	void generateSetOfPolygons(std::vector<mrpt::math::TPolygon3D>& res) const;
	/**
	 * Get a polyhedron consisting of a set of closed sections of the cylinder.
	 * \sa mrpt::opengl::CPolyhedron
	 */
	void getClosedSection(
		size_t index1, size_t index2, CPolyhedron::Ptr& poly) const;
	/**
	 * Get a polyhedron consisting of a single section of the cylinder.
	 * \sa mrpt::opengl::CPolyhedron
	 */
	inline void getClosedSection(size_t index, CPolyhedron::Ptr& poly) const
	{
		getClosedSection(index, index, poly);
	}
	/**
	 * Get the number of sections in this cylinder.
	 */
	inline size_t getNumberOfSections() const
	{
		return axis.size() ? (axis.size() - 1) : 0;
	}
	/**
	 * Get how many visible sections are in the cylinder.
	 */
	inline size_t getVisibleSections() const
	{
		return fullyVisible ? getNumberOfSections()
							: (lastSection - firstSection);
	}
	/**
	 * Gets the cylinder's visible sections.
	 */
	void getVisibleSections(size_t& first, size_t& last) const
	{
		if (fullyVisible)
		{
			first = 0;
			last = getNumberOfSections();
		}
		else
		{
			first = firstSection;
			last = lastSection;
		}
	}
	/**
	 * Sets all sections visible.
	 */
	inline void setAllSectionsVisible()
	{
		fullyVisible = true;
		CRenderizable::notifyChange();
	}
	/**
	 * Hides all sections.
	 */
	inline void setAllSectionsInvisible(size_t pointer = 0)
	{
		fullyVisible = false;
		firstSection = pointer;
		lastSection = pointer;
		CRenderizable::notifyChange();
	}
	/**
	 * Sets which sections are visible.
	 * \throw std::logic_error on wrongly defined bounds.
	 */
	inline void setVisibleSections(size_t first, size_t last)
	{
		fullyVisible = false;
		if (first > last || last > getNumberOfSections())
			throw std::logic_error("Wrong bound definition");
		firstSection = first;
		lastSection = last;
		CRenderizable::notifyChange();
	}
	/**
	 * Adds another visible section at the start of the cylinder. The cylinder
	 * must have an invisble section to display.
	 * \throw std::logic_error if there is no section to add to the displaying
	 * set.
	 * \sa
	 * addVisibleSectionAtEnd,removeVisibleSectionAtStart,removeVisibleSectionAtEnd
	 */
	inline void addVisibleSectionAtStart()
	{
		if (fullyVisible || firstSection == 0)
			throw std::logic_error("No more sections");
		firstSection--;
		CRenderizable::notifyChange();
	}
	/**
	 * Adds another visible section at the end of the cylinder. The cylinder
	 * must have an invisible section to display.
	 * \throw std::logic_error if there is no section to add to the displaying
	 * set.
	 * \sa
	 * addVisibleSectionAtStart,removeVisibleSectionAtStart,removeVisibleSectionAtEnd
	 */
	inline void addVisibleSectionAtEnd()
	{
		if (fullyVisible || lastSection == getNumberOfSections())
			throw std::logic_error("No more sections");
		lastSection++;
		CRenderizable::notifyChange();
	}
	/**
	 * Removes a visible section from the start of the currently visible set.
	 * \throw std::logic_error if there are no visible sections.
	 * \sa
	 * addVisibleSectionAtStart,addVisibleSectionAtEnd,removeVisibleSectionAtEnd
	 */
	void removeVisibleSectionAtStart();
	/**
	 * Removes a visible section from the ending of the currently visible set.
	 * \throw std::logic_error when there is no such section.
	 * \sa
	 * addVisibleSectionAtStart,addVisibleSectionAtEnd,removeVisibleSectionAtStart
	 */
	void removeVisibleSectionAtEnd();
	/**
	 * Gets the axis pose of the first section, returning false if there is no
	 * such pose.
	 */
	bool getFirstSectionPose(mrpt::poses::CPose3D& p);
	/**
	 * Gets the axis pose of the last section, returning false if there is no
	 * such pose.
	 */
	bool getLastSectionPose(mrpt::poses::CPose3D& p);
	/**
	 * Gets the axis pose of the first visible section, returning false if
	 * there is no such pose.
	 */
	bool getFirstVisibleSectionPose(mrpt::poses::CPose3D& p);
	/**
	 * Gets the axis pose of the last section, returning false if there is no
	 * such pose.
	 */
	bool getLastVisibleSectionPose(mrpt::poses::CPose3D& p);
	/**
	 * Updates the mutable set of polygons used in ray tracing.
	 */
	void updatePolys() const;

	/** Evaluates the bounding box of this object (including possible children)
	 * in the coordinate frame of the object parent. */
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

   private:
	/**
	 * Updates the axis, transforming each point into a pose pointing to the
	 * next section.
	 */
	void generatePoses(
		const std::vector<mrpt::math::TPoint3D>& pIn,
		std::vector<mrpt::poses::CPose3D>& pOut);
	/**
	 * Updates the mutable mesh.
	 */
	void updateMesh() const;
	/**
	 * Given a vector of polyhedrons, gets the starting and ending iterators to
	 * the section to be actually rendered.
	 */
	void getMeshIterators(
		const std::vector<TQuadrilateral>& m,
		std::vector<TQuadrilateral>::const_iterator& begin,
		std::vector<TQuadrilateral>::const_iterator& end) const;

   public:
	/**
	 * Basic constructor with default initialization.
	 */
	CGeneralizedCylinder() : axis(), generatrix(), mesh() {}
	/**
	 * Constructor with axis and generatrix.
	 */
	CGeneralizedCylinder(
		const std::vector<mrpt::math::TPoint3D>& a,
		const std::vector<mrpt::math::TPoint3D>& g)
		: generatrix(g),
		  mesh(),
		  meshUpToDate(false),
		  polysUpToDate(false),
		  closed(false),
		  fullyVisible(true)
	{
		generatePoses(a, axis);
	}
	/**
	 * Destructor.
	 */
	~CGeneralizedCylinder() override = default;
};
}  // namespace mrpt::opengl
