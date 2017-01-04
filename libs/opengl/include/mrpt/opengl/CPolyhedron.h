/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_CPolyhedron_H
#define opengl_CPolyhedron_H

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/math/geometry.h>

namespace mrpt	{
namespace opengl	{
	class OPENGL_IMPEXP CPolyhedron;

	// This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CPolyhedron,CRenderizableDisplayList, OPENGL_IMPEXP)
	/**
	  * This class represents arbitrary polyhedra. The class includes a set of static methods to create common polyhedrons. The class includes many methods to create standard polyhedra, not intended to be fast but to be simple. For example, the dodecahedron is not created efficiently: first, an icosahedron is created, and a duality operator is applied to it, which yields the dodecahedron. This way, code is much smaller, although much slower. This is not a big problem, since polyhedron creation does not usually take a significant amount of time (they are created once and rendered many times).
	  * Polyhedra information and models have been gotten from the Wikipedia, http://wikipedia.org
	  * \sa opengl::COpenGLScene
	  *
	  *  <div align="center">
	  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
	  *   <tr> <td> mrpt::opengl::CPolyhedron </td> <td> \image html preview_CPolyhedron.png </td> </tr>
	  *  </table>
	  *  </div>
	  *
	  * \ingroup mrpt_opengl_grp
	  */
	class OPENGL_IMPEXP CPolyhedron:public CRenderizableDisplayList	{
		DEFINE_SERIALIZABLE(CPolyhedron)
	public:
		/**
		  * Struct used to store a polyhedron edge. The struct consists only of two vertex indices, used to access the polyhedron vertex list.
		  */
		struct OPENGL_IMPEXP TPolyhedronEdge	{
			/**
			  * First vertex.
			  */
			uint32_t v1;
			/**
			  * Second vertex.
			  */
			uint32_t v2;
			/**
			  * Default constructor. Initializes to garbage.
			  */
			TPolyhedronEdge():v1(),v2()	{}
			/**
			  * Comparison agains another edge. Simmetry is taken into account.
			  */
			bool operator==(const TPolyhedronEdge &e) const	{
				if (e.v1==v1&&e.v2==v2) return true;
				else return e.v1==v2&&e.v2==v1;
			}
			/**
			  * Given a set of vertices, computes the length of the vertex.
			  */
			double length(const std::vector<mrpt::math::TPoint3D> &vs) const;
			/**
			  * Destructor.
			  */
			~TPolyhedronEdge()	{}
		};
		/**
		  * Struct used to store a polyhedron face. Consists on a set of vertex indices and a normal vector.
		  */
		struct OPENGL_IMPEXP TPolyhedronFace	{
			std::vector<uint32_t> vertices; //!< Vector of indices to the vertex list.
			double normal[3]; //!< Normal vector.
			/** Fast default constructor. Initializes to garbage. */
			TPolyhedronFace():vertices()	{}
			/** Destructor.  */
			~TPolyhedronFace()	{}
			double area(const std::vector<mrpt::math::TPoint3D> &vertices) const; //!< Given a set of vertices, computes the area of this face.
			void getCenter(const std::vector<mrpt::math::TPoint3D> &vertices, mrpt::math::TPoint3D &p) const; //!< Given a set of vertices, get this face's center.
		};
	protected:
		/**
		  * List of vertices presents in the polyhedron.
		  */
		std::vector<mrpt::math::TPoint3D> mVertices;
		/**
		  * List of polyhedron's edges.
		  */
		std::vector<TPolyhedronEdge> mEdges;
		/**
		  * List of polyhedron's faces.
		  */
		std::vector<TPolyhedronFace> mFaces;
		/**
		  * This flag determines whether the polyhedron will be displayed as a solid object or as a set of edges.
		  */
		bool mWireframe;
		/**
		  * When displaying as wireframe object, this variable stores the width of the edges.
		  */
		double mLineWidth;
		/**
		  * Mutable list of actual polygons, maintained for speed.
		  */
		mutable std::vector<mrpt::math::TPolygonWithPlane> tempPolygons;
		/**
		  * Whether the set of actual polygons is up to date or not.
		  */
		mutable bool polygonsUpToDate;
	public:
		/**
		  * Creation of a polyhedron from its vertices and faces.
		  * \throw logic_error if the polyhedron definition has flaws (bad vertex indices, etc.).
		  */
		static CPolyhedronPtr Create(const std::vector<mrpt::math::TPoint3D> &vertices,const std::vector<std::vector<uint32_t> > &faces);
		/**
		  * Creation of a polyhedron from its vertices and faces.
		  * \throw logic_error if the polyhedron definition has flaws (bad vertex indices, etc.).
		  */
		static CPolyhedronPtr Create(const std::vector<mrpt::math::TPoint3D> &vertices,const std::vector<TPolyhedronFace> &faces);
		/**
		  * Creation from a set of polygons.
		  * \sa mrpt::math::TPolygon3D
		  */
		static CPolyhedronPtr Create(const std::vector<mrpt::math::TPolygon3D> &polys);

		/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
		void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const MRPT_OVERRIDE;

		//Static methods to create frequent polyhedra. More bizarre polyhedra are intended to be added in a near future.

		/** @name Platonic solids.
			@{
		 */
		/**
		  * Creates a regular tetrahedron (see http://en.wikipedia.org/wiki/Tetrahedron). The tetrahedron is created as a triangular pyramid whose edges and vertices are transitive.
		  * The tetrahedron is the dual to itself.
		  <p align="center"><img src="Tetrahedron.gif"></p>
		  * \sa CreatePyramid,CreateJohnsonSolidWithConstantBase,CreateTruncatedTetrahedron
		  */
		static CPolyhedronPtr CreateTetrahedron(double radius);
		/**
		  * Creates a regular cube, also called hexahedron (see http://en.wikipedia.org/wiki/Hexahedron). The hexahedron is created as a cubic prism which transitive edges. Another ways to create it include:
		  <ul><li>Dual to an octahedron.</li><li>Parallelepiped with three orthogonal, equally-lengthed vectors.</li><li>Triangular trapezohedron with proper height.</li></ul>
		  <p align="center"><img src="Hexahedron.gif"></p>
		  * \sa CreateOctahedron,getDual,CreateParallelepiped,CreateTrapezohedron,CreateTruncatedHexahedron,CreateTruncatedOctahedron,CreateCuboctahedron,CreateRhombicuboctahedron
		  */
		static CPolyhedronPtr CreateHexahedron(double radius);
		/**
		  * Creates a regular octahedron (see http://en.wikipedia.org/wiki/Octahedron). The octahedron is created as a square bipyramid whit transitive edges and vertices. Another ways to create an octahedron are:
		  <ul><li>Dual to an hexahedron</li><li>Triangular antiprism with transitive vertices.</li><li>Conveniently truncated tetrahedron.</li></ul>
		  <p align="center"><img src="Octahedron.gif"></p>
		  * \sa CreateHexahedron,getDual,CreateArchimedeanAntiprism,CreateTetrahedron,truncate,CreateTruncatedOctahedron,CreateTruncatedHexahedron,CreateCuboctahedron,CreateRhombicuboctahedron
		  */
		static CPolyhedronPtr CreateOctahedron(double radius);
		/**
		  * Creates a regular dodecahedron (see http://en.wikipedia.org/wiki/Dodecahedron). The dodecahedron is created as the dual to an icosahedron.
		  <p align="center"><img src="Dodecahedron.gif"></p>
		  * \sa CreateIcosahedron,getDual,CreateTruncatedDodecahedron,CreateTruncatedIcosahedron,CreateIcosidodecahedron,CreateRhombicosidodecahedron
		  */
		static CPolyhedronPtr CreateDodecahedron(double radius);
		/**
		  * Creates a regular icosahedron (see http://en.wikipedia.org/wiki/Icosahedron). The icosahedron is created as a gyroelongated pentagonal bipyramid with transitive edges, and it's the dual to a dodecahedron.
		  <p align="center"><img src="Icosahedron.gif"></p>
		  * \sa CreateJohnsonSolidWithConstantBase,CreateDodecahedron,getDual,CreateTruncatedIcosahedron,CreateTruncatedDodecahedron,CreateIcosidodecahedron,CreateRhombicosidodecahedron
		  */
		static CPolyhedronPtr CreateIcosahedron(double radius);
		/** @}
		  */

		/** @name Archimedean solids.
			@{
		 */
		/**
		  * Creates a truncated tetrahedron, consisting of four triangular faces and for hexagonal ones (see http://en.wikipedia.org/wiki/Truncated_tetrahedron). Its dual is the triakis tetrahedron.
		  <p align="center"><img src="Truncatedtetrahedron.gif"></p>
		  * \sa CreateTetrahedron,CreateTriakisTetrahedron
		  */
		static CPolyhedronPtr CreateTruncatedTetrahedron(double radius);
		/**
		  * Creates a cuboctahedron, consisting of six square faces and eight triangular ones (see http://en.wikipedia.org/wiki/Cuboctahedron). There are several ways to create a cuboctahedron:
		  <ul><li>Hexahedron truncated to a certain extent.</li><li>Octahedron truncated to a certain extent.</li><li>Cantellated tetrahedron</li><li>Dual to a rhombic dodecahedron.</li></ul>
		  <p align="center"><img src="Cuboctahedron.gif"></p>
		  * \sa CreateHexahedron,CreateOctahedron,truncate,CreateTetrahedron,cantellate,CreateRhombicuboctahedron,CreateRhombicDodecahedron,
		  */
		static CPolyhedronPtr CreateCuboctahedron(double radius);
		/**
		  * Creates a truncated hexahedron, with six octogonal faces and eight triangular ones (see http://en.wikipedia.org/wiki/Truncated_hexahedron). The truncated octahedron is dual to the triakis octahedron.
		  <p align="center"><img src="Truncatedhexahedron.gif"></p>
		  * \sa CreateHexahedron,CreateTriakisOctahedron
		  */
		static CPolyhedronPtr CreateTruncatedHexahedron(double radius);
		/**
		  * Creates a truncated octahedron, with eight hexagons and eight squares (see http://en.wikipedia.org/wiki/Truncated_octahedron). It's the dual to the tetrakis hexahedron.
		  <p align="center"><img src="Truncatedoctahedron.gif"></p>
		  * \sa CreateOctahedron,CreateTetrakisHexahedron
		  */
		static CPolyhedronPtr CreateTruncatedOctahedron(double radius);
		/**
		  * Creates a rhombicuboctahedron, with 18 squares and 8 triangles (see http://en.wikipedia.org/wiki/Rhombicuboctahedron), calculated as an elongated square bicupola. It can also be calculated as a cantellated hexahedron or octahedron, and its dual is the deltoidal icositetrahedron.
		  * If the second argument is set to false, the lower cupola is rotated, so that the objet created is an elongated square gyrobicupola (see http://en.wikipedia.org/wiki/Elongated_square_gyrobicupola). This is not an archimedean solid, but a Johnson one, since it hasn't got vertex transitivity.
		  <p align="center"><img src="Rhombicuboctahedron.gif"></p>
		  * \sa CreateJohnsonSolidWithConstantBase,CreateHexahedron,CreateOctahedron,cantellate,CreateCuboctahedron,CreateDeltoidalIcositetrahedron
		  */
		static CPolyhedronPtr CreateRhombicuboctahedron(double radius,bool type=true);
		/**
		  * Creates an icosidodecahedron, with 12 pentagons and 20 triangles (see http://en.wikipedia.org/wiki/Icosidodecahedron). Certain truncations of either a dodecahedron or an icosahedron yield an icosidodecahedron.
		  * The dual of the icosidodecahedron is the rhombic triacontahedron.
		  * If the second argument is set to false, the lower rotunda is rotated. In this case, the object created is a pentagonal orthobirotunda (see http://en.wikipedia.org/wiki/Pentagonal_orthobirotunda). This object presents symmetry against the XY plane and is not vertex transitive, so it's a Johnson's solid.
		  <p align="center"><img src="Icosidodecahedron.gif"></p>
		  * \sa CreateDodecahedron,CreateIcosahedron,truncate,CreateRhombicosidodecahedron,CreateRhombicTriacontahedron
		  */
		static CPolyhedronPtr CreateIcosidodecahedron(double radius,bool type=true);
		/**
		  * Creates a truncated dodecahedron, consisting of 12 dodecagons and 20 triangles (see http://en.wikipedia.org/wiki/Truncated_dodecahedron). The truncated dodecahedron is the dual to the triakis icosahedron.
		  <p align="center"><img src="Truncateddodecahedron.gif"></p>
		  * \sa CreateDodecahedron,CreateTriakisIcosahedron
		  */
		static CPolyhedronPtr CreateTruncatedDodecahedron(double radius);
		/**
		  * Creates a truncated icosahedron, consisting of 20 hexagons and 12 pentagons. This object resembles a typical soccer ball (see http://en.wikipedia.org/wiki/Truncated_icosahedron). The pentakis dodecahedron is the dual to the truncated icosahedron.
		  <p align="center"><img src="Truncatedicosahedron.gif"></p>
		  * \sa CreateIcosahedron,CreatePentakisDodecahedron
		  */
		static CPolyhedronPtr CreateTruncatedIcosahedron(double radius);
		/**
		  * Creates a rhombicosidodecahedron, consisting of 30 squares, 12 pentagons and 20 triangles (see http://en.wikipedia.org/wiki/Rhombicosidodecahedron). This object can be obtained as the cantellation of either a dodecahedron or an icosahedron. The dual of the rhombicosidodecahedron is the deltoidal hexecontahedron.
		  <p align="center"><img src="Rhombicosidodecahedron.gif"></p>
		  * \sa CreateDodecahedron,CreateIcosahedron,CreateIcosidodecahedron,CreateDeltoidalHexecontahedron
		  */
		static CPolyhedronPtr CreateRhombicosidodecahedron(double radius);
		/** @}
		 */

		/** @name Other Johnson solids.
			@{
		 */
		/**
		  * Creates a pentagonal rotunda (half an icosidodecahedron), consisting of six pentagons, ten triangles and a decagon (see http://en.wikipedia.org/wiki/Pentagonal_rotunda).
		  * \sa CreateIcosidodecahedron,CreateJohnsonSolidWithConstantBase
		  */
		static CPolyhedronPtr CreatePentagonalRotunda(double radius);
		/** @}
		 */

		/** @name Catalan solids.
			@{
		*/
		/**
		  * Creates a triakis tetrahedron, dual to the truncated tetrahedron. This body consists of 12 isosceles triangles (see http://en.wikipedia.org/wiki/Triakis_tetrahedron).
		  <p align="center"><img src="Triakistetrahedron.gif"></p>
		  * \sa CreateTruncatedTetrahedron
		  */
		static CPolyhedronPtr CreateTriakisTetrahedron(double radius);

		/**
		  * Creates a rhombic dodecahedron, dual to the cuboctahedron. This body consists of 12 rhombi (see http://en.wikipedia.org/wiki/Rhombic_dodecahedron).
		  <p align="center"><img src="Rhombicdodecahedron.gif"></p>
		  * \sa CreateCuboctahedron
		  */
		static CPolyhedronPtr CreateRhombicDodecahedron(double radius);

		/**
		  * Creates a triakis octahedron, dual to the truncated hexahedron. This body consists of 24 isosceles triangles (see http://en.wikipedia.org/wiki/Triakis_octahedron).
		  <p align="center"><img src="Triakisoctahedron.gif"></p>
		  * \sa CreateTruncatedHexahedron
		  */
		static CPolyhedronPtr CreateTriakisOctahedron(double radius);

		/**
		  * Creates a tetrakis hexahedron, dual to the truncated octahedron. This body consists of 24 isosceles triangles (see http://en.wikipedia.org/wiki/Tetrakis_hexahedron).
		  <p align="center"><img src="Tetrakishexahedron.gif"></p>
		  * \sa CreateTruncatedOctahedron
		  */
		static CPolyhedronPtr CreateTetrakisHexahedron(double radius);

		/**
		  * Creates a deltoidal icositetrahedron, dual to the rhombicuboctahedron. This body consists of 24 kites (see http://en.wikipedia.org/wiki/Deltoidal_icositetrahedron).
		  <p align="center"><img src="Deltoidalicositetrahedron.gif"></p>
		  * \sa CreateRhombicuboctahedron
		  */
		static CPolyhedronPtr CreateDeltoidalIcositetrahedron(double radius);

		/**
		  * Creates a rhombic triacontahedron, dual to the icosidodecahedron. This body consists of 30 rhombi (see http://en.wikipedia.org/wiki/Rhombic_triacontahedron).
		  <p align="center"><img src="Rhombictriacontahedron.gif"></p>
		  * \sa CreateIcosidodecahedron
		  */
		static CPolyhedronPtr CreateRhombicTriacontahedron(double radius);

		/**
		  * Creates a triakis icosahedron, dual to the truncated dodecahedron. This body consists of 60 isosceles triangles http://en.wikipedia.org/wiki/Triakis_icosahedron).
		  <p align="center"><img src="Triakisicosahedron.gif"></p>
		  * \sa CreateTruncatedDodecahedron
		  */
		static CPolyhedronPtr CreateTriakisIcosahedron(double radius);

		/**
		  * Creates a pentakis dodecahedron, dual to the truncated icosahedron. This body consists of 60 isosceles triangles (see http://en.wikipedia.org/wiki/Pentakis_dodecahedron).
		  <p align="center"><img src="Pentakisdodecahedron.gif"></p>
		  * \sa CreateTruncatedIcosahedron
		  */
		static CPolyhedronPtr CreatePentakisDodecahedron(double radius);

		/**
		  * Creates a deltoidal hexecontahedron, dual to the rhombicosidodecahedron. This body consists of 60 kites (see http://en.wikipedia.org/wiki/Deltoidal_hexecontahedron).
		  <p align="center"><img src="Deltoidalhexecontahedron.gif"></p>
		  * \sa CreateRhombicosidodecahedron
		  */
		static CPolyhedronPtr CreateDeltoidalHexecontahedron(double radius);
		/** @}
		 */

		/** @name Customizable polyhedra
			@{
		 */
		/**
		  * Creates a cubic prism, given the coordinates of two opposite vertices. Each edge will be parallel to one of the coordinate axes, although the orientation may change by assigning a pose to the object.
		  * \sa CreateCubicPrism(const mrpt::math::TPoint3D &,const mrpt::math::TPoint3D &),CreateParallelepiped,CreateCustomPrism,CreateRegularPrism,CreateArchimedeanRegularPrism
		  */
		static CPolyhedronPtr CreateCubicPrism(double x1,double x2,double y1,double y2,double z1,double z2);
		/**
		  * Creates a cubic prism, given two opposite vertices.
		  * \sa CreateCubicPrism(double,double,double,double,double,double),CreateParallelepiped,CreateCustomPrism,CreateRegularPrism,CreateArchimedeanRegularPrism
		  */
		static CPolyhedronPtr CreateCubicPrism(const mrpt::math::TPoint3D &p1,const mrpt::math::TPoint3D &p2);
		/**
		  * Creates a custom pyramid, using a set of 2D vertices which will lie on the XY plane.
		  * \sa CreateDoublePyramid,CreateFrustum,CreateBifrustum,CreateRegularPyramid
		  */
		static CPolyhedronPtr CreatePyramid(const std::vector<mrpt::math::TPoint2D> &baseVertices,double height);
		/**
		  * Creates a double pyramid, using a set of 2D vertices which will lie on the XY plane. The second height is used with the downwards pointing pyramid, so that it will effectively point downwards if it's positive.
		  * \sa CreatePyramid,CreateBifrustum,CreateRegularDoublePyramid
		  */
		static CPolyhedronPtr CreateDoublePyramid(const std::vector<mrpt::math::TPoint2D> &baseVertices,double height1,double height2);
		/**
		  * Creates a truncated pyramid, using a set of vertices which will lie on the XY plane.
		  * Do not try to use with a ratio equal to zero; use CreatePyramid instead. When using a ratio of 1, it will create a Prism.
		  * \sa CreatePyramid,CreateBifrustum
		  */
		static CPolyhedronPtr CreateTruncatedPyramid(const std::vector<mrpt::math::TPoint2D> &baseVertices,double height,double ratio);
		/**
		  * This is a synonym for CreateTruncatedPyramid.
		  * \sa CreateTruncatedPyramid
		  */
		static CPolyhedronPtr CreateFrustum(const std::vector<mrpt::math::TPoint2D> &baseVertices,double height,double ratio);
		/**
		  * Creates a custom prism with vertical edges, given any base which will lie on the XY plane.
		  * \sa CreateCubicPrism,CreateCustomAntiprism,CreateRegularPrism,CreateArchimedeanRegularPrism
		  */
		static CPolyhedronPtr CreateCustomPrism(const std::vector<mrpt::math::TPoint2D> &baseVertices,double height);
		/**
		  * Creates a custom antiprism, using two custom bases. For better results, the top base should be slightly rotated with respect to the bottom one.
		  * \sa CreateCustomPrism,CreateRegularAntiprism,CreateArchimedeanRegularAntiprism
		  */
		static CPolyhedronPtr CreateCustomAntiprism(const std::vector<mrpt::math::TPoint2D> &bottomBase,const std::vector<mrpt::math::TPoint2D> &topBase,double height);
		/**
		  * Creates a parallelepiped, given a base point and three vectors represented as points.
		  * \sa CreateCubicPrism
		  */
		static CPolyhedronPtr CreateParallelepiped(const mrpt::math::TPoint3D &base,const mrpt::math::TPoint3D &v1,const mrpt::math::TPoint3D &v2,const mrpt::math::TPoint3D &v3);
		/**
		  * Creates a bifrustum, or double truncated pyramid, given a base which will lie on the XY plane.
		  * \sa CreateFrustum,CreateDoublePyramid
		  */
		static CPolyhedronPtr CreateBifrustum(const std::vector<mrpt::math::TPoint2D> &baseVertices,double height1,double ratio1,double height2,double ratio2);
		/**
		  * Creates a trapezohedron, consisting of 2*N kites, where N is the number of edges in the base. The base radius controls the polyhedron height, whilst the distance between bases affects the height.
		  * When the number of edges equals 3, the polyhedron is actually a parallelepiped, and it can even be a cube.
		  */
		static CPolyhedronPtr CreateTrapezohedron(uint32_t numBaseEdges,double baseRadius,double basesDistance);
		/**
		  * Creates an antiprism whose base is a regular polygon. The upper base is rotated \f$\frac\pi N\f$ with respect to the lower one, where N is the number of vertices in the base, and thus the lateral triangles are isosceles.
		  * \sa CreateCustomAntiprism,CreateArchimedeanRegularAntiprism
		  */
		static CPolyhedronPtr CreateRegularAntiprism(uint32_t numBaseEdges,double baseRadius,double height);
		/**
		  * Creates a regular prism whose base is a regular polygon and whose edges are either parallel or perpendicular to the XY plane.
		  * \sa CreateCubicPrism,CreateCustomPrism,CreateArchimedeanRegularAntiprism
		  */
		static CPolyhedronPtr CreateRegularPrism(uint32_t numBaseEdges,double baseRadius,double height);
		/**
		  * Creates a regular pyramid whose base is a regular polygon.
		  * \sa CreatePyramid
		  */
		static CPolyhedronPtr CreateRegularPyramid(uint32_t numBaseEdges,double baseRadius,double height);
		/**
		  * Creates a regular double pyramid whose base is a regular polygon.
		  * \sa CreateDoublePyramid
		  */
		static CPolyhedronPtr CreateRegularDoublePyramid(uint32_t numBaseEdges,double baseRadius,double height1,double height2);
		/**
		  * Creates a regular prism whose lateral area is comprised of squares, and so each face of its is a regular polygon. Due to vertex transitivity, the resulting object is always archimedean.
		  * \sa CreateRegularPrism,CreateCustomPrism
		  */
		static CPolyhedronPtr CreateArchimedeanRegularPrism(uint32_t numBaseEdges,double baseRadius);
		/**
		  * Creates a regular antiprism whose lateral polygons are equilateral triangles, and so each face of its is a regular polygon. Due to vertex transitivity, the resulting object is always archimedean.
		  * \sa CreateRegularAntiprism,CreateCustomAntiprism
		  */
		static CPolyhedronPtr CreateArchimedeanRegularAntiprism(uint32_t numBaseEdges,double baseRadius);
		/**
		  * Creates a regular truncated pyramid whose base is a regular polygon.
		  * \sa CreateTruncatedPyramid
		  */
		static CPolyhedronPtr CreateRegularTruncatedPyramid(uint32_t numBaseEdges,double baseRadius,double height,double ratio);
		/**
		  * This is a synonym for CreateRegularTruncatedPyramid.
		  * \sa CreateRegularTruncatedPyramid
		  */
		static CPolyhedronPtr CreateRegularFrustum(uint32_t numBaseEdges,double baseRadius,double height,double ratio);
		/**
		  * Creates a bifrustum (double truncated pyramid) whose base is a regular polygon lying in the XY plane.
		  * \sa CreateBifrustum
		  */
		static CPolyhedronPtr CreateRegularBifrustum(uint32_t numBaseEdges,double baseRadius,double height1,double ratio1,double height2,double ratio2);
		/**
		  * Creates a cupola.
		  * \throw std::logic_error if the number of edges is odd or less than four.
		  */
		static CPolyhedronPtr CreateCupola(uint32_t numBaseEdges,double edgeLength);
		/**
		  * Creates a trapezohedron whose dual is exactly an archimedean antiprism. Creates a cube if numBaseEdges is equal to 3.
		  * \todo Actually resulting height is significantly higher than that passed to the algorithm.
		  * \sa CreateTrapezohedron,CreateArchimedeanRegularAntiprism,getDual
		  */
		static CPolyhedronPtr CreateCatalanTrapezohedron(uint32_t numBaseEdges,double height);
		/**
		  * Creates a double pyramid whose dual is exactly an archimedean prism. Creates an octahedron if numBaseEdges is equal to 4.
		  * \todo Actually resulting height is significantly higher than that passed to the algorithm.
		  * \sa CreateDoublePyramid,CreateArchimedeanRegularPrism,getDual
		  */
		static CPolyhedronPtr CreateCatalanDoublePyramid(uint32_t numBaseEdges,double height);
		/**
		  * Creates a series of concatenated solids (most of which are prismatoids) whose base is a regular polygon with a given number of edges. Every face of the resulting body will be a regular polygon, so it is a Johnson solid; in special cases, it may be archimedean or even platonic.
		  * The shape of the body is defined by the string argument, which can include one or more of the following:
		  <center><table>
		  <tr><td><b>String</b></td><td><b>Body</b></td><td><b>Restrictions</b></td></tr>
		  <tr><td>P+</td><td>Upward pointing pyramid</td><td>Must be the last object, vertex number cannot surpass 5</td></tr>
		  <tr><td>P-</td><td>Downward pointing pyramid</td><td>Must be the first object, vertex number cannot surpass 5</td></tr>
		  <tr><td>C+</td><td>Upward pointing cupola</td><td>Must be the last object, vertex number must be an even number in the range 4-10.</td></tr>
		  <tr><td>C-</td><td>Downward pointing cupola</td><td>Must be the first object, vertex number must be an even number in the range 4-10.</td></tr>
		  <tr><td>GC+</td><td>Upward pointing cupola, rotated</td><td>Must be the last object, vertex number must be an even number in the range 4-10.</td></tr>
		  <tr><td>GC-</td><td>Downward pointing cupola, rotated</td><td>Must be the first object, vertex number must be an even number in the range 4-10.</td></tr>
		  <tr><td>PR</td><td>Archimedean prism</td><td>Cannot abut other prism</td></tr>
		  <tr><td>A</td><td>Archimedean antiprism</td><td>None</td></tr>
		  <tr><td>R+</td><td>Upward pointing rotunda</td><td>Must be the last object, vertex number must be exactly 10</td></tr>
		  <tr><td>R-</td><td>Downward pointing rotunda</td><td>Must be the first object, vertex number must be exactly 10</td></tr>
		  <tr><td>GR+</td><td>Upward pointing rotunda, rotated</td><td>Must be the last object, vertex number must be exactly 10</td></tr>
		  <tr><td>GR-</td><td>Downward pointing rotunda</td><td>Must be the first object, vertex number must be exactly 10</td></tr>
		  </table></center>
		  * Some examples of bodies are:
		  <center><table>
		  <tr><td><b>String</b></td><td><b>Vertices</b></td><td><b>Resulting body</b></td></tr>
		  <tr><td>P+</td><td align="center">3</td><td>Tetrahedron</td></tr>
		  <tr><td>PR</td><td align="center">4</td><td>Hexahedron</td></tr>
		  <tr><td>P-P+</td><td align="center">4</td><td>Octahedron</td></tr>
		  <tr><td>A</td><td align="center">3</td><td>Octahedron</td></tr>
		  <tr><td>C+PRC-</td><td align="center">8</td><td>Rhombicuboctahedron</td></tr>
		  <tr><td>P-AP+</td><td align="center">5</td><td>Icosahedron</td></tr>
		  <tr><td>R-R+</td><td align="center">10</td><td>Icosidodecahedron</td></tr>
		  </table></center>
		  */
		static CPolyhedronPtr CreateJohnsonSolidWithConstantBase(uint32_t numBaseEdges,double baseRadius,const std::string &components,size_t shifts=0);
		/** @}
		 */

		/**
		  * Render
		  * \sa CRenderizable
		  */
		void render_dl() const MRPT_OVERRIDE;
		/**
		  * Ray trace
		  * \sa CRenderizable
		  */
		bool traceRay(const mrpt::poses::CPose3D &o,double &dist) const MRPT_OVERRIDE;
		/**
		  * Gets a list with the polyhedron's vertices.
		  */
		inline void getVertices(std::vector<mrpt::math::TPoint3D> &vertices) const	{
			vertices=mVertices;
		}
		/**
		  * Gets a list with the polyhedron's edges.
		  */
		inline void getEdges(std::vector<TPolyhedronEdge> &edges) const	{
			edges=mEdges;
		}
		/**
		  * Gets a list with the polyhedron's faces.
		  */
		inline void getFaces(std::vector<TPolyhedronFace> &faces) const	{
			faces=mFaces;
		}
		/**
		  * Gets the amount of vertices.
		  */
		inline uint32_t getNumberOfVertices() const	{
			return mVertices.size();
		}
		/**
		  * Gets the amount of edges.
		  */
		inline uint32_t getNumberOfEdges() const	{
			return mEdges.size();
		}
		/**
		  * Gets the amount of faces.
		  */
		inline uint32_t getNumberOfFaces() const	{
			return mFaces.size();
		}
		/**
		  * Gets a vector with each edge's length.
		  */
		void getEdgesLength(std::vector<double> &lengths) const;
		/**
		  * Gets a vector with each face's area. Won't work properly if the polygons are not convex.
		  */
		void getFacesArea(std::vector<double> &areas) const;
		/**
		  * Gets the polyhedron volume. Won't work properly if the polyhedron is not convex.
		  */
		double getVolume() const;
		/**
		  * Returns whether the polyhedron will be rendered as a wireframe object.
		  */
		inline bool isWireframe() const	{
			return mWireframe;
		}
		/**
		  * Sets whether the polyhedron will be rendered as a wireframe object.
		  */
		inline void setWireframe(bool enabled=true)	{
			mWireframe=enabled;
			CRenderizableDisplayList::notifyChange();
		}
		/**
		  * Gets the wireframe lines width.
		  */
		inline double getLineWidth() const	{
			return mLineWidth;
		}
		/**
		  * Sets the width used to render lines, when wireframe rendering is activated.
		  */
		inline void setLineWidth(double lineWidth)	{
			mLineWidth=lineWidth;
			CRenderizableDisplayList::notifyChange();
		}
		/**
		  * Gets the polyhedron as a set of polygons.
		  * \sa mrpt::math::TPolygon3D
		  */
		void getSetOfPolygons(std::vector<math::TPolygon3D> &vec) const;
		/**
		  * Gets the polyhedron as a set of polygons, with the pose transformation already applied.
		  * \sa mrpt::math::TPolygon3D,mrpt::poses::CPose3D
		  */
		void getSetOfPolygonsAbsolute(std::vector<math::TPolygon3D> &vec) const;
		/** Gets the intersection of two polyhedra, either as a set or as a matrix of intersections. Each intersection is represented by a TObject3D.
		  * \sa mrpt::math::TObject3D
		  */
		template<class T> inline static size_t getIntersection(const CPolyhedronPtr &p1,const CPolyhedronPtr &p2,T &container);
		/**
		  * Returns true if the polygon is a completely closed object.
		  */
		inline bool isClosed() const	{
			for (size_t i=0;i<mVertices.size();i++) if (edgesInVertex(i)!=facesInVertex(i)) return false;
			return true;
		}
		/**
		  * Recomputes polygons, if necessary, so that each one is convex.
		  */
		void makeConvexPolygons();
		/**
		  * Gets the center of the polyhedron.
		  */
		void getCenter(mrpt::math::TPoint3D &center) const;
		/**
		  * Creates a random polyhedron from the static methods.
		  */
		static CPolyhedronPtr CreateRandomPolyhedron(double radius);

		/** @name Polyhedron special operations.
			@{
		 */
		/**
		  * Given a polyhedron, creates its dual.
		  * \sa truncate,cantellate,augment
		  * \throw std::logic_error Can't get the dual to this polyhedron.
		  */
		CPolyhedronPtr getDual() const;
		/**
		  * Truncates a polyhedron to a given factor.
		  * \sa getDual,cantellate,augment
		  * \throw std::logic_error Polyhedron truncation results in skew polygons and thus it's impossible to perform.
		  */
		CPolyhedronPtr truncate(double factor) const;
		/**
		  * Cantellates a polyhedron to a given factor.
		  * \sa getDual,truncate,augment
		  */
		CPolyhedronPtr cantellate(double factor) const;
		/**
		  * Augments a polyhedron to a given height. This operation is roughly dual to the truncation: given a body P, the operation dtdP and aP yield resembling results.
		  * \sa getDual,truncate,cantellate
		  */
		CPolyhedronPtr augment(double height) const;
		/**
		  * Augments a polyhedron to a given height. This method only affects to faces with certain number of vertices.
		  * \sa augment(double) const
		  */
		CPolyhedronPtr augment(double height,size_t numVertices) const;
		/**
		  * Augments a polyhedron, so that the resulting triangles are equilateral. If the argument is true, triangles are "cut" from the polyhedron, instead of being added.
		  * \throw std::logic_error a non-regular face has been found.
		  * \sa augment(double) const
		  */
		CPolyhedronPtr augment(bool direction=false) const;
		/**
		  * Augments a polyhedron, so that the resulting triangles are equilateral; affects only faces with certain number of faces. If the second argument is true, triangles are "cut" from the polyhedron.
		  * \throw std::logic_error a non-regular face has been found.
		  * \sa augment(double) const
		  */
		CPolyhedronPtr augment(size_t numVertices,bool direction=false) const;
		/**
		  * Rotates a polyhedron around the Z axis a given amount of radians. In some cases, this operation may be necessary to view the symmetry between related objects.
		  *	\sa scale
		  */
		CPolyhedronPtr rotate(double angle) const;
		/**
		  * Scales a polyhedron to a given factor.
		  * \throw std::logic_error factor is not a strictly positive number.
		  * \sa rotate
		  */
		CPolyhedronPtr scale(double factor) const;
		/** @}
		 */
		/**
		  * Updates the mutable list of polygons used in rendering and ray tracing.
		  */
		void updatePolygons() const;
	private:
		/**
		  * Generates a list of 2D vertices constituting a regular polygon.
		  */
		static std::vector<mrpt::math::TPoint2D> generateBase(uint32_t numBaseEdges,double baseRadius);
		/**
		  * Generates a list of 2D vertices constituting a regular polygon, with an angle shift which makes it suitable for antiprisms.
		  */
		static std::vector<mrpt::math::TPoint2D> generateShiftedBase(uint32_t numBaseEdges,double baseRadius);
		/**
		  * Generates a list of 3D vertices constituting a regular polygon, appending it to an existing vector.
		  */
		static void generateBase(uint32_t numBaseEdges,double baseRadius,double height,std::vector<mrpt::math::TPoint3D> &vec);
		/**
		  * Generates a list of 3D vertices constituting a regular polygon conveniently shifted, appending it to an existing vector.
		  */
		static void generateShiftedBase(uint32_t numBaseEdges,double baseRadius,double height,double shift,std::vector<mrpt::math::TPoint3D> &vec);
		/**
		  * Calculates the normal vector to a face.
		  */
		bool setNormal(TPolyhedronFace &f,bool doCheck=true);
		/**
		  * Adds, to the existing list of edges, each edge in a given face.
		  */
		void addEdges(const TPolyhedronFace &e);
		/**
		  * Checks whether a set of faces is suitable for a set of vertices.
		  */
		static bool checkConsistence(const std::vector<mrpt::math::TPoint3D> &vertices,const std::vector<TPolyhedronFace> &faces);
		/**
		  * Returns how many edges converge in a given vertex.
		  */
		size_t edgesInVertex(size_t vertex) const;
		/**
		  * Returns how many faces converge in a given vertex.
		  */
		size_t facesInVertex(size_t vertex) const;
		/**
		  * Basic empty constructor.
		  */
		inline CPolyhedron():mVertices(),mEdges(),mFaces(),mWireframe(false),mLineWidth(1),polygonsUpToDate(false)	{}
		/**
		  * Basic constructor with a list of vertices and another of faces, checking for correctness.
		  */
		inline CPolyhedron(const std::vector<mrpt::math::TPoint3D> &vertices,const std::vector<TPolyhedronFace> &faces,bool doCheck=true):mVertices(vertices),mEdges(),mFaces(faces),mWireframe(false),mLineWidth(1),polygonsUpToDate(false)	{
			if (doCheck) if (!checkConsistence(vertices,faces)) throw std::logic_error("Face list accesses a vertex out of range");
			for (std::vector<TPolyhedronFace>::iterator it=mFaces.begin();it!=mFaces.end();++it)	{
				if (!setNormal(*it,doCheck)) throw std::logic_error("Bad face specification");
				addEdges(*it);
			}
		}
		/** Creates a polyhedron without checking its correctness. */
		static CPolyhedronPtr CreateNoCheck(const std::vector<mrpt::math::TPoint3D> &vertices,const std::vector<TPolyhedronFace> &faces);
		/** Creates an empty Polyhedron. */
		static CPolyhedronPtr CreateEmpty();
		/** Destructor. */
		virtual ~CPolyhedron()	{}
	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(CPolyhedron,CRenderizableDisplayList, OPENGL_IMPEXP)

	// Implemented after the definition of SmartPtrs in the _POST() macro above.
	template<class T>
	size_t CPolyhedron::getIntersection(const CPolyhedronPtr &p1,const CPolyhedronPtr &p2,T &container)	{
		std::vector<mrpt::math::TPolygon3D> polys1,polys2;
		p1->getSetOfPolygonsAbsolute(polys1);
		p2->getSetOfPolygonsAbsolute(polys2);
		return mrpt::math::intersect(polys1,polys2,container);
	}

	/**
	  * Reads a polyhedron edge from a binary stream.
	  */
	OPENGL_IMPEXP mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in,CPolyhedron::TPolyhedronEdge &o);
	/**
	  * Writes a polyhedron edge to a binary stream.
	  */
	OPENGL_IMPEXP mrpt::utils::CStream& operator<<(mrpt::utils::CStream& out,const CPolyhedron::TPolyhedronEdge &o);
	/**
	  * Reads a polyhedron face from a binary stream.
	  */
	OPENGL_IMPEXP mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in,CPolyhedron::TPolyhedronFace &o);
	/**
	  * Writes a polyhedron face to a binary stream.
	  */
	OPENGL_IMPEXP mrpt::utils::CStream& operator<<(mrpt::utils::CStream& out,const CPolyhedron::TPolyhedronFace &o);
}
	namespace utils	{
		// Specialization must occur in the same namespace
		MRPT_DECLARE_TTYPENAME_NAMESPACE(CPolyhedron::TPolyhedronEdge,mrpt::opengl)
		MRPT_DECLARE_TTYPENAME_NAMESPACE(CPolyhedron::TPolyhedronFace,mrpt::opengl)
	}
}
#endif
