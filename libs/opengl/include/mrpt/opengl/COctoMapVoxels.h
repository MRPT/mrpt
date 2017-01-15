/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef opengl_COctoMapVoxels_H
#define opengl_COctoMapVoxels_H

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/math/lightweight_geom_data.h>

namespace mrpt
{
	namespace opengl
	{
		enum predefined_voxel_sets_t
		{
			VOXEL_SET_OCCUPIED = 0,
			VOXEL_SET_FREESPACE = 1
		};

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( COctoMapVoxels, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** A flexible renderer of voxels, typically from a 3D octo map (see mrpt::maps::COctoMap).
		  *  This class is sort of equivalent to octovis::OcTreeDrawer from the octomap package, but
		  *  relying on MRPT's CRenderizableDisplayList so there's no need to manually cache the rendering of OpenGL primitives.
		  *
		  *  Normally users call mrpt::maps::COctoMap::getAs3DObject() to obtain a generic mrpt::opengl::CSetOfObjects which insides holds an instance of COctoMapVoxels.
		  *  You can also alternativelly call COctoMapVoxels::setFromOctoMap(), so you can tune the display parameters, colors, etc.
		  *  As with any other mrpt::opengl class, all object coordinates refer to some frame of reference which is relative to the object parent and can be changed with mrpt::opengl::CRenderizable::setPose()
		  *
		  *  This class draws these separate elements to represent an OctoMap:
		  *		- A grid representation of all cubes, as simple lines (occupied/free, leafs/nodes,... whatever). See:
		  *			- showGridLines()
		  *			- setGridLinesColor()
		  *			- setGridLinesWidth()
		  *			- push_back_GridCube()
		  *		- A number of <b>voxel collections</b>, drawn as cubes each having a different color (e.g. depending on the color scheme in the original mrpt::maps::COctoMap object).
		  *       The meanning of each collection is user-defined, but you can use the constants VOXEL_SET_OCCUPIED, VOXEL_SET_FREESPACE for predefined meanings.
		  *			- showVoxels()
		  *			- push_back_Voxel()
		  *
		  *  Several coloring schemes can be selected with setVisualizationMode(). See COctoMapVoxels::visualization_mode_t
		  *
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *   <tr> <td> mrpt::opengl::COctoMapVoxels </td> <td> \image html preview_COctoMapVoxels.png </td> </tr>
		  *  </table>
		  *  </div>
		  *
		  *  \sa opengl::COpenGLScene
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP COctoMapVoxels : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( COctoMapVoxels )
		public:

			/** The different coloring schemes, which modulate the generic mrpt::opengl::CRenderizable object color. Set with setVisualizationMode() */
			enum visualization_mode_t
			{
				COLOR_FROM_HEIGHT,				//!< Color goes from black (at the bottom) to the chosen color (at the top)
				COLOR_FROM_OCCUPANCY,			//!< Color goes from black (occupied voxel) to the chosen color (free voxel)
				TRANSPARENCY_FROM_OCCUPANCY,	//!< Transparency goes from opaque (occupied voxel) to transparent (free voxel).
				TRANS_AND_COLOR_FROM_OCCUPANCY,	//!< Color goes from black (occupaid voxel) to the chosen color (free voxel) and they are transparent
				MIXED,							//!< Combination of COLOR_FROM_HEIGHT and TRANSPARENCY_FROM_OCCUPANCY
				FIXED							//!< All cubes are of identical color.
			};


			/** The info of each of the voxels */
			struct OPENGL_IMPEXP TVoxel
			{
				mrpt::math::TPoint3D coords;
				double               side_length;
				mrpt::utils::TColor  color;

				TVoxel() {}
				TVoxel(const mrpt::math::TPoint3D &coords_, const double side_length_, mrpt::utils::TColor color_) : coords(coords_), side_length(side_length_),color(color_) {}
			};

			/** The info of each grid block */
			struct OPENGL_IMPEXP TGridCube
			{
				mrpt::math::TPoint3D  min,max; //!< opposite corners of the cube

				TGridCube() {}
				TGridCube(const mrpt::math::TPoint3D &min_,const mrpt::math::TPoint3D &max_) : min(min_),max(max_) { }
			};

			struct OPENGL_IMPEXP TInfoPerVoxelSet
			{
				bool                 visible;
				std::vector<TVoxel>  voxels;

				TInfoPerVoxelSet() : visible(true) {}
			};
		protected:

			std::deque<TInfoPerVoxelSet>   m_voxel_sets;
			std::vector<TGridCube>         m_grid_cubes;

			mrpt::math::TPoint3D   m_bb_min, m_bb_max; //!< Cached bounding boxes

			bool					m_enable_lighting;
			bool					m_enable_cube_transparency;
			bool					m_showVoxelsAsPoints;
			float					m_showVoxelsAsPointsSize;
			bool					m_show_grids;
            float					m_grid_width;
			mrpt::utils::TColor		m_grid_color;
			visualization_mode_t	m_visual_mode;

		public:

			/** Clears everything */
			void clear();

			/** Select the visualization mode. To have any effect, this method has to be called before loading the octomap. */
			inline void setVisualizationMode(visualization_mode_t mode) {
				m_visual_mode = mode; CRenderizableDisplayList::notifyChange();
			}
			inline visualization_mode_t getVisualizationMode() const {return m_visual_mode;}

			/** Can be used to enable/disable the effects of lighting in this object */
			inline void enableLights(bool enable) { m_enable_lighting=enable; CRenderizableDisplayList::notifyChange(); }
			inline bool areLightsEnabled() const { return m_enable_lighting; }



			/** By default, the alpha (transparency) component of voxel cubes is taken into account, but transparency can be disabled with this method. */
			inline void enableCubeTransparency(bool enable) { m_enable_cube_transparency=enable;  CRenderizableDisplayList::notifyChange(); }
			inline bool isCubeTransparencyEnabled() const { return m_enable_cube_transparency; }

			/** Shows/hides the grid lines */
			inline void showGridLines(bool show) { m_show_grids=show; CRenderizableDisplayList::notifyChange(); }
			inline bool areGridLinesVisible() const { return m_show_grids; }

			/** Shows/hides the voxels (voxel_set is a 0-based index for the set of voxels to modify, e.g. VOXEL_SET_OCCUPIED, VOXEL_SET_FREESPACE) */
			inline void showVoxels(unsigned int voxel_set, bool show) { ASSERT_(voxel_set<m_voxel_sets.size()) m_voxel_sets[voxel_set].visible=show; CRenderizableDisplayList::notifyChange(); }
			inline bool areVoxelsVisible(unsigned int voxel_set) const { ASSERT_(voxel_set<m_voxel_sets.size()) return m_voxel_sets[voxel_set].visible; }

			/** For quick renders: render voxels as points instead of cubes. \sa setVoxelAsPointsSize */
			inline void showVoxelsAsPoints(const bool enable) { m_showVoxelsAsPoints=enable; CRenderizableDisplayList::notifyChange(); }
			inline bool areVoxelsShownAsPoints() const { return m_showVoxelsAsPoints; }

			/** Only used when showVoxelsAsPoints() is enabled.  */
			inline void setVoxelAsPointsSize(float pointSize) { m_showVoxelsAsPointsSize=pointSize; CRenderizableDisplayList::notifyChange(); }
			inline float getVoxelAsPointsSize() const { return m_showVoxelsAsPointsSize; }

			/** Sets the width of grid lines */
			inline void setGridLinesWidth(float w) { m_grid_width=w; CRenderizableDisplayList::notifyChange(); }
			/** Gets the width of grid lines */
			inline float getGridLinesWidth() const { return m_grid_width; }

			inline void setGridLinesColor(const mrpt::utils::TColor &color) { m_grid_color=color;  CRenderizableDisplayList::notifyChange(); }
			inline const mrpt::utils::TColor & getGridLinesColor() const { return m_grid_color; }

			/** Returns the total count of grid cubes. */
			inline size_t getGridCubeCount() const	{ return m_grid_cubes.size(); }
			/** Returns the number of voxel sets. */
			inline size_t getVoxelSetCount() const { return m_voxel_sets.size(); }
			/** Returns the total count of voxels in one voxel set. */
			inline size_t getVoxelCount(const size_t set_index) const	{ ASSERT_(set_index<m_voxel_sets.size()) return m_voxel_sets[set_index].voxels.size(); }

			/** Manually changes the bounding box (normally the user doesn't need to call this) */
			void setBoundingBox(const mrpt::math::TPoint3D &bb_min, const mrpt::math::TPoint3D &bb_max);

			inline void resizeGridCubes(const size_t nCubes) { m_grid_cubes.resize(nCubes); CRenderizableDisplayList::notifyChange(); }
			inline void resizeVoxelSets(const size_t nVoxelSets) { m_voxel_sets.resize(nVoxelSets); CRenderizableDisplayList::notifyChange(); }
			inline void resizeVoxels(const size_t set_index, const size_t nVoxels) {  ASSERT_(set_index<m_voxel_sets.size())  m_voxel_sets[set_index].voxels.resize(nVoxels); CRenderizableDisplayList::notifyChange(); }

			inline void reserveGridCubes(const size_t nCubes) { m_grid_cubes.reserve(nCubes); }
			inline void reserveVoxels(const size_t set_index, const size_t nVoxels) {  ASSERT_(set_index<m_voxel_sets.size())  m_voxel_sets[set_index].voxels.reserve(nVoxels); CRenderizableDisplayList::notifyChange(); }

			inline TGridCube & getGridCubeRef(const size_t idx) { ASSERTDEB_(idx<m_grid_cubes.size()) CRenderizableDisplayList::notifyChange(); return m_grid_cubes[idx]; }
			inline const TGridCube & getGridCube(const size_t idx) const { ASSERTDEB_(idx<m_grid_cubes.size()) return m_grid_cubes[idx]; }

			inline TVoxel & getVoxelRef(const size_t set_index, const size_t idx) { ASSERTDEB_(set_index<m_voxel_sets.size() && idx<m_voxel_sets[set_index].voxels.size()) CRenderizableDisplayList::notifyChange(); return m_voxel_sets[set_index].voxels[idx]; }
			inline const TVoxel & getVoxel(const size_t set_index, const size_t idx) const { ASSERTDEB_(set_index<m_voxel_sets.size() && idx<m_voxel_sets[set_index].voxels.size()) CRenderizableDisplayList::notifyChange(); return m_voxel_sets[set_index].voxels[idx]; }

			inline void push_back_GridCube(const TGridCube & c) { CRenderizableDisplayList::notifyChange(); m_grid_cubes.push_back(c); }
			inline void push_back_Voxel(const size_t set_index, const TVoxel & v) { ASSERTDEB_(set_index<m_voxel_sets.size()) CRenderizableDisplayList::notifyChange(); m_voxel_sets[set_index].voxels.push_back(v); }

			void sort_voxels_by_z();

			/** Render */
			void  render_dl() const MRPT_OVERRIDE;

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const MRPT_OVERRIDE;

			/** Sets the contents of the object from a mrpt::maps::COctoMap object.
			  * \tparam Typically, an mrpt::maps::COctoMap object
			  *
			  * \note Declared as a template because in the library [mrpt-opengl] we don't have access to the library [mrpt-maps].
			  */
			template <class OCTOMAP>
			void setFromOctoMap(OCTOMAP &m) {
				m.getAsOctoMapVoxels(*this);
			}

		private:
			/** Constructor */
			COctoMapVoxels();
			/** Private, virtual destructor: only can be deleted from smart pointers. */
			virtual ~COctoMapVoxels() { }
		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( COctoMapVoxels, CRenderizableDisplayList, OPENGL_IMPEXP )

	} // end namespace
} // End of namespace

#endif
