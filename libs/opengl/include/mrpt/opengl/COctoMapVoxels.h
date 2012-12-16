/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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

#ifndef opengl_COctoMapVoxels_H
#define opengl_COctoMapVoxels_H

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/utils/stl_extensions.h>

namespace mrpt
{
	namespace opengl
	{
		using mrpt::math::TPoint3D;
		using mrpt::utils::TColor;
		class OPENGL_IMPEXP COctoMapVoxels;

		enum predefined_voxel_sets_t
		{
			VOXEL_SET_OCCUPIED = 0, 
			VOXEL_SET_FREESPACE = 1
		};

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( COctoMapVoxels, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** A flexible renderer of voxels, typically from a 3D octo map (see mrpt::slam::COctoMap).
		  *  This class is sort of equivalent to octovis::OcTreeDrawer from the octomap package, but 
		  *  relying on MRPT's CRenderizableDisplayList so there's no need to manually cache the rendering of OpenGL primitives.
		  *
		  *  Normally users call mrpt::slam::COctoMap::getAs3DObject() to obtain a generic mrpt::opengl::CSetOfObjects which insides holds an instance of COctoMapVoxels.
		  *  You can also alternativelly call COctoMapVoxels::setFromOctoMap(), so you can tune the display parameters, colors, etc. 
		  *  As with any other mrpt::opengl class, all object coordinates refer to some frame of reference which is relative to the object parent and can be changed with mrpt::opengl::CRenderizable::setPose()
		  *
		  *  This class draws these separate elements to represent an OctoMap:
		  *		- A grid representation of all cubes, as simple lines (occupied/free, leafs/nodes,... whatever). See:
		  *			- showGridLines()
		  *			- setGridLinesColor()
		  *			- setGridLinesWidth()
		  *			- push_back_GridCube()
		  *		- A number of <b>voxel collections</b>, drawn as cubes each having a different color (e.g. depending on the color scheme in the original mrpt::slam::COctoMap object). 
		  *       The meanning of each collection is user-defined, but you can use the constants VOXEL_SET_OCCUPIED, VOXEL_SET_FREESPACE for predefined meanings.
		  *			- showVoxels()
		  *			- push_back_Voxel()
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

		protected:
			struct OPENGL_IMPEXP TInfoPerVoxelSet
			{
				bool                 visible;
				std::vector<TVoxel>  voxels;

				TInfoPerVoxelSet() : visible(true) {}
			};

			std::deque<TInfoPerVoxelSet>   m_voxel_sets; 
			std::vector<TGridCube>         m_grid_cubes;
			
			mrpt::math::TPoint3D   m_bb_min, m_bb_max; //!< Cached bounding boxes

			bool                   m_enable_lighting;
			bool                   m_show_grids;
            float                  m_grid_width;
			mrpt::utils::TColor    m_grid_color;

		public:

			/** Clears everything */
			void clear();

			inline void enableLights(bool enable) { m_enable_lighting=enable; CRenderizableDisplayList::notifyChange(); }
			inline bool areLightsEnabled() const { return m_enable_lighting; }

			/** Shows/hides the grid lines */
			inline void showGridLines(bool show) { m_show_grids=show; CRenderizableDisplayList::notifyChange(); }
			inline bool areGridLinesVisible() const { return m_show_grids; }

			/** Shows/hides the voxels (voxel_set is a 0-based index for the set of voxels to modify, e.g. VOXEL_SET_OCCUPIED, VOXEL_SET_FREESPACE) */
			inline void showVoxels(unsigned int voxel_set, bool show) { ASSERT_(voxel_set<m_voxel_sets.size()) m_voxel_sets[voxel_set].visible=show; CRenderizableDisplayList::notifyChange(); }
			inline bool areVoxelsVisible(unsigned int voxel_set) const { ASSERT_(voxel_set<m_voxel_sets.size()) return m_voxel_sets[voxel_set].visible; }

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

			/** Render */
			void  render_dl() const;

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			virtual void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const;

			/** Sets the contents of the object from a mrpt::slam::COctoMap object.
			  * \tparam Typically, an mrpt::slam::COctoMap object
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

	} // end namespace
} // End of namespace

#endif
