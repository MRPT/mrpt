/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/TPoint3D.h>
#include <mrpt/opengl/CRenderizable.h>

namespace mrpt::opengl
{
enum predefined_voxel_sets_t
{
	VOXEL_SET_OCCUPIED = 0,
	VOXEL_SET_FREESPACE = 1
};

/** A flexible renderer of voxels, typically from a 3D octo map (see
 *mrpt::maps::COctoMap).
 *  This class is sort of equivalent to octovis::OcTreeDrawer from the octomap
 *package, but
 *  relying on MRPT's CRenderizableShader so there's no need to manually
 *cache the rendering of OpenGL primitives.
 *
 *  Normally users call mrpt::maps::COctoMap::getAs3DObject() to obtain a
 *generic mrpt::opengl::CSetOfObjects which insides holds an instance of
 *COctoMapVoxels.
 *  You can also alternativelly call COctoMapVoxels::setFromOctoMap(), so you
 *can tune the display parameters, colors, etc.
 *  As with any other mrpt::opengl class, all object coordinates refer to some
 *frame of reference which is relative to the object parent and can be changed
 *with mrpt::opengl::CRenderizable::setPose()
 *
 *  This class draws these separate elements to represent an OctoMap:
 *		- A grid representation of all cubes, as simple lines (occupied/free,
 *leafs/nodes,... whatever). See:
 *			- showGridLines()
 *			- setGridLinesColor()
 *			- setGridLinesWidth()
 *			- push_back_GridCube()
 *		- A number of <b>voxel collections</b>, drawn as cubes each having a
 *different color (e.g. depending on the color scheme in the original
 *mrpt::maps::COctoMap object).
 *       The meanning of each collection is user-defined, but you can use the
 *constants VOXEL_SET_OCCUPIED, VOXEL_SET_FREESPACE for predefined meanings.
 *			- showVoxels()
 *			- push_back_Voxel()
 *
 *  Several coloring schemes can be selected with setVisualizationMode(). See
 *COctoMapVoxels::visualization_mode_t
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 *border-style: solid;">
 *   <tr> <td> mrpt::opengl::COctoMapVoxels </td> <td> \image html
 *preview_COctoMapVoxels.png </td> </tr>
 *  </table>
 *  </div>
 *
 *  \sa opengl::COpenGLScene
 * \ingroup mrpt_opengl_grp
 */
class COctoMapVoxels : public CRenderizable
{
	DEFINE_SERIALIZABLE(COctoMapVoxels, mrpt::opengl)
   public:
	/** The different coloring schemes, which modulate the generic
	 * mrpt::opengl::CRenderizable object color. Set with setVisualizationMode()
	 */
	enum visualization_mode_t
	{
		/** Color goes from black (at the bottom) to the chosen color (at the
		   top) */
		COLOR_FROM_HEIGHT,
		/** Color goes from black (occupied voxel) to the chosen color (free
		   voxel) */
		COLOR_FROM_OCCUPANCY,
		/** Transparency goes from opaque (occupied voxel) to transparent (free
		   voxel). */
		TRANSPARENCY_FROM_OCCUPANCY,
		/** Color goes from black (occupaid voxel) to the chosen color (free
		   voxel) and they are transparent */
		TRANS_AND_COLOR_FROM_OCCUPANCY,
		/** Combination of COLOR_FROM_HEIGHT and TRANSPARENCY_FROM_OCCUPANCY */
		MIXED,
		/** All cubes are of identical color. */
		FIXED
	};

	/** The info of each of the voxels */
	struct TVoxel
	{
		mrpt::math::TPoint3D coords;
		double side_length;
		mrpt::img::TColor color;

		TVoxel() = default;
		TVoxel(
			const mrpt::math::TPoint3D& coords_, const double side_length_,
			mrpt::img::TColor color_)
			: coords(coords_), side_length(side_length_), color(color_)
		{
		}
	};

	/** The info of each grid block */
	struct TGridCube
	{
		/** opposite corners of the cube */
		mrpt::math::TPoint3D min, max;

		TGridCube() = default;
		TGridCube(
			const mrpt::math::TPoint3D& min_, const mrpt::math::TPoint3D& max_)
			: min(min_), max(max_)
		{
		}
	};

	struct TInfoPerVoxelSet
	{
		bool visible{true};
		std::vector<TVoxel> voxels;

		TInfoPerVoxelSet() = default;
	};

   protected:
	std::deque<TInfoPerVoxelSet> m_voxel_sets;
	std::vector<TGridCube> m_grid_cubes;

	/** Cached bounding boxes */
	mrpt::math::TPoint3D m_bb_min, m_bb_max;

	bool m_enable_lighting{false};
	bool m_enable_cube_transparency{true};
	bool m_showVoxelsAsPoints{false};
	float m_showVoxelsAsPointsSize{3.0f};
	bool m_show_grids{false};
	float m_grid_width{1.0f};
	mrpt::img::TColor m_grid_color;
	visualization_mode_t m_visual_mode{COctoMapVoxels::COLOR_FROM_OCCUPANCY};

   public:
	/** Clears everything */
	void clear();

	/** Select the visualization mode. To have any effect, this method has to be
	 * called before loading the octomap. */
	inline void setVisualizationMode(visualization_mode_t mode)
	{
		m_visual_mode = mode;
		CRenderizable::notifyChange();
	}
	inline visualization_mode_t getVisualizationMode() const
	{
		return m_visual_mode;
	}

	/** Can be used to enable/disable the effects of lighting in this object */
	inline void enableLights(bool enable)
	{
		m_enable_lighting = enable;
		CRenderizable::notifyChange();
	}
	inline bool areLightsEnabled() const { return m_enable_lighting; }
	/** By default, the alpha (transparency) component of voxel cubes is taken
	 * into account, but transparency can be disabled with this method. */
	inline void enableCubeTransparency(bool enable)
	{
		m_enable_cube_transparency = enable;
		CRenderizable::notifyChange();
	}
	inline bool isCubeTransparencyEnabled() const
	{
		return m_enable_cube_transparency;
	}

	/** Shows/hides the grid lines */
	inline void showGridLines(bool show)
	{
		m_show_grids = show;
		CRenderizable::notifyChange();
	}
	inline bool areGridLinesVisible() const { return m_show_grids; }
	/** Shows/hides the voxels (voxel_set is a 0-based index for the set of
	 * voxels to modify, e.g. VOXEL_SET_OCCUPIED, VOXEL_SET_FREESPACE) */
	inline void showVoxels(unsigned int voxel_set, bool show)
	{
		ASSERT_(voxel_set < m_voxel_sets.size());
		m_voxel_sets[voxel_set].visible = show;
		CRenderizable::notifyChange();
	}
	inline bool areVoxelsVisible(unsigned int voxel_set) const
	{
		ASSERT_(voxel_set < m_voxel_sets.size());
		return m_voxel_sets[voxel_set].visible;
	}

	/** For quick renders: render voxels as points instead of cubes. \sa
	 * setVoxelAsPointsSize */
	inline void showVoxelsAsPoints(const bool enable)
	{
		m_showVoxelsAsPoints = enable;
		CRenderizable::notifyChange();
	}
	inline bool areVoxelsShownAsPoints() const { return m_showVoxelsAsPoints; }
	/** Only used when showVoxelsAsPoints() is enabled.  */
	inline void setVoxelAsPointsSize(float pointSize)
	{
		m_showVoxelsAsPointsSize = pointSize;
		CRenderizable::notifyChange();
	}
	inline float getVoxelAsPointsSize() const
	{
		return m_showVoxelsAsPointsSize;
	}

	/** Sets the width of grid lines */
	inline void setGridLinesWidth(float w)
	{
		m_grid_width = w;
		CRenderizable::notifyChange();
	}
	/** Gets the width of grid lines */
	inline float getGridLinesWidth() const { return m_grid_width; }
	inline void setGridLinesColor(const mrpt::img::TColor& color)
	{
		m_grid_color = color;
		CRenderizable::notifyChange();
	}
	inline const mrpt::img::TColor& getGridLinesColor() const
	{
		return m_grid_color;
	}

	/** Returns the total count of grid cubes. */
	inline size_t getGridCubeCount() const { return m_grid_cubes.size(); }
	/** Returns the number of voxel sets. */
	inline size_t getVoxelSetCount() const { return m_voxel_sets.size(); }
	/** Returns the total count of voxels in one voxel set. */
	inline size_t getVoxelCount(const size_t set_index) const
	{
		ASSERT_(set_index < m_voxel_sets.size());
		return m_voxel_sets[set_index].voxels.size();
	}

	/** Manually changes the bounding box (normally the user doesn't need to
	 * call this) */
	void setBoundingBox(
		const mrpt::math::TPoint3D& bb_min, const mrpt::math::TPoint3D& bb_max);

	inline void resizeGridCubes(const size_t nCubes)
	{
		m_grid_cubes.resize(nCubes);
		CRenderizable::notifyChange();
	}
	inline void resizeVoxelSets(const size_t nVoxelSets)
	{
		m_voxel_sets.resize(nVoxelSets);
		CRenderizable::notifyChange();
	}
	inline void resizeVoxels(const size_t set_index, const size_t nVoxels)
	{
		ASSERT_(set_index < m_voxel_sets.size());
		m_voxel_sets[set_index].voxels.resize(nVoxels);
		CRenderizable::notifyChange();
	}

	inline void reserveGridCubes(const size_t nCubes)
	{
		m_grid_cubes.reserve(nCubes);
	}
	inline void reserveVoxels(const size_t set_index, const size_t nVoxels)
	{
		ASSERT_(set_index < m_voxel_sets.size());
		m_voxel_sets[set_index].voxels.reserve(nVoxels);
		CRenderizable::notifyChange();
	}

	inline TGridCube& getGridCubeRef(const size_t idx)
	{
		ASSERTDEB_(idx < m_grid_cubes.size());
		CRenderizable::notifyChange();
		return m_grid_cubes[idx];
	}
	inline const TGridCube& getGridCube(const size_t idx) const
	{
		ASSERTDEB_(idx < m_grid_cubes.size());
		return m_grid_cubes[idx];
	}

	inline TVoxel& getVoxelRef(const size_t set_index, const size_t idx)
	{
		ASSERTDEB_(
			set_index < m_voxel_sets.size() &&
			idx < m_voxel_sets[set_index].voxels.size());
		CRenderizable::notifyChange();
		return m_voxel_sets[set_index].voxels[idx];
	}
	inline const TVoxel& getVoxel(
		const size_t set_index, const size_t idx) const
	{
		ASSERTDEB_(
			set_index < m_voxel_sets.size() &&
			idx < m_voxel_sets[set_index].voxels.size());
		CRenderizable::notifyChange();
		return m_voxel_sets[set_index].voxels[idx];
	}

	inline void push_back_GridCube(const TGridCube& c)
	{
		CRenderizable::notifyChange();
		m_grid_cubes.push_back(c);
	}
	inline void push_back_Voxel(const size_t set_index, const TVoxel& v)
	{
		ASSERTDEB_(set_index < m_voxel_sets.size());
		CRenderizable::notifyChange();
		m_voxel_sets[set_index].voxels.push_back(v);
	}

	void sort_voxels_by_z();

	void render() const override;
	void renderUpdateBuffers() const override;
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	/** Sets the contents of the object from a mrpt::maps::COctoMap object.
	 * \tparam Typically, an mrpt::maps::COctoMap object
	 *
	 * \note Declared as a template because in the library [mrpt-opengl] we
	 * don't have access to the library [mrpt-maps].
	 */
	template <class OCTOMAP>
	void setFromOctoMap(OCTOMAP& m)
	{
		m.getAsOctoMapVoxels(*this);
	}

	/** Constructor */
	COctoMapVoxels();
	/** Private, virtual destructor: only can be deleted from smart pointers. */
	~COctoMapVoxels() override = default;
};

}  // namespace mrpt::opengl
