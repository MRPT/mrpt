/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/img/TColor.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <vector>
#include <string>

namespace mrpt::opengl
{
/** A virtual base class that implements the capability of importing 3D point
 * clouds and faces from a file in the Stanford PLY format.
 * \sa http://www.mrpt.org/Support_for_the_Stanford_3D_models_file_format_PLY
 * \sa PLY_Exporter
 * \ingroup mrpt_base_grp
 */
class PLY_Importer
{
   public:
	/** Loads from a PLY file.
	 * \param[in]  filename The filename to open. It can be either in binary or
	 * text format.
	 * \param[out] file_comments If provided (!=nullptr) the list of comment
	 * strings stored in the file will be returned.
	 * \param[out] file_obj_info If provided (!=nullptr) the list of "object
	 * info" strings stored in the file will be returned.
	 * \return false on any error in the file format or reading it. To obtain
	 * more details on the error you can call getLoadPLYErrorString()
	 */
	bool loadFromPlyFile(
		const std::string& filename,
		std::vector<std::string>* file_comments = nullptr,
		std::vector<std::string>* file_obj_info = nullptr);

	/** Return a description of the error if loadFromPlyFile() returned false,
	 * or an empty string if the file was loaded without problems. */
	std::string getLoadPLYErrorString() const
	{
		return m_ply_import_last_error;
	}

   protected:
	/** @name PLY Import virtual methods to implement in base classes
		@{ */

	/** In a base class, reserve memory to prepare subsequent calls to
	 * PLY_import_set_vertex */
	virtual void PLY_import_set_vertex_count(const size_t N) = 0;

	/** In a base class, reserve memory to prepare subsequent calls to
	 * PLY_import_set_face */
	virtual void PLY_import_set_face_count(const size_t N) = 0;

	/** In a base class, will be called after PLY_import_set_vertex_count() once
	 * for each loaded point.
	 *  \param pt_color Will be nullptr if the loaded file does not provide
	 * color info.
	 */
	virtual void PLY_import_set_vertex(
		const size_t idx, const mrpt::math::TPoint3Df& pt,
		const mrpt::img::TColorf* pt_color = nullptr) = 0;

	/** @} */

   private:
	std::string m_ply_import_last_error;

};  // End of class def.

/** A virtual base class that implements the capability of exporting 3D point
 * clouds and faces to a file in the Stanford PLY format.
 * \sa http://www.mrpt.org/Support_for_the_Stanford_3D_models_file_format_PLY
 * \sa PLY_Importer
 * \ingroup mrpt_base_grp
 */
class PLY_Exporter
{
   public:
	/** Saves to a PLY file.
	 * \param[in]  filename The filename to be saved.
	 * \param[in] file_comments If provided (!=nullptr) the list of comment
	 * strings stored in the file will be returned.
	 * \param[in] file_obj_info If provided (!=nullptr) the list of "object
	 * info" strings stored in the file will be returned.
	 * \return false on any error writing the file. To obtain more details on
	 * the error you can call getSavePLYErrorString()
	 */
	bool saveToPlyFile(
		const std::string& filename, bool save_in_binary = false,
		const std::vector<std::string>& file_comments =
			std::vector<std::string>(),
		const std::vector<std::string>& file_obj_info =
			std::vector<std::string>()) const;

	/** Return a description of the error if loadFromPlyFile() returned false,
	 * or an empty string if the file was loaded without problems. */
	std::string getSavePLYErrorString() const
	{
		return m_ply_export_last_error;
	}

   protected:
	/** @name PLY Export virtual methods to implement in base classes
		@{ */

	/** In a base class, return the number of vertices */
	virtual size_t PLY_export_get_vertex_count() const = 0;

	/** In a base class, return the number of faces */
	virtual size_t PLY_export_get_face_count() const = 0;

	/** In a base class, will be called after PLY_export_get_vertex_count() once
	 * for each exported point.
	 *  \param pt_color Will be nullptr if the loaded file does not provide
	 * color info.
	 */
	virtual void PLY_export_get_vertex(
		const size_t idx, mrpt::math::TPoint3Df& pt, bool& pt_has_color,
		mrpt::img::TColorf& pt_color) const = 0;

	/** @} */

   private:
	mutable std::string m_ply_export_last_error;

};  // End of class def.

}  // namespace mrpt::opengl
