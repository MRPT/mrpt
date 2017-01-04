
#pragma once

#include <mrpt/utils/core_defs.h>
#include <mrpt/utils/round.h>
#include <mrpt/utils/CStream.h>
#include <vector>
#include <string>
#define _USE_MATH_DEFINES // (For VS to define M_PI, etc. in cmath)
#include <cmath>

namespace mrpt
{
	namespace utils
	{
		/** A 3D rectangular grid of dynamic size which stores any kind of data at each voxel.
		* \tparam T The type of each voxel in the grid.
		*/
		template <class T>
		class CDynamicGrid3D
		{
		public:
			/** Constructor */
			CDynamicGrid3D(double x_min=-1.0, double x_max=1.0, double y_min=-1.0, double y_max=+1.0, double z_min=-1.0, double z_max=1.0, double resolution_xy=0.5, double resolution_z=0.5) :
				m_map()
			{
				setSize(x_min,x_max,y_min,y_max,z_min, z_max, resolution_xy, resolution_z);
			}

			/** Changes the size of the grid, maintaining previous contents.
			* \sa setSize
			*/
			virtual void  resize(
				double new_x_min, double new_x_max,
				double new_y_min, double new_y_max,
				double new_z_min, double new_z_max,
				const T& defaultValueNewCells, double additionalMarginMeters = 2.0)
			{
				// Is resize really necesary?
				if (new_x_min >= m_x_min &&
					new_y_min >= m_y_min &&
					new_z_min >= m_z_min &&
					new_x_max <= m_x_max &&
					new_y_max <= m_y_max &&
					new_z_max <= m_z_max)
					return;

				if (new_x_min>m_x_min) new_x_min = m_x_min;
				if (new_x_max<m_x_max) new_x_max = m_x_max;
				if (new_y_min>m_y_min) new_y_min = m_y_min;
				if (new_y_max<m_y_max) new_y_max = m_y_max;
				if (new_z_min>m_z_min) new_z_min = m_z_min;
				if (new_z_max<m_z_max) new_z_max = m_z_max;

				// Additional margin:
				if (additionalMarginMeters>0)
				{
					if (new_x_min<m_x_min) new_x_min = floor(new_x_min - additionalMarginMeters);
					if (new_x_max>m_x_max) new_x_max = ceil(new_x_max + additionalMarginMeters);
					if (new_y_min<m_y_min) new_y_min = floor(new_y_min - additionalMarginMeters);
					if (new_y_max>m_y_max) new_y_max = ceil(new_y_max + additionalMarginMeters);
					if (new_z_min<m_z_min) new_z_min = floor(new_z_min - additionalMarginMeters);
					if (new_z_max>m_z_max) new_z_max = ceil(new_z_max + additionalMarginMeters);
				}

				// Adjust sizes to adapt them to full sized cells acording to the resolution:
				if (fabs(new_x_min / m_resolution_xy - round(new_x_min / m_resolution_xy))>0.05)
					new_x_min = m_resolution_xy*round(new_x_min / m_resolution_xy);
				if (fabs(new_y_min / m_resolution_xy - round(new_y_min / m_resolution_xy))>0.05)
					new_y_min = m_resolution_xy*round(new_y_min / m_resolution_xy);
				if (fabs(new_z_min / m_resolution_z - round(new_z_min / m_resolution_z))>0.05)
					new_z_min = m_resolution_z*round(new_z_min / m_resolution_z);
				if (fabs(new_x_max / m_resolution_xy - round(new_x_max / m_resolution_xy))>0.05)
					new_x_max = m_resolution_xy*round(new_x_max / m_resolution_xy);
				if (fabs(new_y_max / m_resolution_xy - round(new_y_max / m_resolution_xy))>0.05)
					new_y_max = m_resolution_xy*round(new_y_max / m_resolution_xy);
				if (fabs(new_z_max / m_resolution_z - round(new_z_max / m_resolution_z))>0.05)
					new_z_max = m_resolution_z*round(new_z_max / m_resolution_z);

				// Change the map size: Extensions at each side:
				size_t extra_x_izq = round((m_x_min - new_x_min) / m_resolution_xy);
				size_t extra_y_arr = round((m_y_min - new_y_min) / m_resolution_xy);
				size_t extra_z_top = round((m_z_min - new_z_min) / m_resolution_z);

				size_t new_size_x = round((new_x_max - new_x_min) / m_resolution_xy);
				size_t new_size_y = round((new_y_max - new_y_min) / m_resolution_xy);
				size_t new_size_z = round((new_z_max - new_z_min) / m_resolution_z);
				size_t new_size_x_times_y = new_size_x*new_size_y;

				// Reserve new memory:
				typename std::vector<T> new_map;
				new_map.resize(new_size_x*new_size_y*new_size_z, defaultValueNewCells);

				// Copy previous rows:
				size_t x, y, z;
				typename std::vector<T>::iterator itSrc, itDst;
				for (z = 0; z < m_size_z; z++)
				{
					for (y = 0; y < m_size_y; y++)
					{
						for (x = 0, itSrc = (m_map.begin() + y*m_size_x+z*m_size_x_times_y), itDst = (new_map.begin() + extra_x_izq + (y + extra_y_arr)*new_size_x + (z+extra_z_top)*new_size_x_times_y );
							x < m_size_x;
							++x, ++itSrc, ++itDst)
						{
							*itDst = *itSrc;
						}
					}
				}

				// Update the new map limits:
				m_x_min = new_x_min;
				m_x_max = new_x_max;
				m_y_min = new_y_min;
				m_y_max = new_y_max;
				m_z_min = new_z_min;
				m_z_max = new_z_max;

				m_size_x = new_size_x;
				m_size_y = new_size_y;
				m_size_z = new_size_z;
				m_size_x_times_y = new_size_x_times_y;

				// Keep the new map only:
				m_map.swap(new_map);
			}

			/** Changes the size of the grid, ERASING all previous contents.
			  * If \a fill_value is left as NULL, the contents of cells may be undefined (some will remain with
			  *  their old values, the new ones will have the default voxel value, but the location of old values
			  *  may change wrt their old places).
			  * If \a fill_value is not NULL, it is assured that all cells will have a copy of that value after resizing.
			  * If `resolution_z`<0, the same resolution will be used for all dimensions x,y,z as given in `resolution_xy`
			  * \sa resize, fill
			  */
			virtual void  setSize(
				const double x_min, const double x_max,
				const double y_min, const double y_max,
				const double z_min, const double z_max,
				const double resolution_xy, const double resolution_z_=-1.0, 
				const T * fill_value = NULL)
			{
				const double resolution_z = resolution_z_ > 0.0 ? resolution_z_ : resolution_xy;

				// Adjust sizes to adapt them to full sized cells acording to the resolution:
				m_x_min = x_min;
				m_y_min = y_min;
				m_z_min = z_min;

				m_x_max = x_min + resolution_xy*round((x_max-x_min) / resolution_xy);
				m_y_max = y_min + resolution_xy*round((y_max-y_min)/ resolution_xy);
				m_z_max = z_min + resolution_z*round((z_max-z_min) / resolution_z);

				// Res:
				m_resolution_xy = resolution_xy;
				m_resolution_z = resolution_z;

				// Now the number of cells should be integers:
				m_size_x = round((m_x_max-m_x_min)/m_resolution_xy);
				m_size_y = round((m_y_max - m_y_min) / m_resolution_xy);
				m_size_x_times_y = m_size_x * m_size_y;
				m_size_z = round((m_z_max-m_z_min)/m_resolution_z);

				// Cells memory:
				if (fill_value)
					m_map.assign(m_size_x*m_size_y*m_size_z, *fill_value);
				else m_map.resize(m_size_x*m_size_y*m_size_z);
			}

			/** Erase the contents of all the cells, setting them to their default values (default ctor). */
			virtual void clear() {
				m_map.clear();
				m_map.resize(m_size_x*m_size_y*m_size_z);
			}

			/** Fills all the cells with the same value
				*/
			inline void fill( const T& value ) {
				for (typename std::vector<T>::iterator it=m_map.begin();it!=m_map.end();++it)
						*it=value;
			}

			static const size_t INVALID_VOXEL_IDX = size_t(-1);

			/** Gets the absolute index of a voxel in the linear container m_map[] from its cx,cy,cz indices, or -1 if out of map bounds (in any dimension). \sa x2idx(), y2idx(), z2idx() */
			inline size_t cellAbsIndexFromCXCYCZ(const int cx, const int cy, const int cz) const {
				if (cx<0 || cx >= static_cast<int>(m_size_x)) return INVALID_VOXEL_IDX;
				if (cy<0 || cy >= static_cast<int>(m_size_y)) return INVALID_VOXEL_IDX;
				if (cz<0 || cy >= static_cast<int>(m_size_z)) return INVALID_VOXEL_IDX;
				return cx + cy*m_size_x + cz * m_size_x_times_y;
			}
				

			/** Returns a pointer to the contents of a voxel given by its coordinates, or NULL if it is out of the map extensions.
				*/
			inline T*	cellByPos( double x, double y, double z )
			{
				const size_t cidx = cellAbsIndexFromCXCYCZ(x2idx(x), y2idx(y), z2idx(z));
				if (cidx == INVALID_VOXEL_IDX) return NULL;
				return &m_map[cidx];
			}
			/** \overload */
			inline const T*	cellByPos( double x, double y, double z ) const
			{
				const size_t cidx = cellAbsIndexFromCXCYCZ(x2idx(x), y2idx(y), z2idx(z));
				if (cidx == INVALID_VOXEL_IDX) return NULL;
				return &m_map[cidx];
			}

			/** Returns a pointer to the contents of a voxel given by its voxel indexes, or NULL if it is out of the map extensions.
				*/
			inline  T*	cellByIndex( unsigned int cx, unsigned int cy, unsigned int cz )
			{
				const size_t cidx = cellAbsIndexFromCXCYCZ(cx, cy, cz);
				if (cidx == INVALID_VOXEL_IDX) return NULL;
				return &m_map[cidx];
			}

			/** Returns a pointer to the contents of a voxel given by its voxel indexes, or NULL if it is out of the map extensions.
				*/
			inline const T* cellByIndex( unsigned int cx, unsigned int cy, unsigned int cz  ) const
			{
				const size_t cidx = cellAbsIndexFromCXCYCZ(cx, cy, cz);
				if (cidx == INVALID_VOXEL_IDX) return NULL;
				return &m_map[cidx];
			}

			inline size_t getSizeX() const { return m_size_x; }
			inline size_t getSizeY() const { return m_size_y; }
			inline size_t getSizeZ() const { return m_size_z; }
			inline size_t getVoxelCount() const { return m_size_x_times_y * m_size_z; }

			inline double getXMin()const  { return m_x_min; }
			inline double getXMax()const  { return m_x_max; }

			inline double getZMin()const  { return m_z_min; }
			inline double getZMax()const  { return m_z_max; }

			inline double getResolutionXY()const { return m_resolution_xy; }
			inline double getResolutionZ()const  { return m_resolution_z; }

			/** Transform a coordinate values into voxel indexes */
			inline int x2idx(double x) const { return static_cast<int>( (x-m_x_min)/m_resolution_xy ); }
			inline int y2idx(double y) const { return static_cast<int>((y - m_y_min) / m_resolution_xy); }
			inline int z2idx(double z) const { return static_cast<int>( (z-m_z_min)/m_resolution_z ); }

			/** Transform a voxel index into a coordinate value of the voxel central point */
			inline double idx2x(int cx) const { return m_x_min+(cx)*m_resolution_xy; }
			inline double idx2y(int cy) const { return m_y_min+(cy)*m_resolution_xy; }
			inline double idx2z(int cz) const { return m_z_min + (cz)*m_resolution_z; }

		protected:
			mutable std::vector<T> m_map; //!< The cells
			/** Used only from logically const method that really need to modify the object */
			inline std::vector<T> & m_map_castaway_const() const { return m_map; }

			double m_x_min, m_x_max, m_y_min, m_y_max, m_z_min, m_z_max, m_resolution_xy, m_resolution_z;
			size_t m_size_x, m_size_y, m_size_z, m_size_x_times_y;
			/** Serialization of all parameters, except the contents of each voxel (responsability of the derived class) */
			void  dyngridcommon_writeToStream(mrpt::utils::CStream &out) const {
				out << m_x_min << m_x_max << m_y_min << m_y_max << m_z_min << m_z_max;
				out << m_resolution_xy << m_resolution_z;
				out << static_cast<uint32_t>(m_size_x) << static_cast<uint32_t>(m_size_y) << static_cast<uint32_t>(m_size_z);
			}
			/** Serialization of all parameters, except the contents of each voxel (responsability of the derived class) */
			void  dyngridcommon_readFromStream(mrpt::utils::CStream &in) {
				in >> m_x_min >> m_x_max >> m_y_min >> m_y_max >> m_z_min >> m_z_max;
				in >> m_resolution_xy >> m_resolution_z;

				uint32_t nX,nY,nZ;
				in >> nX >> nY >> nZ;
				m_size_x = nX; m_size_y = nY; m_size_z = nZ;
				m_map.resize(nX*nY*nZ);
			}

		}; // end of CDynamicGrid3D<>

	} // End of namespace
} // end of namespace

