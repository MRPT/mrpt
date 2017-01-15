/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CDynamicGrid_H
#define CDynamicGrid_H

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
		namespace internal {
			// Aux class.
			struct BASE_IMPEXP dynamic_grid_txt_saver
			{
				bool saveToTextFile(const std::string &fileName) const;
				virtual unsigned int getSizeX() const = 0;
				virtual unsigned int getSizeY() const = 0;
				virtual float getCellAsFloat(unsigned int cx,unsigned int cy) const = 0;
			};
			} // internal

		/** A 2D grid of dynamic size which stores any kind of data at each cell.
			* \tparam T The type of each cell in the 2D grid.
			* \ingroup mrpt_base_grp
			*/
		template <class T>
		class CDynamicGrid
		{
		protected:
			std::vector<T> m_map; //!< The cells 
			/** Used only from logically const method that really need to modify the object */
			inline std::vector<T> &	m_map_castaway_const() const  { return const_cast< std::vector<T>& >( m_map ); }

			double m_x_min,m_x_max,m_y_min,m_y_max,m_resolution;
			size_t m_size_x, m_size_y;

		public:
			/** Constructor */
			CDynamicGrid(double x_min = -10.0, double x_max = 10.0, double y_min = -10.0f, double y_max = 10.0f, double resolution = 0.10f) :
				m_map(), m_x_min(),m_x_max(),m_y_min(),m_y_max(),
				m_resolution(),m_size_x(), m_size_y()
			{
				setSize(x_min,x_max,y_min,y_max,resolution);
			}

			/** Destructor */
			virtual ~CDynamicGrid() { }

			/** Changes the size of the grid, ERASING all previous contents.
			  * If \a fill_value is left as NULL, the contents of cells may be undefined (some will remain with
			  *  their old values, the new ones will have the default cell value, but the location of old values
			  *  may change wrt their old places).
			  * If \a fill_value is not NULL, it is assured that all cells will have a copy of that value after resizing.
			  * \sa resize, fill
			  */
			void  setSize(
				const double x_min, const double x_max,
				const double y_min, const double y_max,
				const double resolution, const T * fill_value = NULL)
			{
				// Adjust sizes to adapt them to full sized cells acording to the resolution:
				m_x_min = resolution*round(x_min/resolution);
				m_y_min = resolution*round(y_min/resolution);
				m_x_max = resolution*round(x_max/resolution);
				m_y_max = resolution*round(y_max/resolution);

				// Res:
				m_resolution = resolution;

				// Now the number of cells should be integers:
				m_size_x = round((m_x_max-m_x_min)/m_resolution);
				m_size_y = round((m_y_max-m_y_min)/m_resolution);

				// Cells memory:
				if (fill_value)
					m_map.assign(m_size_x*m_size_y, *fill_value);
				else m_map.resize(m_size_x*m_size_y);
			}

			/** Erase the contents of all the cells. */
			void  clear() {
				m_map.clear();
				m_map.resize(m_size_x*m_size_y);
			}

			/** Fills all the cells with the same value
				*/
			inline void fill( const T& value ) {
				for (typename std::vector<T>::iterator it=m_map.begin();it!=m_map.end();++it)
						*it=value;
			}

			/** Changes the size of the grid, maintaining previous contents.
				* \sa setSize
				*/
			virtual void  resize(
				double new_x_min, double new_x_max,
				double new_y_min, double new_y_max,
				const T& defaultValueNewCells, double additionalMarginMeters = 2.0 )
			{
				// Is resize really necesary?
				if (new_x_min>=m_x_min &&
					new_y_min>=m_y_min &&
					new_x_max<=m_x_max &&
					new_y_max<=m_y_max)	return;

				if (new_x_min>m_x_min) new_x_min=m_x_min;
				if (new_x_max<m_x_max) new_x_max=m_x_max;
				if (new_y_min>m_y_min) new_y_min=m_y_min;
				if (new_y_max<m_y_max) new_y_max=m_y_max;

				// Additional margin:
				if (additionalMarginMeters>0)
				{
					if (new_x_min<m_x_min) new_x_min= floor(new_x_min-additionalMarginMeters);
					if (new_x_max>m_x_max) new_x_max= ceil(new_x_max+additionalMarginMeters);
					if (new_y_min<m_y_min) new_y_min= floor(new_y_min-additionalMarginMeters);
					if (new_y_max>m_y_max) new_y_max= ceil(new_y_max+additionalMarginMeters);
				}

				// Adjust sizes to adapt them to full sized cells acording to the resolution:
				if (fabs(new_x_min/m_resolution - round(new_x_min/m_resolution))>0.05f )
					new_x_min = m_resolution*round(new_x_min/m_resolution);
				if (fabs(new_y_min/m_resolution - round(new_y_min/m_resolution))>0.05f )
					new_y_min = m_resolution*round(new_y_min/m_resolution);
				if (fabs(new_x_max/m_resolution - round(new_x_max/m_resolution))>0.05f )
					new_x_max = m_resolution*round(new_x_max/m_resolution);
				if (fabs(new_y_max/m_resolution - round(new_y_max/m_resolution))>0.05f )
					new_y_max = m_resolution*round(new_y_max/m_resolution);

				// Change the map size: Extensions at each side:
				unsigned int extra_x_izq = round((m_x_min-new_x_min) / m_resolution);
				unsigned int extra_y_arr = round((m_y_min-new_y_min) / m_resolution);

				unsigned int new_size_x = round((new_x_max-new_x_min) / m_resolution);
				unsigned int new_size_y = round((new_y_max-new_y_min) / m_resolution);

				// Reserve new memory:
				typename std::vector<T> new_map;
				new_map.resize(new_size_x*new_size_y,defaultValueNewCells);

				// Copy previous rows:
				unsigned int x,y;
				typename std::vector<T>::iterator itSrc,itDst;
				for (y=0;y<m_size_y;y++)
				{
					for (x=0,itSrc=(m_map.begin()+y*m_size_x),itDst=(new_map.begin()+extra_x_izq + (y+extra_y_arr)*new_size_x);
							x<m_size_x;
						++x,++itSrc,++itDst)
					{
						*itDst = *itSrc;
					}
				}

				// Update the new map limits:
				m_x_min = new_x_min;
				m_x_max = new_x_max;
				m_y_min = new_y_min;
				m_y_max = new_y_max;

				m_size_x = new_size_x;
				m_size_y = new_size_y;

				// Keep the new map only:
				m_map.swap(new_map);
			}

			/** Returns a pointer to the contents of a cell given by its coordinates, or NULL if it is out of the map extensions.
				*/
			inline T*	cellByPos( double x, double y )
			{
				const int cx = x2idx(x);
				const int cy = y2idx(y);
				if( cx<0 || cx>=static_cast<int>(m_size_x) ) return NULL;
				if( cy<0 || cy>=static_cast<int>(m_size_y) ) return NULL;
				return &m_map[ cx + cy*m_size_x ];
			}
			/** \overload */
			inline const T*	cellByPos( double x, double y ) const
			{
				const int cx = x2idx(x);
				const int cy = y2idx(y);
				if( cx<0 || cx>=static_cast<int>(m_size_x) ) return NULL;
				if( cy<0 || cy>=static_cast<int>(m_size_y) ) return NULL;
				return &m_map[ cx + cy*m_size_x ];
			}

			/** Returns a pointer to the contents of a cell given by its cell indexes, or NULL if it is out of the map extensions.
				*/
			inline  T*	cellByIndex( unsigned int cx, unsigned int cy )
			{
				if( cx>=m_size_x || cy>=m_size_y)
						return NULL;
				else	return &m_map[ cx + cy*m_size_x ];
			}

			/** Returns a pointer to the contents of a cell given by its cell indexes, or NULL if it is out of the map extensions.
				*/
			inline const T* cellByIndex( unsigned int cx, unsigned int cy ) const
			{
				if( cx>=m_size_x || cy>=m_size_y)
						return NULL;
				else	return &m_map[ cx + cy*m_size_x ];
			}

			/** Returns the horizontal size of grid map in cells count */
			inline size_t getSizeX() const { return m_size_x; }

			/** Returns the vertical size of grid map in cells count */
			inline size_t getSizeY() const { return m_size_y; }

			/** Returns the "x" coordinate of left side of grid map */
			inline double getXMin()const  { return m_x_min; }

			/** Returns the "x" coordinate of right side of grid map */
			inline double getXMax()const  { return m_x_max; }

			/** Returns the "y" coordinate of top side of grid map */
			inline double getYMin()const  { return m_y_min; }

			/** Returns the "y" coordinate of bottom side of grid map */
			inline double getYMax()const  { return m_y_max; }

			/** Returns the resolution of the grid map */
			inline double getResolution()const  { return m_resolution; }

			/** Transform a coordinate values into cell indexes */
			inline int x2idx(double x) const { return static_cast<int>( (x-m_x_min)/m_resolution ); }
			inline int y2idx(double y) const { return static_cast<int>( (y-m_y_min)/m_resolution ); }
			inline int xy2idx(double x,double y) const { return x2idx(x) + y2idx(y)*m_size_x; }

			/** Transform a global (linear) cell index value into its corresponding (x,y) cell indexes. */
			inline void idx2cxcy(const int &idx,  int &cx, int &cy ) const
			{
				cx = idx % m_size_x;
				cy = idx / m_size_x;
			}

			/** Transform a cell index into a coordinate value of the cell central point */
			inline double idx2x(int cx) const { return m_x_min+(cx+0.5)*m_resolution; }
			inline double idx2y(int cy) const { return m_y_min+(cy+0.5)*m_resolution; }

			/** Get the entire grid as a matrix.
				*  \tparam MAT The type of the matrix, typically a mrpt::math::CMatrixDouble.
				*  \param[out] m The output matrix; will be set automatically to the correct size.
				*  Entry (cy,cx) in the matrix contains the grid cell with indices (cx,cy).
				* \note This method will compile only for cell types that can be converted to the type of the matrix elements (e.g. double).
				*/
			template <class MAT>
			void getAsMatrix(MAT &m) const
			{
				m.setSize(m_size_y, m_size_x);
				if (m_map.empty()) return;
				const T* c = &m_map[0];
				for (size_t cy=0;cy<m_size_y;cy++)
					for (size_t cx=0;cx<m_size_x;cx++)
						m.set_unsafe(cy,cx, *c++);
			}

			/** The user must implement this in order to provide "saveToTextFile" a way to convert each cell into a numeric value */
			virtual float cell2float(const T& c) const
			{
				MRPT_UNUSED_PARAM(c);
				return 0;
			}
			/** Saves a float representation of the grid (via "cell2float()") to a text file. \return false on error */
			bool saveToTextFile(const std::string &fileName) const
			{
				struct aux_saver : public internal::dynamic_grid_txt_saver
				{
					aux_saver(const CDynamicGrid<T> &obj) : m_obj(obj) {}
					virtual unsigned int getSizeX() const { return m_obj.getSizeX(); }
					virtual unsigned int getSizeY() const { return m_obj.getSizeY(); }
					virtual float getCellAsFloat(unsigned int cx,unsigned int cy) const { return m_obj.cell2float(m_obj.m_map[ cx + cy*m_obj.getSizeX() ]); }
					const CDynamicGrid<T> & m_obj;
				};
				aux_saver aux(*this);
				return aux.saveToTextFile(fileName);
			}

		protected:
			void  dyngridcommon_writeToStream(mrpt::utils::CStream &out) const {
				out << m_x_min << m_x_max << m_y_min << m_y_max;
				out << m_resolution;
				out << static_cast<uint32_t>(m_size_x) << static_cast<uint32_t>(m_size_y);
			}
			void  dyngridcommon_readFromStream(mrpt::utils::CStream &in, bool cast_from_float = false) {
				if (!cast_from_float) {
					in >> m_x_min >> m_x_max >> m_y_min >> m_y_max;
					in >> m_resolution;
				} else {
					float xmin,xmax,ymin,ymax,res;
					in >> xmin >> xmax >> ymin >> ymax >> res;
					m_x_min = xmin; m_x_max = xmax; m_y_min = ymin; m_y_max = ymax; m_resolution = res;
				}
				uint32_t nX,nY;
				in >> nX >> nY;
				m_size_x = nX; m_size_y = nY;
				m_map.resize(nX*nY);
			}


		}; // end of CDynamicGrid<>

	} // End of namespace
} // end of namespace
#endif
