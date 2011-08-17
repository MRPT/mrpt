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
#ifndef CDynamicGrid_H
#define CDynamicGrid_H

#include <mrpt/utils/utils_defs.h>

namespace mrpt
{
    namespace utils
    {
        using namespace mrpt::system;

        /** A 2D grid of dynamic size which stores any kind of data at each cell.
	  * \tparam T The type of each cell in the 2D grid.
	  * \ingroup mrpt_base_grp
          */
        template <class T>
        class CDynamicGrid
        {
        protected:
            /** The cells.
              */
            std::vector<T>		m_map;

            /** Used only from logically const method that really need to modify the object */
            inline std::vector<T> &	m_map_castaway_const() const  { return const_cast< std::vector<T>& >( m_map ); }

            float			m_x_min,m_x_max,m_y_min,m_y_max;
            float			m_resolution;
            size_t			m_size_x, m_size_y;

        public:
            /** Constructor
              */
            CDynamicGrid(	float	x_min = -10.0f,
                            float	x_max = 10.0f,
                            float	y_min = -10.0f,
                            float	y_max = 10.0f,
                            float	resolution  = 0.10f) :
                                m_map(),
                                m_x_min(),m_x_max(),m_y_min(),m_y_max(),
                                m_resolution(),
                                m_size_x(), m_size_y()
            {
                setSize(x_min,x_max,y_min,y_max,resolution);
            }

            /** Destructor
              */
            virtual ~CDynamicGrid()
            {
            }

            /** Changes the size of the grid, ERASING all previous contents.
              * \sa resize
              */
            void  setSize(	float	x_min,
                                        float	x_max,
                                        float	y_min,
                                        float	y_max,
                                        float	resolution )
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
                m_map.resize(m_size_x*m_size_y);
            }

            /** Erase the contents of all the cells.
              */
            void  clear()
            {
                m_map.clear();
                m_map.resize(m_size_x*m_size_y);
            }

            /** Fills all the cells with the same value
              */
            inline void fill( const T& value ) { std::fill(m_map.begin(),m_map.end(),value); }

            /** Changes the size of the grid, maintaining previous contents.
              * \sa setSize
              */
            virtual void  resize(
                float	new_x_min,
                float	new_x_max,
                float	new_y_min,
                float	new_y_max,
                const T& defaultValueNewCells,
                float	additionalMarginMeters = 2.0f )
            {
                MRPT_START

				MRPT_CHECK_NORMAL_NUMBER(new_x_min)
				MRPT_CHECK_NORMAL_NUMBER(new_x_max)
				MRPT_CHECK_NORMAL_NUMBER(new_y_min)
				MRPT_CHECK_NORMAL_NUMBER(new_y_max)

                unsigned int				x,y;
                unsigned int				extra_x_izq,extra_y_arr,new_size_x=0,new_size_y=0;
                typename std::vector<T>				new_map;
                typename std::vector<T>::iterator	itSrc,itDst;

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
                extra_x_izq = round((m_x_min-new_x_min) / m_resolution);
                extra_y_arr = round((m_y_min-new_y_min) / m_resolution);

                new_size_x = round((new_x_max-new_x_min) / m_resolution);
                new_size_y = round((new_y_max-new_y_min) / m_resolution);

                // Reserve new memory:
                new_map.resize(new_size_x*new_size_y,defaultValueNewCells);

                // Copy previous rows:
                for (y=0;y<m_size_y;y++)
                {
                    for (x=0,itSrc=(m_map.begin()+y*m_size_x),itDst=(new_map.begin()+extra_x_izq + (y+extra_y_arr)*new_size_x);
                            x<m_size_x;
                        x++,itSrc++,itDst++)
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

                MRPT_END

            }

            /** Returns a pointer to the contents of a cell given by its coordinates, or NULL if it is out of the map extensions.
              */
            inline T*	cellByPos( float x, float y )
            {
                int cx = x2idx(x);
                int cy = y2idx(y);

                if( cx<0 || cx>=static_cast<int>(m_size_x) ) return NULL;
                if( cy<0 || cy>=static_cast<int>(m_size_y) ) return NULL;

                return &m_map[ cx + cy*m_size_x ];
            }

            /** Returns a pointer to the contents of a cell given by its coordinates, or NULL if it is out of the map extensions.
              */
            inline const T*	cellByPos( float x, float y ) const
            {
                int cx = x2idx(x);
                int cy = y2idx(y);

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

            /** Returns the horizontal size of grid map in cells count.
                */
            inline size_t getSizeX() const { return m_size_x; }

            /** Returns the vertical size of grid map in cells count.
                */
            inline size_t getSizeY() const { return m_size_y; }

            /** Returns the "x" coordinate of left side of grid map.
                */
            inline float  getXMin()const  { return m_x_min; }

            /** Returns the "x" coordinate of right side of grid map.
                */
            inline float  getXMax()const  { return m_x_max; }

            /** Returns the "y" coordinate of top side of grid map.
                */
            inline float  getYMin()const  { return m_y_min; }

            /** Returns the "y" coordinate of bottom side of grid map.
                */
            inline float  getYMax()const  { return m_y_max; }

            /** Returns the resolution of the grid map.
                */
            inline float  getResolution()const  { return m_resolution; }

            /** The user must implement this in order to provide "saveToTextFile" a way to convert each cell into a numeric value */
            virtual float cell2float(const T& c) const
            {
                return 0;
            }

            void  saveToTextFile(const std::string &fileName) const
            {
                FILE	*f=os::fopen(fileName.c_str(),"wt");
                if(!f) return;
                for (unsigned int cy=0;cy<m_size_y;cy++)
                {
                    for (unsigned int cx=0;cx<m_size_x;cx++)
                        os::fprintf(f,"%f ",cell2float(m_map[ cx + cy*m_size_x ]));
                    os::fprintf(f,"\n");
                }
                os::fclose(f);
            }

            /** Transform a coordinate values into cell indexes.
              */
            inline int   x2idx(float x) const { return static_cast<int>( (x-m_x_min)/m_resolution ); }
            inline int   y2idx(float y) const { return static_cast<int>( (y-m_y_min)/m_resolution ); }
            inline int   xy2idx(float x,float y) const { return x2idx(x) + y2idx(y)*m_size_x; }

            /** Transform a global (linear) cell index value into its corresponding (x,y) cell indexes. */
            inline void  idx2cxcy(const int &idx,  int &cx, int &cy ) const
            {
                cx = idx % m_size_x;
                cy = idx / m_size_x;
            }

            /** Transform a cell index into a coordinate value.
              */
            inline float   idx2x(int cx) const { return m_x_min+(cx+0.5f)*m_resolution; }
            inline float   idx2y(int cy) const { return m_y_min+(cy+0.5f)*m_resolution; }

            /** Transform a coordinate value into a cell index, using a diferent "x_min" value
                */
            inline int   x2idx(float x,float x_min) const { return static_cast<int>((x-m_x_min)/m_resolution ); }
            inline int   y2idx(float y, float y_min) const { return static_cast<int>((y-m_y_min)/m_resolution ); }

			/** Get the entire grid as a matrix. 
			  *  \tparam MAT The type of the matrix, typically a CMatrixDouble.
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

        };

	} // End of namespace
} // end of namespace
#endif
