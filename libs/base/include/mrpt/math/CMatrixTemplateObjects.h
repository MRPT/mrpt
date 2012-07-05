/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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
#ifndef CMatrixTemplateObjects_H
#define CMatrixTemplateObjects_H

#include <mrpt/math/CMatrixTemplate.h>

namespace mrpt
{
namespace math
{
/**  This template class extends the class "CMatrixTemplate" for storing "objects" at each matrix entry.
 *    This class allows a very efficient representation of sparse matrixes where each cell is an arbitrary C++ class, 
 *    but its use must carefully observe the following rules:
 *			- The type in the template especialization MUST be a class with a proper default constructor.
 *			- Initially all entries are set to NULL pointers.
 *			- Pointers can be manually asigned, or automatically created through a call to "CMatrixTemplateObjects<T>::allocAllObjects"
 *			- Independently of how pointers are asigned, memory will be free by destroying objects for each non-NULL entry in the matrix. In some special situations, the user can indicate not to free those objects by calling "CMatrixTemplateObjects<T>::setDestroyBehavior", then it is up to the user to free the memory. In all cases the default behavior is to free the memory.
 *			- Asignament operator with matrixes will COPY THE POINTERS, thus a copy of objects is not performed.
 *			- WARNING: Objects are not deleted while shrinking the matrix by calling "setSize", thus please call ""CMatrixTemplateObjects<T>::freeAllObjects" or manually delete objects before shrinking.
 *
 * \ingroup mrpt_base_grp
 * \sa CMatrixTemplate
 */
template <class T>
class CMatrixTemplateObjects : public CMatrixTemplate<T*>
{
private:
	bool	m_freeObjects;

public:
	/** Copy constructor
	*/
	CMatrixTemplateObjects(const CMatrixTemplate<T>& m) : CMatrixTemplate<T*>( m ), m_freeObjects(true)
	{
	}

	/** Constructor
	*/
	CMatrixTemplateObjects(size_t row = 3, size_t col = 3) :  CMatrixTemplate<T*>( row, col ), m_freeObjects(true)
	{
		for (size_t i=0; i < CMatrixTemplate<T*>::getRowCount(); i++)
			for (size_t j=0; j < CMatrixTemplate<T*>::getColCount(); j++)
				CMatrixTemplate<T*>::m_Val[i][j] = NULL;
	}

	/** Changes the size of matrix
	*/
	virtual void setSize(size_t row, size_t col)
	{
		//TODO: BUGFIX. Doesn't remove objetcs if row<m_Row or col<m_Col
        CMatrixTemplate<T*>::realloc(row,col,true);
	}

	/** Destructor
	  */
	virtual ~CMatrixTemplateObjects()
	{
		if (m_freeObjects)
			freeAllObjects();
	}

	/** Delete all the objects in the matrix and set all entries to NULL pointers.
	  */
	void  freeAllObjects()
	{
		for (size_t i=0; i < CMatrixTemplate<T*>::getRowCount(); i++)
			for (size_t j=0; j < CMatrixTemplate<T*>::getColCount(); j++)
				if (CMatrixTemplate<T*>::m_Val[i][j]!=NULL)
				{
					delete CMatrixTemplate<T*>::m_Val[i][j];
					CMatrixTemplate<T*>::m_Val[i][j] = NULL;
				}
	}

	/** Assignment operator
	*/
	CMatrixTemplateObjects& operator = (const CMatrixTemplateObjects& m)
	{
		CMatrixTemplate<T*>::realloc( m.getRowCount(), m.getColCount() );

		for (size_t i=0; i < CMatrixTemplate<T*>::getRowCount(); i++)
			for (size_t j=0; j < CMatrixTemplate<T*>::getColCount(); j++)
				CMatrixTemplate<T*>::m_Val[i][j] = m.m_Val[i][j];
		return *this;
	}

	/** Sets the behavior on matrix destroy.
	  * See the general description of the class on the top.
	  */
	void  setDestroyBehavior(bool freeObjects = true)
	{
		m_freeObjects = freeObjects;
	}

	/** Alloc memory for all the non-NULL entries in the matrix.
	  * See the general description of the class on the top.
	  */
	void  allocAllObjects()
	{
		for (size_t i=0; i < CMatrixTemplate<T*>::getRowCount(); i++)
			for (size_t j=0; j < CMatrixTemplate<T*>::getColCount(); j++)
				if (NULL==CMatrixTemplate<T*>::m_Val[i][j])
					CMatrixTemplate<T*>::m_Val[i][j] = new T();
	}

}; // end of class definition


	} // End of namespace
} // End of namespace

#endif
