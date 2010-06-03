/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
#ifndef CVectorTemplate_H
#define CVectorTemplate_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/CArray.h>

namespace mrpt
{
namespace math
{
	/**  This template class provides the basic functionality for a general 1D any-size, resizable container of numerical or non-numerical elements.
	* NOTES:
	*		- First element index is "0".
	*		- This class is not serializable (use std::vector<T> instead if serialization is needed).
	*		- Please DO NOT use any other class as template parameter T. It can be safely used the following types:
	*			- Elemental types (POD: int,char,float,doble,...)
	*			- Data struct (No classes!)
	*			- Any kind of pointers (user is responsible for allocating and freeing the memory addressed by pointers).
	*
	*/

	template <class T>
	class CVectorTemplate : public std::vector<T>
	{
	protected:
		typedef std::vector<T> 		base_class;

	public:
		typedef CVectorTemplate<T> mrpt_autotype;
		DECLARE_MRPT_CONTAINER_TYPES
		DECLARE_MRPT_CONTAINER_IS_VECTOR

	/** Construct the vector with a given initial size
	  */
	CVectorTemplate(size_t creationSize=0) : std::vector<T>(creationSize)
	{
	}

	/** Construct the vector with a given initial size and initial filling value
	  */
	CVectorTemplate(size_t creationSize, const T& initVal) : std::vector<T>(creationSize,initVal)
	{
	}

	/** This function extract a part of a vector. It's necessary initialize the output vector to desired size.
	*
	*/
		void  extract_vector(const size_t &index, CVectorTemplate<T> &out)
		{
			size_t	i;

			if (index + out.size() > base_class::size())
				THROW_EXCEPTION("extract_vector: Index out of bounds");

			for (i=index;i<index+out.size();i++)
                out[i-index] = (*this)[i];
		}
/*****************************************************AJOGD*********************************************************/
		/** This function extract a part of a vector. It's necessary initialize the output vector to desired size.
	*
	*/
		CVectorTemplate<T>  extract_vector(const size_t index, const unsigned int length)
		{
			if (index + length > base_class::size())
				THROW_EXCEPTION("extract_vector: Index out of bounds");

			size_t	i;
			CVectorTemplate<T> out;
			out.resize(length);

			for (i=index;i<index+length;i++)
                out[i-index] = (*this)[i];

			return out;
		}
/*******************************************************************************************************************/
	/** This function insert the vector "in" into in the vector from an index.
	*
	*/
		void  insert_vector(const size_t &index, const CVectorTemplate<T> &in)
		{
			size_t	i;

			if (index + in.size()>base_class::size())
				THROW_EXCEPTION("insert_vector: Index out of bounds");

			for (i=index;i<index+in.size();i++)
                (*this)[i] = in[i-index];
		}


		/** Adds a row or column matrix of the correct length to the vector.
		  */
		void operator +=(const CMatrixTemplateNumeric<T> &M)
		{
			MRPT_START
			if (M.getColCount()!=1 && M.getRowCount()!=1)
				THROW_EXCEPTION("Matrix must be a column or row matrix!");

			if (M.getColCount()==1)
			{
				// Column matrix
				ASSERT_(M.getRowCount()== base_class::size())
				size_t i,N = base_class::size();

				for (i=0;i<N;i++)
					(*this)[i] += M(i,0);
			}
			else
			{
				// Row matrix
				ASSERT_(M.getColCount()==base_class::size())
				size_t i,N = base_class::size();

				for (i=0;i<N;i++)
					(*this)[i] += M(0,i);
			}

			MRPT_END
		}

	/** This function join two vectors in a new vector.
	*
	*/
		void  concatenate(const CVectorTemplate &first, const CVectorTemplate &second)
		{
			size_t i;
			(*this).resize(first.size()+second.size());
			for (i=0;i<first.size();i++)
                (*this)[i] = first[i];
			for (i=0;i<second.size();i++)
				std::vector<T>::at(i+first.size()) = second[i];
		}


	/** This function split a vectors in a two new vector.
	*
	*/
		void  deconcatenate(CVectorTemplate &first, CVectorTemplate &second, const size_t &index)
		{
			if (index>base_class::size())
				THROW_EXCEPTION("Error in CVectorTemplate::deconcatenate. Index out of bounds");

			size_t i;

			first.resize(index);
			second.resize(base_class::size()-index);
			for (i=0;i<index;i++)
                first[i] = (*this)[i];
			for (i=0;i<second.size();i++)
				second[i] = std::vector<T>::at(i+index);
		}

/*************************************************AJOGD****************************************************************/
	/** This function find the maximun value of a vector and return this value and his index
	*
	*/
		void  find_max(size_t &index, T &val)
		{
			if (base_class::size()<1)
				THROW_EXCEPTION("vector without dimensions in CVectorTemplate::find_max");

			val=std::vector<T>::at(0);
			index=0;
			for (size_t i=1; i<base_class::size();i++)
			{
				if (val<(*this)[i])
				{
					val=(*this)[i];
					index = i;
				}
			}
		}
/*************************************************AJOGD****************************************************************/
	/** This function find the minimun value of a vector and return this value and his index
	*
	*/
		void  find_min(size_t &index, T &val)
		{
			if (base_class::size()<1)
				THROW_EXCEPTION("vector without dimensions in CVectorTemplate::find_max");

			val=(*this)[0];
			index=0;
			for (size_t i=1; i<base_class::size();i++)
			{
				if (val>(*this)[i])
				{
					val=(*this)[i];
					index = i;
				}
			}
		}

/*************************************************AJOGD****************************************************************/
	/** This function find the maximun and minimun value of a vector and return this value and his index
	*
	*/
		void  find_min_max(size_t &index_min, size_t &index_max, T &min, T &max)
		{
			if (base_class::size()<1)
				THROW_EXCEPTION("vector without dimensions in CVectorTemplate::find_max");

			min=(*this)[0];
			max=(*this)[0];
			index_min=0;
			index_max=0;
			for (size_t i=1; i<base_class::size();i++)
			{
				if (min>(*this)[i])
				{
					min=(*this)[i];
					index_min = i;
				}
				else if (max<(*this)[i])
				{
					max=(*this)[i];
					index_max = i;
				}
			}
		}
/*************************************************AJOGD****************************************************************/
	/** This function make the vector absolute value for each element
	*
	*/
		void  abs()
		{
			for (size_t i=0; i<base_class::size();i++)
				if ((*this)[i] < 0)
					(*this)[i] *= -1;

		}
/*************************************************AJOGD****************************************************************/
	/** This function is a fast solution to read a vector from text, but isn't optimum
	*
	*/
		void  loadFromTextFile(const std::string &file)
		{
			CMatrixTemplateNumeric<T> aux;
			aux.loadFromTextFile(file);
			unsigned int row = aux.getRowCount();
			unsigned int col = aux.getColCount();
			if ((row!=1)&&(col!=1))
				THROW_EXCEPTION("Error loading vector from text file, isn't a vector");

			if (row==1)	//row vector
			{
				(*this).resize(col);
				for (size_t i=0;i<col;i++)
					(*this)[i] = aux(0,i);
			}
			else	//col vector
			{
				(*this).resize(row);
				for (size_t j=0;j<row;j++)
					(*this)[j] = aux(j,0);
			}
		}

/*************************************************AJOGD****************************************************************/
	/** This function is a fast solution to write a vector into a text file, but isn't optimum
	*
	*/
		void  saveToTextFile(const std::string &file)
		{
			CMatrixTemplateNumeric<T> aux(1,base_class::size());
			for (size_t i=0;i<base_class::size();i++)
				aux(0,i) = (*this)[i];
			aux.saveToTextFile(file);
		}

	/** Calculate the mean value of a vector
	*
	*/
		T  mean()
		{
			T sum=0.0;
			for (size_t i=0;i<base_class::size();i++)
				sum += (*this)[i];
			return sum/base_class::size();
		}

	/** Return the vector like a matrix with dimension [1 , vector-size]
	*
	*/
		CMatrixTemplateNumeric<T> likeMatrix()
		{
			CMatrixTemplateNumeric<T>	out;
			out.setSize(1,base_class::size());
			for( size_t i = 0; i < base_class::size() ; i++)
				out(0,i) = (*this)[i];
			return out;
		}
/********************************************OPERATORS*****************************************************************/
/** A template function for adding std::vector's having the same number of elements.
*/
		CVectorTemplate<T>  operator + (CVectorTemplate &b)
		{
			ASSERT_(this->size()==b.size());

			CVectorTemplate<T>	res;	res.assign(this->size(),0);
			typename std::vector<T>::iterator	it_a,it_b,it_res;

			for (it_a=this->begin(), it_b=b.begin(), it_res=res.begin(); it_a!=this->end(); it_a++, it_b++, it_res++)
				*it_res = *it_a + *it_b;
			return res;
		}


/** A template function for substracting std::vector's having the same number of elements.
*/
		CVectorTemplate<T>  operator - (CVectorTemplate &b)
		{
		ASSERT_(this->size()==b.size());

		CVectorTemplate<T>	res;	res.assign(this->size(),0);
		typename std::vector<T>::iterator	it_a,it_b,it_res;

		for (it_a=this->begin(), it_b=b.begin(), it_res=res.begin(); it_a!=this->end(); it_a++, it_b++, it_res++)
			*it_res = *it_a - *it_b;
		return res;
		}


/** A template function for scalar product (element by element, like .* in MATLAB) of std::vector's having the same number of elements.
*/
		CVectorTemplate<T>  operator * (CVectorTemplate &b)
		{
		ASSERT_(this->size()==b.size());

		CVectorTemplate<T>	res;	res.assign(this->size(),0);
		typename std::vector<T>::iterator	it_a,it_b,it_res;

		for (it_a=this->begin(), it_b=b.begin(), it_res=res.begin(); it_a!=this->end(); it_a++, it_b++, it_res++)
			*it_res = (*it_a) * (*it_b);
		return res;
		}


/** A template function for scalar division (element by element, like ./ in MATLAB) of std::vector's having the same number of elements.
*/
		CVectorTemplate<T>  operator / (CVectorTemplate &b)
		{
		ASSERT_(this->size()==b.size());

		CVectorTemplate<T>	res;	res.assign(this->size(),0);
		typename std::vector<T>::iterator	it_a,it_b,it_res;

		for (it_a=this->begin(), it_b=b.begin(), it_res=res.begin(); it_a!=this->end(); it_a++, it_b++, it_res++)
			*it_res = (*it_a) / (*it_b);
		return res;
		}


/** A template function for adding a scalar to a std::vector.
*/
		CVectorTemplate<T>  operator + (T b)
		{

		CVectorTemplate<T>	res;	res.assign(this->size(),0);
		typename std::vector<T>::iterator	it_a,it_res;

		for (it_a=this->begin(), it_res=res.begin(); it_a!=this->end(); it_a++, it_res++)
			*it_res = (*it_a) + b;
		return res;
		}

/** A template function for subtracting a scalar to a std::vector.
*/
		CVectorTemplate<T>  operator - (T b)
		{

		CVectorTemplate<T>	res;	res.assign(this->size(),0);
		typename std::vector<T>::iterator	it_a,it_res;

		for (it_a=this->begin(), it_res=res.begin(); it_a!=this->end(); it_a++, it_res++)
			*it_res = (*it_a) - b;
		return res;
		}

/** A template function for multiplying a scalar to a std::vector.
*/
		CVectorTemplate<T>  operator * (T b)
		{

		CVectorTemplate<T>	res;	res.assign(this->size(),0);
		typename std::vector<T>::iterator	it_a,it_res;

		for (it_a=this->begin(), it_res=res.begin(); it_a!=this->end(); it_a++, it_res++)
			*it_res = (*it_a) * b;
		return res;
		}

/** A template function for adding a scalar to a std::vector.
*/
		CVectorTemplate<T>  operator / (T b)
		{

		CVectorTemplate<T>	res;	res.assign(this->size(),0);
		typename std::vector<T>::iterator	it_a,it_res;

		for (it_a=this->begin(), it_res=res.begin(); it_a!=this->end(); it_a++, it_res++)
			*it_res = (*it_a) / b;
		return res;
		}

/** unary transpose operator
 */
        CMatrixTemplateNumeric<T>  operator ~ ()
		{
            CMatrixTemplateNumeric<T>	temp(base_class::size(),1);
            for (size_t i=0; i < base_class::size(); i++)
                temp(i,0) = (*this)[i];
            return temp;
		}


		/** Extract a fixed-size array from the vector elements [first_idx,first_idx+N-1] */
		template <size_t N>
		void extract_array(size_t first_idx, CArrayPOD<T,N> &out_array) const
		{
			ASSERT_( first_idx+N <= this->size() );
			::memcpy( out_array.data(), &(*this)[first_idx], sizeof(T)*N );
		}

		/** Insert a fixed-size array into the vector elements [first_idx,first_idx+N-1] */
		template <size_t N>
		void insert_array(size_t first_idx, CArrayPOD<T,N> &in_array)
		{
			ASSERT_( first_idx+N <= this->size() );
			::memcpy( &(*this)[first_idx], in_array.data(), sizeof(T)*N );
		}

	}; // end class


    /** Declares a vector of float elements.
      * \sa CVectorDouble
      */
    typedef CVectorTemplate<float> CVectorFloat;

    /** Declares a vector of double elements.
      * \sa CVectorFloat
      */
    typedef CVectorTemplate<double> CVectorDouble;


	/** A template function for printing out the contents of a std::vector variable.
		*/
	/*template <class T>
	std::ostream& operator << (std::ostream& out, const CVectorTemplate<T> &d)
	{
		out << "[";
		for (typename CVectorTemplate<T>::const_iterator it = d.begin();it!=d.end();++it)
		{
			out << *it;
			if (it!=d.end()-1)
				out << ",";
		}
		out << "]";
		return out;
	}*/

	} // end namespace
} // end namespace

#endif
