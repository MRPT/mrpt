/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  mrpt_map_as_vector_H
#define  mrpt_map_as_vector_H

#include <mrpt/utils/aligned_containers.h>
#include <map>
#include <vector>

namespace mrpt
{
	namespace utils
	{
		/** A STL-like container which looks and behaves (almost exactly) like a std::map<> but is implemented as a linear std::vector<> indexed by KEY.
		  *  Note that KEY must be integer types only (size_t, uint32_t, etc.)
		  *  This implementation is much more efficient than std::map<> when the most common operation is accesing elements
		  *   by KEY with find() or [], and the range of KEY values starts at 0 (or a reasonable low number).
		  *
		  * This container is internally implemented as a linear array (std::vector) of the same fundamental type than the equivalent std::map<K,V>,
		  *  that is, elements are <code> std::pair<K,V> </code> (note that K is NOT const as in std::map).
		  * I know, I know... this implementation wastes a lot of useless key elements in the pair.first when indices
		  * are already implicit in the std::vector<> order... but I promise I'll pay a beer to whoever show me an
		  *  *efficient* alternative. If failed, update this comment: COUNTER OF WASTED HOURS WITH THIS: 3h
		  *
		  * Note that there is one <b>fundamental difference</b> wrt std::map<>: if you start with an empty map_as_vector<> and
		  *   insert one element at the i'th position (with either [] or insert), the elements [0,i-1] will also exist then, but both
		  *   their first & second entries (for the corresponding std::pair) will be <b>undefined</b>. This was intentional in order to
		  *   gain efficiency (in particular, each std::pair<> doesn't have a constructor when resizing the underlying std::vector).
		  *
		  * The default underlying non-associative container is a "memory-aligned std::vector<>", but it can be changed to a
		  *  standard vector<> or to a deque<> (to avoid memory reallocations) by changing the template parameter \a VECTOR_T.
		  *
		  * \note Defined in #include <mrpt/utils/map_as_vector.h>
		  * \ingroup stlext_grp
		  */
		template <
			typename KEY,
			typename VALUE,
			typename VECTOR_T = typename mrpt::aligned_containers<std::pair<KEY,VALUE> >::vector_t
			>
		class map_as_vector
		{
		public:
			/** @name Iterators stuff and other types
			    @{ */
			typedef KEY                                     key_type;
			typedef std::pair<KEY,VALUE>                    value_type;
			typedef VECTOR_T                                vec_t;
			typedef typename vec_t::size_type               size_type;
			typedef typename vec_t::iterator                iterator;
			typedef typename vec_t::const_iterator          const_iterator;
			typedef std::reverse_iterator<iterator> 		reverse_iterator;
			typedef std::reverse_iterator<const_iterator> 	const_reverse_iterator;

			inline iterator 		begin()   { return m_vec.begin(); }
			inline iterator 		end()     { return m_vec.end(); }
			inline const_iterator 	begin() const	{ return m_vec.begin(); }
			inline const_iterator 	end() const		{ return m_vec.end(); }
			inline reverse_iterator 		rbegin() 		{ return reverse_iterator(end()); }
			inline const_reverse_iterator 	rbegin() const 	{ return const_reverse_iterator(end()); }
			inline reverse_iterator 		rend() 			{ return reverse_iterator(begin()); }
			inline const_reverse_iterator 	rend() const 	{ return const_reverse_iterator(begin()); }
			/** @} */
		private:
			vec_t  m_vec; //!< The actual container

		public:
			/** @name Constructors, read/write access and other operations
			    @{ */
			//!< Default constructor - does nothing */
			inline map_as_vector() { }
			/** Copy constructor */
			inline map_as_vector(const map_as_vector<KEY,VALUE> &o) : m_vec(o.m_vec) { }

			inline size_t size() const { return m_vec.size(); }
			inline bool empty() const { return m_vec.empty(); }

			/** Count how many entries have a given key value - unlike std::map<K,V>, recall that this class will say an element i<N-1 exists just due to an insertion of element at N */
			inline size_type count ( const key_type i ) const { return (i<m_vec.size()) ? 1 : 0; }

			/** Maximum size due to system limits */
			inline size_type max_size() const { return m_vec.max_size(); }

			/** Return a read-only reference to the internal vector */
			inline const vec_t &getVector() const { return m_vec; }

			/** Clear the contents of this container */
			inline void clear() { m_vec.clear(); }

			/** Efficient swap with another object */
			inline void swap(map_as_vector<KEY,VALUE>& o) { m_vec.swap(o.m_vec); }

			/** Write/read via [i] operator, that creates all elements up to (and including) the i'th if they didn't exist already. */
			inline VALUE & operator[](const size_t i) {
				if (m_vec.size()<=i) m_vec.resize(i+1);
				m_vec[i].first=i;
				return m_vec[i].second;
			}

			/** Insert pair<key,val>, as in std::map (guess_point is actually ignored in this class) */
			inline void insert(const iterator &guess_point, const value_type &keyvalpair ) { this->operator[](keyvalpair.first)=keyvalpair; }
			/** Insert pair<key,val>, as in std::map */
			inline void insert(const value_type &keyvalpair ) { this->operator[](keyvalpair.first)=keyvalpair; }

			/** Constant-time find, returning an iterator to the <key,val> pair or to end() if not found (that is, if it's above the maximum index in the vector) */
			inline iterator       find(const size_t i)       { if (i<m_vec.size()) return m_vec.begin()+i; else return m_vec.end(); }
			/** Constant-time find, returning an iterator to the <key,val> pair or to end() if not found (that is, if it's above the maximum index in the vector) */
			inline const_iterator find(const size_t i) const { if (i<m_vec.size()) return m_vec.begin()+i; else return m_vec.end(); }

			/** @} */


		};  // end class map_as_vector

	} // End of namespace
} // End of namespace
#endif
