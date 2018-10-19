/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/exceptions.h>
#include <map>

namespace mrpt::containers
{
/** A bidirectional version of std::map, declared as bimap<KEY,VALUE> and which
 * actually contains two std::map's, one for keys and another for values.
 * To use this class, insert new pairs KEY<->VALUE with bimap::insert. Then,
 * you can access the KEY->VALUE map with bimap::direct(), and the VALUE->KEY
 * map with bimap::inverse(). The consistency of the two internal maps is
 * assured at any time.
 *
 * \note This class can be accessed through iterators to the map KEY->VALUE
 * only.
 * \note Both typenames KEY and VALUE must be suitable for being employed as
 * keys in a std::map, i.e. they must be comparable through a "< operator".
 * \note Defined in #include <mrpt/containers/bimap.h>
 * \ingroup mrpt_containers_grp
 */
template <typename KEY, typename VALUE>
class bimap
{
   private:
	std::map<KEY, VALUE> m_k2v;
	std::map<VALUE, KEY> m_v2k;

   public:
	using const_iterator = typename std::map<KEY, VALUE>::const_iterator;
	using iterator = typename std::map<KEY, VALUE>::iterator;
	using const_iterator_inverse =
		typename std::map<VALUE, KEY>::const_iterator;
	using iterator_inverse = typename std::map<VALUE, KEY>::iterator;

	/** Default constructor - does nothing */
	bimap() = default;
	inline const_iterator begin() const { return m_k2v.begin(); }
	inline iterator begin() { return m_k2v.begin(); }
	inline const_iterator end() const { return m_k2v.end(); }
	inline iterator end() { return m_k2v.end(); }
	inline const_iterator_inverse inverse_begin() const
	{
		return m_v2k.begin();
	}
	inline iterator_inverse inverse_begin() { return m_v2k.begin(); }
	inline const_iterator_inverse inverse_end() const { return m_v2k.end(); }
	inline iterator_inverse inverse_end() { return m_v2k.end(); }
	inline size_t size() const { return m_k2v.size(); }
	inline bool empty() const { return m_k2v.empty(); }
	/** Return a read-only reference to the internal map KEY->VALUES */
	const std::map<KEY, VALUE>& getDirectMap() const { return m_k2v; }
	/** Return a read-only reference to the internal map KEY->VALUES */
	const std::map<VALUE, KEY>& getInverseMap() const { return m_v2k; }
	/** Clear the contents of the bi-map. */
	void clear()
	{
		m_k2v.clear();
		m_v2k.clear();
	}

	/** Insert a new pair KEY<->VALUE in the bi-map */
	void insert(const KEY& k, const VALUE& v)
	{
		m_k2v[k] = v;
		m_v2k[v] = k;
	}

	/**  Get the value associated the given key, KEY->VALUE, returning false if
	 * not present.
	 *  \sa inverse, hasKey, hasValue
	 * \return false on key not found.
	 */
	bool direct(const KEY& k, VALUE& out_v) const
	{
		const_iterator i = m_k2v.find(k);
		if (i == m_k2v.end()) return false;
		out_v = i->second;
		return true;
	}

	/** Return true if the given key 'k' is in the bi-map  \sa hasValue, direct,
	 * inverse */
	inline bool hasKey(const KEY& k) const
	{
		return m_k2v.find(k) != m_k2v.end();
	}
	/** Return true if the given value 'v' is in the bi-map \sa hasKey, direct,
	 * inverse */
	inline bool hasValue(const VALUE& v) const
	{
		return m_v2k.find(v) != m_v2k.end();
	}

	/**  Get the value associated the given key, KEY->VALUE, raising an
	 * exception if not present.
	 *  \sa inverse, hasKey, hasValue
	 * \exception std::exception On key not present in the bi-map.
	 */
	VALUE direct(const KEY& k) const
	{
		auto i = m_k2v.find(k);
		if (i == m_k2v.end()) THROW_EXCEPTION("Key not found.");
		return i->second;
	}

	/**  Get the key associated the given value, VALUE->KEY, returning false if
	 * not present.
	 *  \sa direct, hasKey, hasValue
	 * \return false on value not found.
	 */
	bool inverse(const VALUE& v, KEY& out_k) const
	{
		const_iterator_inverse i = m_v2k.find(v);
		if (i == m_v2k.end()) return false;
		out_k = i->second;
		return true;
	}

	/**  Get the key associated the given value, VALUE->KEY, raising an
	 * exception if not present.
	 *  \sa direct, hasKey, hasValue
	 * \return false on value not found.
	 */
	KEY inverse(const VALUE& v) const
	{
		auto i = m_v2k.find(v);
		if (i == m_v2k.end()) THROW_EXCEPTION("Value not found.");
		return i->second;
	}

	inline const_iterator find_key(const KEY& k) const { return m_k2v.find(k); }
	inline iterator find_key(const KEY& k) { return m_k2v.find(k); }
	inline const_iterator_inverse find_value(const VALUE& v) const
	{
		return m_v2k.find(v);
	}
	inline iterator_inverse find_value(const VALUE& v) { return m_v2k.find(v); }
};  // end class bimap

}  // namespace mrpt::containers
