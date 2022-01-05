/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
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
 * Note that unique values are required for both KEYS and VALUES, hence this
 * class is designed to work with **bijective** mappings only. An exception
 * will be thrown if this contract is broken.
 *
 * \note Defined in `#include <mrpt/containers/bimap.h>`
 *
 * \note This class can be accessed through iterators to the map KEY->VALUE
 * only.
 * \note Both typenames KEY and VALUE must be suitable for being employed as
 * keys in a std::map, i.e. they must be comparable through a "< operator".
 *
 * \note To serialize this class with the mrpt::serialization API, include the
 * header `#include <mrpt/serialization/bimap_serialization.h>` (New in
 * MRPT 2.3.3)
 *
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

	const_iterator begin() const { return m_k2v.begin(); }
	iterator begin() { return m_k2v.begin(); }
	const_iterator end() const { return m_k2v.end(); }
	iterator end() { return m_k2v.end(); }
	const_iterator_inverse inverse_begin() const { return m_v2k.begin(); }
	iterator_inverse inverse_begin() { return m_v2k.begin(); }
	const_iterator_inverse inverse_end() const { return m_v2k.end(); }
	iterator_inverse inverse_end() { return m_v2k.end(); }
	size_t size() const { return m_k2v.size(); }
	bool empty() const { return m_k2v.empty(); }

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

	/** Insert a new pair KEY<->VALUE in the bi-map
	 *  It is legal to insert the same pair (key,value) more than once, but
	 *  if a duplicated key is attempted to be inserted with a different value
	 * (or viceversa) an exception will be thrown. Remember: this class
	 * represents a  **bijective** mapping.
	 */
	void insert(const KEY& k, const VALUE& v)
	{
		const auto itKey = m_k2v.find(k);
		const auto itValue = m_v2k.find(v);
		const bool keyExists = itKey != m_k2v.end(),
				   valueExists = itValue != m_v2k.end();

		if (keyExists && !valueExists)
			THROW_EXCEPTION("Duplicated `key` with different `value`");

		if (!keyExists && valueExists)
			THROW_EXCEPTION("Duplicated `value` with different `key`");

		if (keyExists && valueExists && itKey->second == v) return;	 // Ok

		// New:
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
	bool hasKey(const KEY& k) const { return m_k2v.find(k) != m_k2v.end(); }
	/** Return true if the given value 'v' is in the bi-map \sa hasKey, direct,
	 * inverse */
	bool hasValue(const VALUE& v) const { return m_v2k.find(v) != m_v2k.end(); }

	/** Get the value associated the given key, KEY->VALUE, raising an
	 * exception if not present (equivalent to `directMap.at()`).
	 *
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
	 * not present (equivalent to `inverseMap.at()`).
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

	const_iterator find_key(const KEY& k) const { return m_k2v.find(k); }
	iterator find_key(const KEY& k) { return m_k2v.find(k); }
	const_iterator_inverse find_value(const VALUE& v) const
	{
		return m_v2k.find(v);
	}
	iterator_inverse find_value(const VALUE& v) { return m_v2k.find(v); }

	/** Removes the bijective application between `KEY<->VALUE` for a given key.
	 *  \exception std::exception If the key does not exist.
	 *  \note (New in MRPT 2.3.3);
	 */
	void erase_by_key(const KEY& k)
	{
		auto i = m_k2v.find(k);
		if (i == m_k2v.end()) THROW_EXCEPTION("Key not found.");
		m_v2k.erase(i->second);
		m_k2v.erase(i);
	}
	/** Removes the bijective application between `KEY<->VALUE` for a given
	 * value.
	 * \exception std::exception If the value does not exist.
	 * \note (New in MRPT 2.3.3);
	 */
	void erase_by_value(const VALUE& v)
	{
		auto i = m_v2k.find(v);
		if (i == m_v2k.end()) THROW_EXCEPTION("Value not found.");
		m_k2v.erase(i->second);
		m_v2k.erase(i);
	}

};	// end class bimap

template <typename KEY, typename VALUE>
bool operator==(const bimap<KEY, VALUE>& bm1, const bimap<KEY, VALUE>& bm2)
{
	return bm1.getDirectMap() == bm2.getDirectMap();
}

template <typename KEY, typename VALUE>
bool operator!=(const bimap<KEY, VALUE>& bm1, const bimap<KEY, VALUE>& bm2)
{
	return !(bm1 == bm2);
}

}  // namespace mrpt::containers
