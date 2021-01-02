/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/CommentPosition.h>

#include <string>

namespace mrpt::containers
{
/** Storage for value-comment pairs, with the comment associated to the value.
 * \ingroup mrpt_containers_yaml
 * \sa mrpt::containers::yaml, vcp, vkcp
 * \note [New in MRPT 2.1.0]
 */
template <typename T>
struct ValueCommentPair
{
	ValueCommentPair(const T& v, const std::string& c, CommentPosition pos)
		: value(v), comment(c), position(pos)
	{
	}
	const T& value;
	const std::string& comment;
	const CommentPosition position;
};

/** Helper syntax sugar for ValueCommentPair
 * \ingroup mrpt_containers_yaml
 * \sa mrpt::containers::yaml, vkcp
 * \note [New in MRPT 2.1.0]
 */
template <typename T>
struct ValueCommentPair<T> vcp(
	const T& v, const std::string& c,
	const CommentPosition pos = CommentPosition::RIGHT)
{
	return {v, c, pos};
}

/** Storage for value-comment pairs, with the comment associated to the key, not
 * the value.
 * See examples in mrpt::containers::yaml.
 * \ingroup mrpt_containers_yaml
 * \sa mrpt::containers::yaml, vkcp
 * \note [New in MRPT 2.1.0]
 */
template <typename T>
struct ValueKeyCommentPair
{
	ValueKeyCommentPair(
		const std::string& keyName, const T& v, const std::string& c,
		CommentPosition pos)
		: keyname(keyName), value(v), comment(c), position(pos)
	{
	}
	const std::string& keyname;
	const T& value;
	const std::string& comment;
	const CommentPosition position;
};

/** Helper syntax sugar for ValueKeyCommentPair
 * \ingroup mrpt_containers_yaml
 * \sa mrpt::containers::yaml, vcp
 * \note [New in MRPT 2.1.0]
 */
template <typename T>
struct ValueKeyCommentPair<T> vkcp(
	const std::string& keyName, const T& v, const std::string& c,
	const CommentPosition pos = CommentPosition::TOP)
{
	return {keyName, v, c, pos};
}

}  // namespace mrpt::containers
