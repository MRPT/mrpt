// -*- mode:c++;tab-width:2;indent-tabs-mode:t;show-trailing-whitespace:t;rm-trailing-spaces:t -*-
// vi: set ts=2 noet:
//
// Copyright (c) 2016 Sergey Lyskov <sergey.lyskov@jhu.edu>
//
// All rights reserved. Use of this source code is governed by a
// MIT license that can be found in the LICENSE file.

/// @file   binder/std_binders.hpp
/// @brief  Support for custom binders for some std:: template classes
/// @author Sergey Lyskov


#ifndef _INCLUDED_std_binders_hpp_
#define _INCLUDED_std_binders_hpp_

#include <map>
#include <deque>
#include <pybind11/stl_bind.h>

namespace binder {

template <typename T, class Allocator> class vector_binder
{
	using Vector = std::vector<T, Allocator>;
	using SizeType = typename Vector::size_type;

	using Class_ = pybind11::class_<Vector, std::shared_ptr< Vector > >;


public:
	vector_binder(pybind11::module &m, std::string const &name, std::string const & /*allocator name*/)
	{
		using Vector = std::vector<T, Allocator>;
		using holder_type = std::shared_ptr<std::vector<T, Allocator>>;
		using Class_ = pybind11::class_<Vector, holder_type>;

		Class_ cl = pybind11::bind_vector<Vector, holder_type>(m, "vector_" + name);

		// cl.def(pybind11::init<size_type>());
		// cl.def("resize", (void (Vector::*) (size_type count)) & Vector::resize, "changes the number of elements stored");

		cl.def("empty", &Vector::empty, "checks whether the container is empty");
		cl.def("max_size", &Vector::max_size, "returns the maximum possible number of elements");
		cl.def("reserve", &Vector::reserve, "reserves storage");
		cl.def("capacity", &Vector::capacity, "returns the number of elements that can be held in currently allocated storage");
		cl.def("shrink_to_fit", &Vector::shrink_to_fit, "reduces memory usage by freeing unused memory");
		cl.def("clear", &Vector::clear, "clears the contents");
		cl.def("swap", (void(Vector::*)(Vector &)) & Vector::swap, "swaps the contents");



		// cl.def("front", [](Vector &v) {
		// 		if (v.size()) return v.front();
		// 		else throw pybind11::index_error();
		// 	}, "access the first element");
		// cl.def("back", [](Vector &v) {
		// 		if (v.size()) return v.back();
		// 		else throw pybind11::index_error();
		// 	}, "access the last element ");
	}
};


template <typename Key, typename T, typename Compare, class Allocator> class map_binder
{
public:
	map_binder(pybind11::module &m, std::string const &key_name, std::string const &value_name, std::string const & /*compare name*/, std::string const & /*allocator name*/)
	{
		using Map = std::map<Key, T, Compare, Allocator>;
		using holder_type = std::shared_ptr< std::map<Key, T, Compare, Allocator> >;
		using Class_ = pybind11::class_<Map, holder_type>;

		Class_ cl = pybind11::bind_map<Map, holder_type>(m, "map_" + key_name + '_' + value_name);
	}
};

template <typename T, class Allocator> class deque_binder
{
	using Vector = std::deque<T, Allocator>;
	using SizeType = typename Vector::size_type;

	using Class_ = pybind11::class_<Vector, std::shared_ptr< Vector > >;


public:
	deque_binder(pybind11::module &m, std::string const &name, std::string const & /*allocator name*/)
	{
		using Vector = std::deque<T, Allocator>;
		using holder_type = std::shared_ptr<std::deque<T, Allocator>>;
		using Class_ = pybind11::class_<Vector, holder_type>;

		Class_ cl = pybind11::bind_vector<Vector, holder_type>(m, "deque_" + name);

		// cl.def(pybind11::init<size_type>());
		// cl.def("resize", (void (Vector::*) (size_type count)) & Vector::resize, "changes the number of elements stored");

		cl.def("empty", &Vector::empty, "checks whether the container is empty");
		cl.def("clear", &Vector::clear, "clears the contents");
		cl.def("swap", (void(Vector::*)(Vector &)) & Vector::swap, "swaps the contents");
	}
};


} // namespace binder

#endif // _INCLUDED_std_binders_hpp_
