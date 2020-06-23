/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "containers-precomp.h"  // Precompiled headers
//
#include <mrpt/containers/Parameters.h>
#include <mrpt/core/exceptions.h>

using namespace mrpt::containers;

static const char* typeIdxToStr(const std::size_t idx)
{
	switch (idx)
	{
		case 0:
			return "uninitialized";
		case 1:
			return "double";
		case 2:
			return "uint64_t";
		case 3:
			return "std::string";
		case 4:
			return "Parameters";
		default:
			return "undefined";
	};
}

bool Parameters::has(const std::string& s) const
{
	if (isProxy_)
	{
		auto p = internalValueAsSelf();
		ASSERTMSG_(p, "Cannot call has() on a value (!)");
		return p->has(s);
	}

	return data_.end() != data_.find(s);
}

std::string Parameters::typeOf(const std::string& name) const
{
	if (isProxy_)
	{
		auto p = internalValueAsSelf();
		ASSERTMSG_(p, "Cannot call typeOf() on a value (!)");
		return p->typeOf(name);
	}

	auto it = data_.find(name);
	if (data_.end() == it) return {};
	return typeIdxToStr(it->second.index());
}

const Parameters* Parameters::internalValueAsSelf() const
{
	const Parameters* p = nullptr;
	if (valuenc_ && std::holds_alternative<Parameters>(*valuenc_))
		p = &std::get<Parameters>(*valuenc_);
	if (value_ && std::holds_alternative<Parameters>(*value_))
		p = &std::get<Parameters>(*value_);
	return p;
}
Parameters* Parameters::internalValueAsSelf()
{
	Parameters* p = nullptr;
	if (valuenc_ && std::holds_alternative<Parameters>(*valuenc_))
		p = &std::get<Parameters>(*valuenc_);
	return p;
}
const Parameters* Parameters::internalMeOrValue() const
{
	const Parameters* p = internalValueAsSelf();
	return p != nullptr ? p : this;
}
Parameters* Parameters::internalMeOrValue()
{
	Parameters* p = internalValueAsSelf();
	return p != nullptr ? p : this;
}

bool Parameters::empty() const
{
	if (isProxy_)
	{
		auto p = internalValueAsSelf();
		ASSERTMSG_(p, "Cannot call empty() on a value (!)");
		return p->empty();
	}
	return data_.empty();
}
void Parameters::clear()
{
	if (isProxy_)
	{
		auto p = internalValueAsSelf();
		ASSERTMSG_(p, "Cannot call clear() on a value (!)");
		p->clear();
	}
	data_.clear();
}

Parameters Parameters::operator[](const char* s)
{
	ASSERT_(s != nullptr);
	Parameters* p = internalMeOrValue();
	return Parameters(internal::tag_as_proxy_t(), p->data_[s], s);
}

const Parameters Parameters::operator[](const char* s) const
{
	ASSERT_(s != nullptr);
	const Parameters* p = internalMeOrValue();
	auto it = p->data_.find(s);
	if (p->data_.end() == it)
		THROW_EXCEPTION_FMT("Access non-existing parameter `%s`", s);

	return Parameters(internal::tag_as_const_proxy_t(), it->second, s);
}

template <typename T>
const T& mrpt::containers::internal::implAsGetter(
	const Parameters& p, const char* expectedType)
{
	ASSERT_(p.isProxy_);
	try
	{
		if (p.isConstProxy_)
			return std::get<T>(*p.value_);
		else
			return std::get<T>(*p.valuenc_);
	}
	catch (const std::bad_variant_access&)
	{
		THROW_EXCEPTION_FMT(
			"Trying to read parameter `%s` of type `%s` as if it was `%s`",
			p.name_, typeIdxToStr(p.value_->index()), expectedType);
	}
}

template <>
const double& mrpt::containers::internal::asGetter(const Parameters& p)
{
	return implAsGetter<double>(p, "double");
}
template <>
const uint64_t& mrpt::containers::internal::asGetter(const Parameters& p)
{
	return implAsGetter<uint64_t>(p, "uint64_t");
}
template <>
const std::string& mrpt::containers::internal::asGetter(const Parameters& p)
{
	return implAsGetter<std::string>(p, "std::string");
}
template <>
const Parameters& mrpt::containers::internal::asGetter(const Parameters& p)
{
	return implAsGetter<Parameters>(p, "Parameters");
}
