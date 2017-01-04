/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// This file implements the part of the class depending on `exprtk`, so it does not 
// need to be recompiled so often (exprtk takes *a long time* to compile...)

#include "nav-precomp.h" // Precomp header
#include <mrpt/nav/tpspace/CPTG_Holo_Blend.h>
#define exprtk_disable_string_capabilities   // Workaround a bug in Ubuntu precise's GCC+libstdc++
#include <mrpt/otherlibs/exprtk.hpp>

PIMPL_IMPLEMENT(exprtk::expression<double>);

using namespace mrpt::nav;

CPTG_Holo_Blend::CPTG_Holo_Blend() :
	T_ramp_max(-1.0),
	V_MAX(-1.0),
	W_MAX(-1.0),
	turningRadiusReference(0.30),
	curVelLocal(0,0,0)
{
	internal_init_exprtks();
}

CPTG_Holo_Blend::CPTG_Holo_Blend(const mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) :
	turningRadiusReference(0.30),
	curVelLocal(0,0,0)
{
	internal_init_exprtks();
	this->loadFromConfigFile(cfg,sSection);
}

CPTG_Holo_Blend::~CPTG_Holo_Blend()
{
}

void CPTG_Holo_Blend::internal_init_exprtks()
{
	PIMPL_CONSTRUCT(exprtk::expression<double>, m_expr_v);
	PIMPL_CONSTRUCT(exprtk::expression<double>, m_expr_w);
	PIMPL_CONSTRUCT(exprtk::expression<double>, m_expr_T_ramp);

	exprtk::symbol_table<double> symbol_table;

	symbol_table.add_variable("dir", m_expr_dir);
	symbol_table.add_variable("V_MAX", V_MAX);
	symbol_table.add_variable("W_MAX", W_MAX); 
	symbol_table.add_variable("T_ramp_max", T_ramp_max);
	symbol_table.add_constants();

	PIMPL_GET_REF(exprtk::expression<double>, m_expr_v).register_symbol_table(symbol_table);
	PIMPL_GET_REF(exprtk::expression<double>, m_expr_w).register_symbol_table(symbol_table);
	PIMPL_GET_REF(exprtk::expression<double>, m_expr_T_ramp).register_symbol_table(symbol_table);

	// Default expressions (can be overloaded by values in a config file)
	expr_V = "V_MAX";
	expr_W = "W_MAX";
	expr_T_ramp = "T_ramp_max";
}

double CPTG_Holo_Blend::internal_get_v(const double dir) const
{
	const_cast<double&>(m_expr_dir) = dir;
	return std::abs(PIMPL_GET_CONSTREF(exprtk::expression<double>, m_expr_v).value());
}
double CPTG_Holo_Blend::internal_get_w(const double dir) const
{
	const_cast<double&>(m_expr_dir) = dir;
	return std::abs(PIMPL_GET_CONSTREF(exprtk::expression<double>, m_expr_w).value());
}
double CPTG_Holo_Blend::internal_get_T_ramp(const double dir) const
{
	const_cast<double&>(m_expr_dir) = dir;
	return PIMPL_GET_CONSTREF(exprtk::expression<double>, m_expr_T_ramp).value();
}

void CPTG_Holo_Blend::internal_initialize(const std::string & cacheFilename, const bool verbose )
{
	// No need to initialize anything, just do some params sanity checks:
	ASSERT_(T_ramp_max>0);
	ASSERT_(V_MAX>0);
	ASSERT_(W_MAX>0);
	ASSERT_(m_alphaValuesCount>0);
	ASSERT_(m_robotRadius>0);

	// Compile user-given expressions:
	exprtk::parser<double> parser;

	if (!parser.compile(expr_V, PIMPL_GET_REF(exprtk::expression<double>, m_expr_v)))
		THROW_EXCEPTION(mrpt::format("Error compiling `expr_V` expression: `%s`. Error: `%s`", expr_V.c_str(),parser.error().c_str()));
	if (!parser.compile(expr_W, PIMPL_GET_REF(exprtk::expression<double>, m_expr_w)))
		THROW_EXCEPTION(mrpt::format("Error compiling `expr_W` expression: `%s`. Error: `%s`", expr_W.c_str(), parser.error().c_str()));
	if (!parser.compile(expr_T_ramp, PIMPL_GET_REF(exprtk::expression<double>, m_expr_T_ramp)))
		THROW_EXCEPTION(mrpt::format("Error compiling `expr_T_ramp` expression: `%s`. Error: `%s`", expr_T_ramp.c_str(), parser.error().c_str()));

#ifdef DO_PERFORMANCE_BENCHMARK
	tl.dumpAllStats();
#endif
}
