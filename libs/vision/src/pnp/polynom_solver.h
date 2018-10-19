/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

int solve_deg2(double a, double b, double c, double& x1, double& x2);

int solve_deg3(
	double a, double b, double c, double d, double& x0, double& x1, double& x2);

int solve_deg4(
	double a, double b, double c, double d, double e, double& x0, double& x1,
	double& x2, double& x3);
