/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

int solve_deg2(double a, double b, double c, double& x1, double& x2);

int solve_deg3(double a, double b, double c, double d, double& x0, double& x1, double& x2);

int solve_deg4(
    double a,
    double b,
    double c,
    double d,
    double e,
    double& x0,
    double& x1,
    double& x2,
    double& x3);
