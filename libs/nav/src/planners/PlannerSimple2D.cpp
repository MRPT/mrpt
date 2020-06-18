/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "nav-precomp.h"  // Precompiled headers

#include <mrpt/math/TPose2D.h>
#include <mrpt/nav/planners/PlannerSimple2D.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::nav;
using namespace std;

/*---------------------------------------------------------------
						computePath
  ---------------------------------------------------------------*/
void PlannerSimple2D::computePath(
	const COccupancyGridMap2D& theMap, const CPose2D& origin_,
	const CPose2D& target_, std::deque<math::TPoint2D>& path, bool& notFound,
	float maxSearchPathLength) const
{
	constexpr uint16_t CELL_ORIGIN = 0x0000;
	constexpr uint16_t CELL_EMPTY = 0x8000;
	constexpr uint16_t CELL_OBSTACLE = 0xFFFF;
	constexpr uint16_t CELL_TARGET = 0xFFFE;

	path.clear();

	const TPoint2D origin = TPoint2D(origin_.asTPose());
	const TPoint2D target = TPoint2D(target_.asTPose());

	std::vector<uint16_t> grid;
	int size_x, size_y, i, n, m;
	int x, y;
	bool searching;
	uint16_t minNeigh = CELL_EMPTY, maxNeigh = CELL_EMPTY, v = 0, c;
	int passCellFound_x = -1, passCellFound_y = -1;
	std::vector<uint16_t> pathcells_x, pathcells_y;

	// Check that origin and target falls inside the grid theMap
	// -----------------------------------------------------------
	if (!((origin.x > theMap.getXMin() && origin.x < theMap.getXMax() &&
		   origin.y > theMap.getYMin() && origin.y < theMap.getYMax()) ||
		  !(target.x > theMap.getXMin() && target.x < theMap.getXMax() &&
			target.y > theMap.getYMin() && target.y < theMap.getYMax())))
	{
		notFound = true;
		return;
	}

	// Check for the special case of origin and target in the same cell:
	// -----------------------------------------------------------------
	if (theMap.x2idx(origin.x) == theMap.x2idx(target.x) &&
		theMap.y2idx(origin.y) == theMap.y2idx(target.y))
	{
		path.emplace_back(target.x, target.y);
		notFound = false;
		return;
	}

	// Get the grid size:
	// -----------------------------------------------------------
	size_x = theMap.getSizeX();
	size_y = theMap.getSizeY();

	// Fill the grid content with free-space and obstacles:
	// -----------------------------------------------------------
	grid.resize(size_x * size_y);
	for (y = 0; y < size_y; y++)
	{
		int row = y * size_x;
		for (x = 0; x < size_x; x++)
		{
			grid[x + row] = (theMap.getCell(x, y) > occupancyThreshold)
								? CELL_EMPTY
								: CELL_OBSTACLE;
		}
	}

	// Enlarge obstacles with the robot radius:
	// -----------------------------------------------------------
	int obsEnlargement = (int)(ceil(robotRadius / theMap.getResolution()));
	for (int nEnlargements = 0; nEnlargements < obsEnlargement; nEnlargements++)
	{
		// For all cells(x,y)=EMPTY:
		// -----------------------------
		for (y = 2; y < size_y - 2; y++)
		{
			int row = y * size_x;
			int row_1 = (y + 1) * size_x;
			int row__1 = (y - 1) * size_x;

			for (x = 2; x < size_x - 2; x++)
			{
				uint16_t val = (CELL_OBSTACLE - nEnlargements);

				//  A cell near an obstacle found??
				// -----------------------------------------------------
				if (grid[x - 1 + row__1] >= val || grid[x + row__1] >= val ||
					grid[x + 1 + row__1] >= val || grid[x - 1 + row] >= val ||
					grid[x + 1 + row] >= val || grid[x - 1 + row_1] >= val ||
					grid[x + row_1] >= val || grid[x + 1 + row_1] >= val)
				{
					grid[x + row] =
						max((uint16_t)grid[x + row], (uint16_t)(val - 1));
				}
			}
		}
	}

	// Definitevely set new obstacles as obstacles
	for (y = 1; y < size_y - 1; y++)
	{
		int row = y * size_x;
		for (x = 1; x < size_x - 1; x++)
		{
			if (grid[x + row] > CELL_EMPTY) grid[x + row] = CELL_OBSTACLE;
		}
	}

	// Put the special cell codes for the origin and target:
	// -----------------------------------------------------------
	grid[theMap.x2idx(origin.x) + size_x * theMap.y2idx(origin.y)] =
		CELL_ORIGIN;
	grid[theMap.x2idx(target.x) + size_x * theMap.y2idx(target.y)] =
		CELL_TARGET;

	// The main path search loop:
	// -----------------------------------------------------------
	searching = true;  // Will become false on path found
	notFound = false;  // Will be true inside the loop if a path is not found

	int range_x_min =
		std::min(theMap.x2idx(origin.x) - 1, theMap.x2idx(target.x) - 1);
	int range_x_max =
		std::max(theMap.x2idx(origin.x) + 1, theMap.x2idx(target.x) + 1);
	int range_y_min =
		std::min(theMap.y2idx(origin.y) - 1, theMap.y2idx(target.y) - 1);
	int range_y_max =
		std::max(theMap.y2idx(origin.y) + 1, theMap.y2idx(target.y) + 1);

	do
	{
		notFound = true;
		bool wave1Found = false, wave2Found = false;
		int size_y_1 = size_y - 1;
		int size_x_1 = size_x - 1;

		range_x_min = std::max(1, range_x_min - 1);
		range_x_max = std::min(size_x_1, range_x_max + 1);
		range_y_min = std::max(1, range_y_min - 1);
		range_y_max = std::min(size_y_1, range_y_max + 1);

		// For all cells(x,y)=EMPTY:
		// -----------------------------
		for (y = range_y_min; y < range_y_max && passCellFound_x == -1; y++)
		{
			int row = y * size_x;
			int row_1 = (y + 1) * size_x;
			int row__1 = (y - 1) * size_x;
			// metric: 2 horz.vert, =3 diagonal <-- Since 3/2 ~= sqrt(2)
			int16_t metric;

			for (x = range_x_min; x < range_x_max; x++)
			{
				if (grid[x + row] != CELL_EMPTY) continue;

				//  Look in the neighboorhood:
				// -----------------------------
				minNeigh = maxNeigh = CELL_EMPTY;
				metric = 2;
				v = grid[x + row__1];
				if (v + 2 < minNeigh) minNeigh = v + 2;
				if (v - 2 > maxNeigh && v != CELL_OBSTACLE) maxNeigh = v - 2;
				v = grid[x - 1 + row];
				if (v + 2 < minNeigh) minNeigh = v + 2;
				if (v - 2 > maxNeigh && v != CELL_OBSTACLE) maxNeigh = v - 2;
				v = grid[x + 1 + row];
				if (v + 2 < minNeigh) minNeigh = v + 2;
				if (v - 2 > maxNeigh && v != CELL_OBSTACLE) maxNeigh = v - 2;
				v = grid[x + row_1];
				if (v + 2 < minNeigh) minNeigh = v + 2;
				if (v - 2 > maxNeigh && v != CELL_OBSTACLE) maxNeigh = v - 2;

				v = grid[x - 1 + row__1];
				if ((v + 3) < minNeigh)
				{
					metric = 3;
					minNeigh = (v + 3);
				}
				if ((v - 3) > maxNeigh && v != CELL_OBSTACLE)
				{
					metric = 3;
					maxNeigh = v - 3;
				}
				v = grid[x + 1 + row__1];
				if ((v + 3) < minNeigh)
				{
					metric = 3;
					minNeigh = (v + 3);
				}
				if ((v - 3) > maxNeigh && v != CELL_OBSTACLE)
				{
					metric = 3;
					maxNeigh = v - 3;
				}
				v = grid[x - 1 + row_1];
				if ((v + 3) < minNeigh)
				{
					metric = 3;
					minNeigh = (v + 3);
				}
				if ((v - 3) > maxNeigh && v != CELL_OBSTACLE)
				{
					metric = 3;
					maxNeigh = v - 3;
				}
				v = grid[x + 1 + row_1];
				if ((v + 3) < minNeigh)
				{
					metric = 3;
					minNeigh = (v + 3);
				}
				if ((v - 3) > maxNeigh && v != CELL_OBSTACLE)
				{
					metric = 3;
					maxNeigh = v - 3;
				}

				//  Convergence cell found? = The shortest path found
				// -----------------------------------------------------
				if (minNeigh < CELL_EMPTY && maxNeigh > CELL_EMPTY)
				{
					// Stop the search:
					passCellFound_x = x;
					passCellFound_y = y;
					searching = false;
					break;
				}
				else if (minNeigh < CELL_EMPTY)
				{
					wave1Found = true;

					// Cell in the expansion-wave from origin
					grid[x + row] = minNeigh + metric;
					ASSERT_(minNeigh + metric < CELL_EMPTY);
				}
				else if (maxNeigh > CELL_EMPTY)
				{
					wave2Found = true;

					// Cell in the expansion-wave from the target
					grid[x + row] = maxNeigh - metric;
					ASSERT_(maxNeigh - metric > CELL_EMPTY);
				}
				else
				{  // Nothing to do: A free cell inside of all also free
				   // cells.
				}
			}  // end for x
		}  // end for y

		notFound = !wave1Found && !wave2Found;

		// Check max. path:
		const int estimPathLen = std::min(minNeigh + 1, CELL_TARGET - maxNeigh);

		if (maxSearchPathLength > 0 &&
			estimPathLen * theMap.getResolution() > maxSearchPathLength)
		{
			notFound = true;
			break;
		}

	} while (!notFound && searching);

	// Path not found:
	if (notFound) return;

	// Rebuild the optimal path from the two-waves convergence cell
	// ----------------------------------------------------------------

	// STEP 1: Trace-back to origin
	//-------------------------------------
	// An (exact?) estimation of the final vector size:
	pathcells_x.reserve((minNeigh + 1) + (CELL_TARGET - maxNeigh));
	pathcells_y.reserve((minNeigh + 1) + (CELL_TARGET - maxNeigh));
	x = passCellFound_x;
	y = passCellFound_y;

	while ((v = grid[x + size_x * y]) != CELL_ORIGIN)
	{
		// Add cell to the path (in inverse order, now we go backward!!) Later
		// is will be reversed
		pathcells_x.push_back(x);
		pathcells_y.push_back(y);

		// Follow the "negative gradient" toward the origin:
		static signed char dx = 0, dy = 0;
		if ((c = grid[x - 1 + size_x * y]) < v)
		{
			v = c;
			dx = -1;
			dy = 0;
		}
		if ((c = grid[x + 1 + size_x * y]) < v)
		{
			v = c;
			dx = 1;
			dy = 0;
		}
		if ((c = grid[x + size_x * (y - 1)]) < v)
		{
			v = c;
			dx = 0;
			dy = -1;
		}
		if ((c = grid[x + size_x * (y + 1)]) < v)
		{
			v = c;
			dx = 0;
			dy = 1;
		}

		if ((c = grid[x - 1 + size_x * (y - 1)]) < v)
		{
			v = c;
			dx = -1;
			dy = -1;
		}
		if ((c = grid[x + 1 + size_x * (y - 1)]) < v)
		{
			v = c;
			dx = 1;
			dy = -1;
		}
		if ((c = grid[x - 1 + size_x * (y + 1)]) < v)
		{
			v = c;
			dx = -1;
			dy = 1;
		}
		if ((c = grid[x + 1 + size_x * (y + 1)]) < v)
		{
			v = c;
			dx = 1;
			dy = 1;
		}

		ASSERT_(dx != 0 || dy != 0);
		x += dx;
		y += dy;
	}

	// STEP 2: Reverse the path, since we want it from the origin
	//   toward the convergence cell
	//--------------------------------------------------------------
	n = pathcells_x.size();
	m = n / 2;
	for (i = 0; i < m; i++)
	{
		v = pathcells_x[i];
		pathcells_x[i] = pathcells_x[n - 1 - i];
		pathcells_x[n - 1 - i] = v;

		v = pathcells_y[i];
		pathcells_y[i] = pathcells_y[n - 1 - i];
		pathcells_y[n - 1 - i] = v;
	}

	// STEP 3: Trace-foward toward the target
	//-------------------------------------
	x = passCellFound_x;
	y = passCellFound_y;

	while ((v = grid[x + size_x * y]) != CELL_TARGET)
	{
		// Add cell to the path
		pathcells_x.push_back(x);
		pathcells_y.push_back(y);

		// Follow the "positive gradient" toward the target:
		static signed char dx = 0, dy = 0;
		c = grid[x - 1 + size_x * y];
		if (c > v && c != CELL_OBSTACLE)
		{
			v = c;
			dx = -1;
			dy = 0;
		}
		c = grid[x + 1 + size_x * y];
		if (c > v && c != CELL_OBSTACLE)
		{
			v = c;
			dx = 1;
			dy = 0;
		}
		c = grid[x + size_x * (y - 1)];
		if (c > v && c != CELL_OBSTACLE)
		{
			v = c;
			dx = 0;
			dy = -1;
		}
		c = grid[x + size_x * (y + 1)];
		if (c > v && c != CELL_OBSTACLE)
		{
			v = c;
			dx = 0;
			dy = 1;
		}

		c = grid[x - 1 + size_x * (y - 1)];
		if (c > v && c != CELL_OBSTACLE)
		{
			v = c;
			dx = -1;
			dy = -1;
		}
		c = grid[x + 1 + size_x * (y - 1)];
		if (c > v && c != CELL_OBSTACLE)
		{
			v = c;
			dx = 1;
			dy = -1;
		}
		c = grid[x - 1 + size_x * (y + 1)];
		if (c > v && c != CELL_OBSTACLE)
		{
			v = c;
			dx = -1;
			dy = 1;
		}
		c = grid[x + 1 + size_x * (y + 1)];
		if (c > v && c != CELL_OBSTACLE)
		{
			v = c;
			dx = 1;
			dy = 1;
		}

		ASSERT_(dx != 0 || dy != 0);
		x += dx;
		y += dy;
	}

	// STEP 4: Translate the path-of-cells to a path-of-2d-points with
	// subsampling
	//-------------------------------------------------------------------------------
	path.clear();
	n = pathcells_x.size();
	float xx, yy;
	float last_xx = origin.x, last_yy = origin.y;
	const float minDistSqr = mrpt::square(minStepInReturnedPath);
	float accumDist = 0;
	for (i = 0; i < n; i++)
	{
		// Get cell coordinates:
		xx = theMap.idx2x(pathcells_x[i]);
		yy = theMap.idx2y(pathcells_y[i]);

		// Enough distance??
		const float distSqr = square(xx - last_xx) + square(yy - last_yy);
		if (distSqr > minDistSqr)
		{
			// Add to the path:
			path.emplace_back(xx, yy);

			// For the next iteration:
			last_xx = xx;
			last_yy = yy;

			accumDist += std::sqrt(distSqr);
		}

		if (maxSearchPathLength > 0 && accumDist > maxSearchPathLength)
		{
			notFound = true;
			path.clear();
			return;
		}
	}

	// Add the target point:
	path.emplace_back(target.x, target.y);

	// That's all!! :-)
}
