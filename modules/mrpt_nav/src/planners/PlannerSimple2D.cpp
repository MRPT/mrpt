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

#include <mrpt/math/TPose2D.h>
#include <mrpt/nav/planners/PlannerSimple2D.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::nav;
using namespace std;

// -------------------------------------------------------------------------
// File-scope types and constants shared between helpers and computePath()
// -------------------------------------------------------------------------
using cell_t = int32_t;

constexpr cell_t CELL_ORIGIN = 0;
constexpr cell_t CELL_EMPTY = 0x8000000;
constexpr cell_t CELL_OBSTACLE = 0xfffffff;
constexpr cell_t CELL_TARGET = 0xffffffe;

/** Build the wave-front search grid from an occupancy map.
 *  Cells are filled with CELL_EMPTY (free) or CELL_OBSTACLE, obstacles are
 *  dilated by robotRadius, and the origin/target cells are stamped.
 */
static std::vector<cell_t> initObstacleGrid(
    const COccupancyGridMap2D& theMap,
    const TPoint2D& origin,
    const TPoint2D& target,
    float occupancyThreshold,
    float robotRadius)
{
  const int size_x = theMap.getSizeX();
  const int size_y = theMap.getSizeY();

  std::vector<cell_t> grid(size_x * size_y);

  // Fill from occupancy map:
  for (int y = 0; y < size_y; y++)
  {
    const int row = y * size_x;
    for (int x = 0; x < size_x; x++)
      grid[x + row] = (theMap.getCell(x, y) > occupancyThreshold) ? CELL_EMPTY : CELL_OBSTACLE;
  }

  // Enlarge obstacles by robotRadius:
  const int obsEnlargement = static_cast<int>(ceil(robotRadius / theMap.getResolution()));
  for (int nEnlargements = 0; nEnlargements < obsEnlargement; nEnlargements++)
  {
    for (int y = 2; y < size_y - 2; y++)
    {
      const int row = y * size_x;
      const int row_1 = (y + 1) * size_x;
      const int row__1 = (y - 1) * size_x;
      const cell_t val = CELL_OBSTACLE - nEnlargements;

      for (int x = 2; x < size_x - 2; x++)
      {
        if (grid[x - 1 + row__1] >= val || grid[x + row__1] >= val || grid[x + 1 + row__1] >= val ||
            grid[x - 1 + row] >= val || grid[x + 1 + row] >= val || grid[x - 1 + row_1] >= val ||
            grid[x + row_1] >= val || grid[x + 1 + row_1] >= val)
        {
          grid[x + row] = std::max(grid[x + row], val - 1);
        }
      }
    }
  }

  // Finalize enlarged obstacles:
  for (auto& cell : grid)
    if (cell > CELL_EMPTY) cell = CELL_OBSTACLE;

  // Stamp origin and target special cells:
  grid[theMap.x2idx(origin.x) + size_x * theMap.y2idx(origin.y)] = CELL_ORIGIN;
  grid[theMap.x2idx(target.x) + size_x * theMap.y2idx(target.y)] = CELL_TARGET;

  return grid;
}

/** Reconstruct the path from the wave-front convergence cell to both the
 *  origin (backward trace) and the target (forward trace), then subsample
 *  into a 2D point deque.
 *
 *  Returns false and leaves `path` empty if the max path length is exceeded.
 */
static bool reconstructPath(
    const std::vector<cell_t>& grid,
    int size_x,
    int passCellFound_x,
    int passCellFound_y,
    const TPoint2D& origin,
    const TPoint2D& target,
    const COccupancyGridMap2D& theMap,
    double minStepInReturnedPath,
    float maxSearchPathLength,
    std::deque<TPoint2D>& path)
{
  std::vector<cell_t> pathcells_x, pathcells_y;

  // STEP 1: Trace-back toward origin
  {
    int x = passCellFound_x, y = passCellFound_y;
    cell_t v = 0, c;
    while ((v = grid[x + size_x * y]) != CELL_ORIGIN)
    {
      pathcells_x.push_back(x);
      pathcells_y.push_back(y);

      int8_t dx = 0, dy = 0;
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
  }

  // STEP 2: Reverse the backward path
  {
    const int n = pathcells_x.size();
    for (int i = 0; i < n / 2; i++)
    {
      std::swap(pathcells_x[i], pathcells_x[n - 1 - i]);
      std::swap(pathcells_y[i], pathcells_y[n - 1 - i]);
    }
  }

  // STEP 3: Trace-forward toward target
  {
    int x = passCellFound_x, y = passCellFound_y;
    cell_t v = 0, c;
    signed char dx = 0, dy = 0;
    while ((v = grid[x + size_x * y]) != CELL_TARGET)
    {
      pathcells_x.push_back(x);
      pathcells_y.push_back(y);

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
  }

  // STEP 4: Subsample path cells → 2D world points
  path.clear();
  const int n = pathcells_x.size();
  double last_xx = origin.x, last_yy = origin.y;
  auto last_cx = theMap.x2idx(origin.x);
  auto last_cy = theMap.y2idx(origin.y);

  const auto minDistSqrCells =
      mrpt::round(mrpt::square(minStepInReturnedPath / theMap.getResolution()));
  double accumDist = 0;

  for (int i = 0; i < n; i++)
  {
    const auto distSqrCells = square(pathcells_x[i] - last_cx) + square(pathcells_y[i] - last_cy);
    if (distSqrCells > minDistSqrCells)
    {
      const auto xx = theMap.idx2x(pathcells_x[i]);
      const auto yy = theMap.idx2y(pathcells_y[i]);
      path.emplace_back(xx, yy);
      accumDist += std::sqrt(square(xx - last_xx) + square(yy - last_yy));
      last_cx = pathcells_x[i];
      last_cy = pathcells_y[i];
      last_xx = xx;
      last_yy = yy;
    }
    if (maxSearchPathLength > 0 && accumDist > maxSearchPathLength)
    {
      path.clear();
      return false;
    }
  }

  path.emplace_back(target.x, target.y);
  return true;
}

/*---------------------------------------------------------------
            computePath
  ---------------------------------------------------------------*/
void PlannerSimple2D::computePath(
    const COccupancyGridMap2D& theMap,
    const CPose2D& origin_,
    const CPose2D& target_,
    std::deque<math::TPoint2D>& path,
    bool& notFound,
    float maxSearchPathLength) const
{
  path.clear();

  const TPoint2D origin = TPoint2D(origin_.asTPose());
  const TPoint2D target = TPoint2D(target_.asTPose());

  // Check that origin and target fall inside the grid:
  if (!((origin.x > theMap.getXMin() && origin.x < theMap.getXMax() &&
         origin.y > theMap.getYMin() && origin.y < theMap.getYMax()) ||
        !(target.x > theMap.getXMin() && target.x < theMap.getXMax() &&
          target.y > theMap.getYMin() && target.y < theMap.getYMax())))
  {
    notFound = true;
    return;
  }

  // Special case: origin and target in the same cell:
  if (theMap.x2idx(origin.x) == theMap.x2idx(target.x) &&
      theMap.y2idx(origin.y) == theMap.y2idx(target.y))
  {
    path.emplace_back(target.x, target.y);
    notFound = false;
    return;
  }

  const int size_x = theMap.getSizeX();
  const int size_y = theMap.getSizeY();

  // Build the search grid (free/obstacle/origin/target cells):
  auto grid = initObstacleGrid(theMap, origin, target, occupancyThreshold, robotRadius);

  // Wave-front search loop:
  bool searching = true;
  notFound = false;
  int passCellFound_x = -1, passCellFound_y = -1;

  cell_t minNeigh = CELL_EMPTY, maxNeigh = CELL_EMPTY;

  int range_x_min = std::min(theMap.x2idx(origin.x) - 1, theMap.x2idx(target.x) - 1);
  int range_x_max = std::max(theMap.x2idx(origin.x) + 1, theMap.x2idx(target.x) + 1);
  int range_y_min = std::min(theMap.y2idx(origin.y) - 1, theMap.y2idx(target.y) - 1);
  int range_y_max = std::max(theMap.y2idx(origin.y) + 1, theMap.y2idx(target.y) + 1);

  do
  {
    notFound = true;
    bool wave1Found = false, wave2Found = false;

    range_x_min = std::max(1, range_x_min - 1);
    range_x_max = std::min(size_x - 1, range_x_max + 1);
    range_y_min = std::max(1, range_y_min - 1);
    range_y_max = std::min(size_y - 1, range_y_max + 1);

    for (int y = range_y_min; y < range_y_max && passCellFound_x == -1; y++)
    {
      const int row = y * size_x;
      const int row_1 = (y + 1) * size_x;
      const int row__1 = (y - 1) * size_x;
      cell_t metric, v;

      for (int x = range_x_min; x < range_x_max; x++)
      {
        if (grid[x + row] != CELL_EMPTY) continue;

        // Scan 8-neighbourhood for wave fronts:
        minNeigh = maxNeigh = CELL_EMPTY;
        metric = 2;

        auto checkNeigh4 = [&](cell_t nv)
        {
          if (nv + 2 < minNeigh) minNeigh = nv + 2;
          if (nv - 2 > maxNeigh && nv != CELL_OBSTACLE) maxNeigh = nv - 2;
        };
        auto checkNeigh8 = [&](cell_t nv)
        {
          if (nv + 3 < minNeigh)
          {
            metric = 3;
            minNeigh = nv + 3;
          }
          if (nv - 3 > maxNeigh && nv != CELL_OBSTACLE)
          {
            metric = 3;
            maxNeigh = nv - 3;
          }
        };

        checkNeigh4(grid[x + row__1]);
        checkNeigh4(grid[x - 1 + row]);
        checkNeigh4(grid[x + 1 + row]);
        checkNeigh4(grid[x + row_1]);
        checkNeigh8(grid[x - 1 + row__1]);
        checkNeigh8(grid[x + 1 + row__1]);
        checkNeigh8(grid[x - 1 + row_1]);
        checkNeigh8(grid[x + 1 + row_1]);

        if (minNeigh < CELL_EMPTY && maxNeigh > CELL_EMPTY)
        {
          // Convergence: shortest path found
          passCellFound_x = x;
          passCellFound_y = y;
          searching = false;
          break;
        }
        else if (minNeigh < CELL_EMPTY)
        {
          wave1Found = true;
          v = minNeigh + metric;
          grid[x + row] = v;
          ASSERT_(v < CELL_EMPTY);
        }
        else if (maxNeigh > CELL_EMPTY)
        {
          wave2Found = true;
          v = maxNeigh - metric;
          grid[x + row] = v;
          ASSERT_(v > CELL_EMPTY);
        }
      }
    }

    notFound = !wave1Found && !wave2Found;

    const int estimPathLen = std::min(minNeigh + 1, CELL_TARGET - maxNeigh);
    if (maxSearchPathLength > 0 && estimPathLen * theMap.getResolution() > maxSearchPathLength)
    {
      notFound = true;
      break;
    }

  } while (!notFound && searching);

  if (notFound) return;

  // Reconstruct path from convergence cell and subsample it:
  if (!reconstructPath(
          grid, size_x, passCellFound_x, passCellFound_y, origin, target, theMap,
          minStepInReturnedPath, maxSearchPathLength, path))
  {
    notFound = true;
  }

  // That's all!! :-)
}
