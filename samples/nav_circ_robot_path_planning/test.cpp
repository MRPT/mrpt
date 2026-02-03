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

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/nav/planners/PlannerSimple2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/filesystem.h>

#include <iostream>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::nav;
using namespace mrpt::serialization;
using namespace mrpt::img;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::io;
using namespace mrpt::system;
using namespace std;

#include <mrpt/examples_config.h>

string myGridMap(
    MRPT_EXAMPLES_BASE_DIRECTORY + string("../share/mrpt/datasets/2006-MalagaCampus.gridmap.gz"));

// ------------------------------------------------------
//				TestPathPlanning
// ------------------------------------------------------
void TestPathPlanning()
{
  // Load the gridmap:
  COccupancyGridMap2D gridmap;

  if (!mrpt::system::fileExists(myGridMap))
    THROW_EXCEPTION_FMT("Map file '%s' not found", myGridMap.c_str());

  printf("Loading gridmap...");
  {
    CCompressedInputStream f(myGridMap);
    auto arch = archiveFrom(f);
    arch >> gridmap;
  }
  printf(
      "Done! %f x %f m\n", gridmap.getXMax() - gridmap.getXMin(),
      gridmap.getYMax() - gridmap.getYMin());

  // Find path:
  PlannerSimple2D pathPlanning;
  pathPlanning.robotRadius = 0.30f;

  std::deque<TPoint2D> thePath;
  bool notFound;
  CTicTac tictac;

  CPose2D origin(20, -110, 0);
  CPose2D target(90, 40, 0);

  cout << "Origin: " << origin << endl;
  cout << "Target: " << target << endl;

  cout << "Searching path...";
  cout.flush();
  tictac.Tic();

  pathPlanning.computePath(gridmap, origin, target, thePath, notFound);

  double t = tictac.Tac();
  cout << "Done in " << t * 1000 << " ms" << endl;

  printf("Path found: %s\n", notFound ? "NO" : "YES");
  printf("Path has %u steps\n", (unsigned)thePath.size());

  // Save result:
  CImage img;
  gridmap.getAsImage(img, false, true);  // Force a RGB image

  // Draw the path:
  // ---------------------
  int R = round(pathPlanning.robotRadius / gridmap.getResolution());

  for (std::deque<TPoint2D>::const_iterator it = thePath.begin(); it != thePath.end(); ++it)
    img.drawCircle(
        gridmap.x2idx(it->x), gridmap.getSizeY() - 1 - gridmap.y2idx(it->y), R, TColor(0, 0, 255));

  img.drawMark(
      gridmap.x2idx(origin.x()), gridmap.getSizeY() - 1 - gridmap.y2idx(origin.y()),
      TColor(0x20, 0x20, 0x20), '+', 10);
  img.drawMark(
      gridmap.x2idx(target.x()), gridmap.getSizeY() - 1 - gridmap.y2idx(target.y()),
      TColor(0x50, 0x50, 0x50), 'x', 10);

  const std::string dest = "path_planning.png";
  cout << "Saving output to: " << dest << endl;
  bool savedOk = img.saveToFile(dest);
  ASSERT_(savedOk);
  printf("Done\n");

#if MRPT_HAS_WXWIDGETS
  mrpt::gui::CDisplayWindow3D win("Computed path");
  win.setImageView(img);
  win.repaint();

  win.waitForKey();
#endif
}

int main(int argc, char** argv)
{
  try
  {
    TestPathPlanning();
    return 0;
  }
  catch (exception& e)
  {
    cout << "MRPT exception caught: " << e.what() << endl;
    return -1;
  }
  catch (...)
  {
    printf("Another exception!!");
    return -1;
  }
}
