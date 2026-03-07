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

#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/vision/CFeatureExtraction.h>

#include <chrono>
#include <thread>

using namespace mrpt::gui;
using namespace mrpt::img;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::vision;
using namespace mrpt;
using namespace std;

static string file1, file2;

/** Helper: draw feature marks on an image (v3 API). */
static void drawFeaturesOnImage(
    mrpt::img::CImage& img, const CFeatureList& feats, const TColor& color)
{
  for (const auto& f : feats)
  {
    img.drawMark({f.keypoint.pt.x, f.keypoint.pt.y}, color, '+', 7);
  }
}

bool DemoFeatures()
{
  // Ask for the pair of images:
  cout << "Note: On all questions, [enter] means taking the default value" << endl << endl;

  if (file1.empty())
  {
    cout << "Enter path to image #1 [sample_image1]: ";
    std::getline(cin, file1);
  }
  else
    cout << "Image #1: " << file1 << endl;

  if (file2.empty())
  {
    cout << "Enter path to image #2 [sample_image2]: ";
    std::getline(cin, file2);
  }
  else
    cout << "Image #2: " << file2 << endl;

  // Max. num of features:
  cout << endl << "Maximum number of features [150 (default), 0: Infinite]:";
  string sel_num_feats;
  std::getline(cin, sel_num_feats);

  const size_t nFeats =
      sel_num_feats.empty() ? 150 : static_cast<size_t>(::atoi(sel_num_feats.c_str()));

  CImage img1, img2;

  if (!file1.empty())
  {
    if (!img1.loadFromFile(file1)) THROW_EXCEPTION_FMT("Error loading file: %s", file1.c_str());
  }
  else
    mrpt::obs::stock_observations::exampleImage(img1, 0);

  if (!file2.empty())
  {
    if (!img2.loadFromFile(file2)) THROW_EXCEPTION_FMT("Error loading file: %s", file2.c_str());
  }
  else
    mrpt::obs::stock_observations::exampleImage(img2, 1);

  mrpt::vision::CFeatureExtraction fext;

  CFeatureList feats1, feats2;

  mrpt::system::CTicTac tictac;

  cout << "Detecting features in image1...";
  tictac.Tic();
  fext.detectFeatures(img1, feats1, 0, nFeats);
  cout << tictac.Tac() * 1000 << " ms (" << feats1.size() << " features)\n";

  cout << "Detecting features in image2...";
  tictac.Tic();
  fext.detectFeatures(img2, feats2, 0, nFeats);
  cout << tictac.Tac() * 1000 << " ms (" << feats2.size() << " features)\n";

  {
    CImage img1_show = img1.colorImage();
    CImage img2_show = img2.colorImage();
    drawFeaturesOnImage(img1_show, feats1, TColor::blue());
    drawFeaturesOnImage(img2_show, feats2, TColor::blue());

    CDisplayWindow win1("Image1"), win2("Image2");
    win1.setPos(10, 10);
    win1.showImage(img1_show);
    win2.setPos(20 + img1.getWidth(), 10);
    win2.showImage(img2_show);

    cout << "Showing all the features" << endl;
    cout << "Press any key on windows 1 or the console to continue..." << endl;
    win1.waitForKey();
  }

  CDisplayWindowPlots winPlots("Distance between descriptors");
  winPlots.setPos(10, 70 + img1.getHeight());
  winPlots.resize(500, 200);

  // Show features distances:
  for (unsigned int i1 = 0; i1 < feats1.size() && winPlots.isOpen(); i1++)
  {
    // Compute distances:
    CVectorDouble distances(feats2.size());

    const auto& ft_i1 = feats1[i1];

    tictac.Tic();
    for (unsigned int i2 = 0; i2 < feats2.size(); i2++)
    {
      distances[i2] = ft_i1.descriptorDistanceTo(feats2[i2]);
    }
    cout << "All distances computed in " << 1000.0 * tictac.Tac() << " ms" << endl;

    // Show Distances:
    winPlots.plot(distances, ".4k", "all_dists");

    mrpt::math::matrix_index_t min_dist_idx = 0, max_dist_idx = 0;
    const double min_dist = distances.minCoeff(min_dist_idx);
    const double max_dist = distances.maxCoeff(max_dist_idx);

    const double dist_std = mrpt::math::stddev(distances);

    cout << "Min. distance=" << min_dist << " for img2 feat #" << min_dist_idx
         << " .Distances sigma: " << dist_std << endl;

    winPlots.axis(-15, distances.size(), -0.15 * max_dist, max_dist * 1.15);
    winPlots.plot(
        std::vector<double>({1.0, double(min_dist_idx)}), std::vector<double>({1.0, min_dist}),
        ".8b", "best_dists");

    winPlots.setWindowTitle(format("Distances feat #%u -> all others ", i1));

    // win2: Show only best matches:
    CImage img2_show_base = img2.colorImage();
    img2_show_base.selectTextFont("6x13");

    CVectorDouble xs_best, ys_best;
    for (unsigned int i2 = 0; i2 < feats2.size(); i2++)
    {
      const int px = feats2[i2].keypoint.pt.x;
      const int py = feats2[i2].keypoint.pt.y;
      if (distances[i2] < min_dist + 0.3 * dist_std)
      {
        img2_show_base.drawMark({px, py}, TColor::red(), '+', 7);
        img2_show_base.textOut(
            {px + 10, py - 10}, format("#%u, dist=%.02f", i2, distances[i2]), TColor::gray());
        xs_best.push_back(i2);
        ys_best.push_back(distances[i2]);
      }
      else
      {
        img2_show_base.drawMark({px, py}, TColor::gray(), '+', 3);
      }
    }

    winPlots.plot(xs_best, ys_best, ".4b", "best_dists2");

    // Show new images with animation:
    CImage img1_show, img2_show;
    for (unsigned anim_loops = 36; anim_loops > 0; anim_loops -= 2)
    {
      img1_show = img1.colorImage();
      img1_show.drawMark({ft_i1.keypoint.pt.x, ft_i1.keypoint.pt.y}, TColor::red(), '+', 7);
      img1_show.drawCircle(
          {ft_i1.keypoint.pt.x, ft_i1.keypoint.pt.y}, 7 + anim_loops, TColor::blue());

      img2_show = img2_show_base.makeDeepCopy();
      for (unsigned int i2 = 0; i2 < feats2.size(); i2++)
      {
        if (distances[i2] < min_dist + 0.1 * dist_std)
        {
          img2_show.drawCircle(
              {feats2[i2].keypoint.pt.x, feats2[i2].keypoint.pt.y}, 7 + anim_loops, TColor::blue());
        }
      }

      std::this_thread::sleep_for(10ms);
    }

    // Wait for the next iteration:
    cout << "Press any key on the distances window or on the console to "
            "continue (close any window to exit)..."
         << endl;
    winPlots.waitForKey();
  }

  return false;
}

int main(int argc, char** argv)
{
  try
  {
    if (argc != 1 && argc != 3)
    {
      cerr << "Usage: " << endl;
      cerr << argv[0] << endl;
      cerr << argv[0] << " <image1> <image2>" << endl;
      return 1;
    }

    if (argc == 3)
    {
      file1 = string(argv[1]);
      file2 = string(argv[2]);
    }

    DemoFeatures();
    return 0;
  }
  catch (exception& e)
  {
    cerr << mrpt::exception_to_str(e);
    return 1;
  }
}
