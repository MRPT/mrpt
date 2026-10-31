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

#include <mrpt/img/TColor.h>
#include <mrpt/imgui/CImGuiFileDialog.h>

#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace mrpt::imgui
{
/** A 2D plot (ImPlot) with named data series, for live or static data.
 *
 * Series can be appended to point by point (addPoint(), e.g. a live signal,
 * optionally shown over a rolling time window), or replaced as a whole
 * (setSeries()). Series may carry a shaded band (e.g. a confidence
 * interval). Long series are decimated for drawing, preserving their
 * extremes. A toolbar offers pause (to inspect), autoscale, and exporting the
 * data to CSV.
 *
 * \code
 *   mrpt::imgui::CImGuiPlotPanel plot;
 *   plot.rollingSpan = 10.0;  // show the last 10 s
 *   plot.addPoint("speed", t, v);
 *   // In a window:
 *   plot.render("Speed");
 * \endcode
 * \ingroup mrpt_imgui_grp
 */
class CImGuiPlotPanel
{
 public:
  enum class Style
  {
    Line,
    Scatter,
    Stairs
  };

  struct Series
  {
    std::string name;
    std::vector<double> x;
    std::vector<double> y;
    /** Optional band (same length as x): drawn as a shaded area. */
    std::vector<double> bandLow;
    std::vector<double> bandHigh;
    std::optional<mrpt::img::TColor> color;
    Style style = Style::Line;
    float lineWidth = 1.0f;
    bool visible = true;
  };

  CImGuiPlotPanel() = default;

  /** @name Data
   * @{ */

  /** Appends a point to a series (created if needed). Old points are
   * dropped beyond maxPoints, and beyond twice rollingSpan, if set. */
  void addPoint(const std::string& series, double x, double y);

  /** Replaces the data of a series (created if needed). */
  void setSeries(const std::string& series, std::vector<double> x, std::vector<double> y);

  /** Sets a shaded band [low, high] along a series (created if needed). If
   * `x` is given, it also replaces the series x values. */
  void setBand(
      const std::string& series,
      std::vector<double> x,
      std::vector<double> low,
      std::vector<double> high);

  /** The series with that name, or nullptr. */
  [[nodiscard]] Series* series(const std::string& name);
  [[nodiscard]] const Series* series(const std::string& name) const;
  /** The series with that name, created (empty) if it does not exist. */
  Series& seriesOrCreate(const std::string& name);

  [[nodiscard]] const std::vector<Series>& allSeries() const { return m_series; }

  void clear() { m_series.clear(); }
  void clearData();
  void removeSeries(const std::string& name);

  /** Maximum points kept per series appended with addPoint(). */
  size_t maxPoints = 200000;

  /** If > 0, the x axis follows the last `rollingSpan` units of data. */
  double rollingSpan = 0;
  /** @} */

  /** @name Display
   * @{ */
  std::string xLabel;
  std::string yLabel;
  bool showLegend = true;
  bool autoscale = true;
  /** Freezes the view (data keeps being appended). */
  bool paused = false;
  /** Same scale in both axes (e.g. for 2D maps). */
  bool equalAxes = false;
  bool showToolbar = true;

  /** Called inside the plot (between ImPlot::BeginPlot()/EndPlot()) to draw
   * extra items, e.g. ellipses or annotations. */
  std::function<void()> onPlotOverlay;

  /** Draws the toolbar and the plot in the current window. A height <= 0
   * uses the available height. */
  void render(const char* title, float height = 0);
  /** @} */

  /** @name Utilities (exposed for testing)
   * @{ */

  /** Data as CSV: one `x,y` column pair per series, with a header row. */
  [[nodiscard]] std::string toCSV() const;

  /** Indices of the points of (x,y) (x sorted ascending) to draw, at most
   * about 2*buckets: for each of `buckets` equal x intervals within
   * [xMin, xMax], its minimum and maximum y; plus the points just outside the
   * interval, so lines reach the plot edges. All indices, if few enough. */
  [[nodiscard]] static std::vector<size_t> decimateMinMax(
      const std::vector<double>& x,
      const std::vector<double>& y,
      double xMin,
      double xMax,
      size_t buckets);
  /** @} */

 private:
  std::vector<Series> m_series;
  CImGuiFileDialog m_csvDialog;
  double m_pausedXMin = 0;
  double m_pausedXMax = 1;

  void trim(Series& s) const;
};

}  // namespace mrpt::imgui
