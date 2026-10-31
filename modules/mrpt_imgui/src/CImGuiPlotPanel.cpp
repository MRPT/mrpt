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

#include <imgui.h>
#include <implot.h>
#include <mrpt/imgui/CImGuiPlotPanel.h>
#include <mrpt/imgui/icons.h>
#include <mrpt/imgui/style.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace mrpt::imgui
{
CImGuiPlotPanel::Series* CImGuiPlotPanel::series(const std::string& name)
{
  for (auto& s : m_series)
  {
    if (s.name == name)
    {
      return &s;
    }
  }
  return nullptr;
}

const CImGuiPlotPanel::Series* CImGuiPlotPanel::series(const std::string& name) const
{
  for (const auto& s : m_series)
  {
    if (s.name == name)
    {
      return &s;
    }
  }
  return nullptr;
}

CImGuiPlotPanel::Series& CImGuiPlotPanel::seriesOrCreate(const std::string& name)
{
  if (auto* s = series(name); s != nullptr)
  {
    return *s;
  }
  m_series.emplace_back();
  m_series.back().name = name;
  return m_series.back();
}

void CImGuiPlotPanel::trim(Series& s) const
{
  size_t drop = 0;
  if (s.x.size() > maxPoints)
  {
    drop = s.x.size() - maxPoints;
  }
  if (rollingSpan > 0 && !s.x.empty())
  {
    // Keep twice the visible span, so pausing still shows some history:
    const double cutoff = s.x.back() - 2 * rollingSpan;
    const auto it = std::lower_bound(s.x.begin(), s.x.end(), cutoff);
    drop = std::max(drop, static_cast<size_t>(std::distance(s.x.begin(), it)));
  }
  // Amortized: only erase when a sizable chunk accumulates.
  if (drop == 0 || (drop < 1024 && drop * 8 < s.x.size()))
  {
    return;
  }
  const auto n = static_cast<std::ptrdiff_t>(drop);
  s.x.erase(s.x.begin(), s.x.begin() + n);
  s.y.erase(s.y.begin(), s.y.begin() + n);
  if (s.bandLow.size() >= drop)
  {
    s.bandLow.erase(s.bandLow.begin(), s.bandLow.begin() + n);
  }
  if (s.bandHigh.size() >= drop)
  {
    s.bandHigh.erase(s.bandHigh.begin(), s.bandHigh.begin() + n);
  }
}

void CImGuiPlotPanel::addPoint(const std::string& name, double x, double y)
{
  auto& s = seriesOrCreate(name);
  s.x.push_back(x);
  s.y.push_back(y);
  trim(s);
}

void CImGuiPlotPanel::setSeries(
    const std::string& name, std::vector<double> x, std::vector<double> y)
{
  auto& s = seriesOrCreate(name);
  const size_t n = std::min(x.size(), y.size());
  x.resize(n);
  y.resize(n);
  s.x = std::move(x);
  s.y = std::move(y);
}

void CImGuiPlotPanel::setBand(
    const std::string& name,
    std::vector<double> x,
    std::vector<double> low,
    std::vector<double> high)
{
  auto& s = seriesOrCreate(name);
  if (!x.empty())
  {
    s.x = std::move(x);
    if (s.y.size() != s.x.size())
    {
      s.y.assign(s.x.size(), std::numeric_limits<double>::quiet_NaN());
    }
  }
  s.bandLow = std::move(low);
  s.bandHigh = std::move(high);
}

void CImGuiPlotPanel::clearData()
{
  for (auto& s : m_series)
  {
    s.x.clear();
    s.y.clear();
    s.bandLow.clear();
    s.bandHigh.clear();
  }
}

void CImGuiPlotPanel::removeSeries(const std::string& name)
{
  m_series.erase(
      std::remove_if(
          m_series.begin(), m_series.end(), [&](const Series& s) { return s.name == name; }),
      m_series.end());
}

std::string CImGuiPlotPanel::toCSV() const
{
  std::ostringstream ss;
  ss << std::setprecision(15);
  size_t maxLen = 0;
  for (size_t i = 0; i < m_series.size(); i++)
  {
    const auto& s = m_series[i];
    ss << (i ? "," : "") << s.name << "_x," << s.name << "_y";
    maxLen = std::max(maxLen, s.x.size());
  }
  ss << "\n";
  for (size_t r = 0; r < maxLen; r++)
  {
    for (size_t i = 0; i < m_series.size(); i++)
    {
      const auto& s = m_series[i];
      if (i)
      {
        ss << ",";
      }
      if (r < s.x.size())
      {
        ss << s.x[r] << "," << s.y[r];
      }
      else
      {
        ss << ",";
      }
    }
    ss << "\n";
  }
  return ss.str();
}

std::vector<size_t> CImGuiPlotPanel::decimateMinMax(
    const std::vector<double>& x,
    const std::vector<double>& y,
    double xMin,
    double xMax,
    size_t buckets)
{
  const size_t n = std::min(x.size(), y.size());
  std::vector<size_t> idx;
  if (n == 0)
  {
    return idx;
  }

  const auto itBegin =
      std::lower_bound(x.begin(), x.begin() + static_cast<std::ptrdiff_t>(n), xMin);
  const auto itEnd = std::upper_bound(itBegin, x.begin() + static_cast<std::ptrdiff_t>(n), xMax);
  size_t i0 = static_cast<size_t>(std::distance(x.begin(), itBegin));
  size_t i1 = static_cast<size_t>(std::distance(x.begin(), itEnd));
  // Include the neighbors just outside, so lines reach the plot edges:
  if (i0 > 0)
  {
    i0--;
  }
  if (i1 < n)
  {
    i1++;
  }

  if (buckets == 0 || i1 - i0 <= 2 * buckets || xMax <= xMin)
  {
    for (size_t i = i0; i < i1; i++)
    {
      idx.push_back(i);
    }
    return idx;
  }

  const double bw = (xMax - xMin) / static_cast<double>(buckets);
  const auto bucketOf = [&](double xv)
  { return std::min<size_t>(buckets - 1, static_cast<size_t>(std::floor((xv - xMin) / bw))); };
  const auto inRange = [&](double xv) { return xv >= xMin && xv <= xMax; };

  size_t i = i0;
  while (i < i1)
  {
    if (!inRange(x[i]))
    {
      idx.push_back(i);
      i++;
      continue;
    }
    const size_t b = bucketOf(x[i]);
    size_t iMin = i;
    size_t iMax = i;
    size_t j = i + 1;
    for (; j < i1 && inRange(x[j]) && bucketOf(x[j]) == b; j++)
    {
      if (y[j] < y[iMin])
      {
        iMin = j;
      }
      if (y[j] > y[iMax])
      {
        iMax = j;
      }
    }
    idx.push_back(std::min(iMin, iMax));
    if (iMin != iMax)
    {
      idx.push_back(std::max(iMin, iMax));
    }
    i = j;
  }
  return idx;
}

void CImGuiPlotPanel::render(const char* title, float height)
{
  ImGui::PushID(title);

  if (showToolbar)
  {
    if (ImGui::Button(paused ? ICON_MS_PLAY_ARROW : ICON_MS_PAUSE))
    {
      paused = !paused;
    }
    tooltip(paused ? "Resume" : "Pause, to inspect the data");
    ImGui::SameLine();
    ImGui::Checkbox("Autoscale", &autoscale);
    ImGui::SameLine();
    ImGui::Checkbox("Legend", &showLegend);
    ImGui::SameLine();
    if (ImGui::Button(ICON_MS_DOWNLOAD " CSV"))
    {
      m_csvDialog.open(
          CImGuiFileDialog::Mode::Save, "Export plot data",
          {
              {"CSV", {"csv"}}
      });
    }
    tooltip("Export the plot data to a CSV file");
  }
  if (auto r = m_csvDialog.poll(); r && !r->empty())
  {
    std::ofstream f(r->front());
    f << toCSV();
  }

  ImPlotFlags flags = ImPlotFlags_None;
  if (!showLegend)
  {
    flags |= ImPlotFlags_NoLegend;
  }
  if (equalAxes)
  {
    flags |= ImPlotFlags_Equal;
  }

  const float h = height > 0 ? height : std::max(50.0f, ImGui::GetContentRegionAvail().y);
  if (ImPlot::BeginPlot(title, ImVec2(-1, h), flags))
  {
    const ImPlotAxisFlags axFlags = (autoscale && !paused) ? ImPlotAxisFlags_AutoFit : 0;
    ImPlot::SetupAxes(
        xLabel.empty() ? nullptr : xLabel.c_str(), yLabel.empty() ? nullptr : yLabel.c_str(),
        rollingSpan > 0 ? 0 : axFlags, axFlags);

    // Rolling window: follow the newest data, unless paused.
    if (rollingSpan > 0)
    {
      double xMax = -std::numeric_limits<double>::infinity();
      for (const auto& s : m_series)
      {
        if (!s.x.empty())
        {
          xMax = std::max(xMax, s.x.back());
        }
      }
      if (!paused && std::isfinite(xMax))
      {
        m_pausedXMin = xMax - rollingSpan;
        m_pausedXMax = xMax;
        ImPlot::SetupAxisLimits(ImAxis_X1, m_pausedXMin, m_pausedXMax, ImGuiCond_Always);
      }
    }

    const ImPlotRect lim = ImPlot::GetPlotLimits();
    const auto buckets = static_cast<size_t>(std::max(16.0f, ImPlot::GetPlotSize().x));

    for (const auto& s : m_series)
    {
      if (!s.visible || s.x.empty())
      {
        continue;
      }
      ImPlotSpec spec;
      if (s.color)
      {
        spec.LineColor = toImVec4(*s.color);
        spec.FillColor = toImVec4(*s.color);
        spec.MarkerFillColor = toImVec4(*s.color);
        spec.MarkerLineColor = toImVec4(*s.color);
      }
      spec.LineWeight = s.lineWidth;

      const bool hasBand = s.bandLow.size() == s.x.size() && s.bandHigh.size() == s.x.size();
      // Decimation needs x sorted; for other data (e.g. 2D paths), draw all.
      const bool sorted = std::is_sorted(s.x.begin(), s.x.end());
      std::vector<double> xs;
      std::vector<double> ys;
      std::vector<double> lo;
      std::vector<double> hi;
      const double* px = s.x.data();
      const double* py = s.y.data();
      int count = static_cast<int>(std::min(s.x.size(), s.y.size()));
      if (sorted && !hasBand)
      {
        const auto idx = decimateMinMax(s.x, s.y, lim.X.Min, lim.X.Max, buckets);
        if (idx.size() < s.x.size())
        {
          xs.reserve(idx.size());
          ys.reserve(idx.size());
          for (const auto i : idx)
          {
            xs.push_back(s.x[i]);
            ys.push_back(s.y[i]);
          }
          px = xs.data();
          py = ys.data();
          count = static_cast<int>(xs.size());
        }
      }

      if (hasBand)
      {
        // Same label as the line: one legend entry and color for all.
        ImPlotSpec bandSpec = spec;
        bandSpec.FillAlpha = 0.3f;
        const int n = static_cast<int>(s.x.size());
        ImPlot::PlotShaded(
            s.name.c_str(), s.x.data(), s.bandLow.data(), s.bandHigh.data(), n, bandSpec);
        ImPlot::PlotLine(s.name.c_str(), s.x.data(), s.bandLow.data(), n, spec);
        ImPlot::PlotLine(s.name.c_str(), s.x.data(), s.bandHigh.data(), n, spec);
      }
      switch (s.style)
      {
        case Style::Scatter:
          spec.Marker = ImPlotMarker_Circle;
          spec.MarkerSize = 2.5f;
          ImPlot::PlotScatter(s.name.c_str(), px, py, count, spec);
          break;
        case Style::Stairs:
          ImPlot::PlotStairs(s.name.c_str(), px, py, count, spec);
          break;
        case Style::Line:
        default:
          ImPlot::PlotLine(s.name.c_str(), px, py, count, spec);
          break;
      }
    }

    if (onPlotOverlay)
    {
      onPlotOverlay();
    }
    ImPlot::EndPlot();
  }

  ImGui::PopID();
}

}  // namespace mrpt::imgui
