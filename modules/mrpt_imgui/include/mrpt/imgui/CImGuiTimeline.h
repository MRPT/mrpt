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

#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace mrpt::imgui
{
/** A global time slider for log players: play/pause, step, speed, a
 * current-time readout, per-sensor tick marks along the axis, click-to-seek,
 * and keyboard shortcuts.
 *
 * The timeline covers [minTime(), maxTime()], in seconds or any other unit
 * (for logs without timestamps, use entry indices as times). If key times
 * are set (e.g. the timestamps of the log entries), stepping jumps between
 * them and currentKeyIndex() tells the entry at the current time.
 *
 * The model (time, stepping, playback) has no ImGui dependency and can be
 * driven programmatically; render() draws the widgets in the current window,
 * and renderAsBottomBar() pins them to the bottom of the main window.
 *
 * Keyboard shortcuts (if keyboardShortcuts is true, and no text is being
 * typed): Space play/pause; Left/Right step; PageUp/PageDown step 10;
 * Home/End jump to start/end.
 * \ingroup mrpt_imgui_grp
 */
class CImGuiTimeline
{
 public:
  /** A row of tick marks along the timeline, e.g. the observations of one
   * sensor. */
  struct Track
  {
    std::string name;
    mrpt::img::TColor color{0x80, 0xc0, 0xff, 0xff};
    std::vector<double> times;
    bool visible = true;
  };

  CImGuiTimeline() = default;

  /** @name Model
   * @{ */

  /** Sets the time range. The current time is clamped into it. */
  void setRange(double tMin, double tMax);
  [[nodiscard]] double minTime() const { return m_tMin; }
  [[nodiscard]] double maxTime() const { return m_tMax; }
  [[nodiscard]] double duration() const { return m_tMax - m_tMin; }

  /** Sets the discrete times stepping moves between (sorted on input). If
   * the range was empty, it is set to span them. */
  void setKeyTimes(std::vector<double> times);
  [[nodiscard]] const std::vector<double>& keyTimes() const { return m_keys; }

  /** Index of the last key time not after the current time (or the first
   * one, if the current time is before all of them); nullopt if there are no
   * key times. If several keys have the current time, the one reached with
   * seekToKey() or step(), if any. */
  [[nodiscard]] std::optional<size_t> currentKeyIndex() const;

  /** Moves to the given key time (index clamped). Calls onTimeChanged if
   * the time or the key changed (keys may share the same time). */
  void seekToKey(size_t index);

  /** Sets the current time, clamped to the range. Calls onTimeChanged if it
   * changed. */
  void setTime(double t);
  [[nodiscard]] double time() const { return m_t; }

  void play();
  void pause();
  void togglePlay();
  [[nodiscard]] bool isPlaying() const { return m_playing; }

  /** Moves `n` key times forward (or backward, if negative); without key
   * times, `n * stepSize` units. Pauses playback. */
  void step(int n);
  void jumpToStart();
  void jumpToEnd();

  /** Advances the time by `wallDt * speed` if playing, and returns true if
   * the time changed. At the end of the range playback stops, or restarts
   * from the beginning if `loop`. */
  bool update(double wallDt);

  /** Playback speed, as a multiple of real time. */
  double speed = 1.0;
  /** Restart from the beginning when the end is reached while playing. */
  bool loop = false;
  /** Step size (in time units) when there are no key times. */
  double stepSize = 0.1;

  /** Called whenever the current time changes. */
  std::function<void(double t)> onTimeChanged;

  /** @} */

  /** @name Tick tracks
   * @{ */
  void clearTracks() { m_tracks.clear(); }
  void addTrack(Track track) { m_tracks.emplace_back(std::move(track)); }
  [[nodiscard]] std::vector<Track>& tracks() { return m_tracks; }
  [[nodiscard]] const std::vector<Track>& tracks() const { return m_tracks; }

  /** Pixel columns, in [0, widthPx), with at least one of `times` within
   * [tMin, tMax] (sorted, unique), so dense tracks are drawn with at most
   * one tick per column. */
  [[nodiscard]] static std::vector<int> tickColumns(
      const std::vector<double>& times, double tMin, double tMax, int widthPx);
  /** @} */

  /** @name Display
   * @{ */

  /** Formats a time for the readout. Default: seconds since minTime(). */
  std::function<std::string(double t)> timeFormatter;

  /** The readout text of the current time, using timeFormatter. */
  [[nodiscard]] std::string timeLabel() const;

  bool keyboardShortcuts = true;

  /** Draws the timeline in the current ImGui window, handles its input, and
   * advances playback with the frame time. Returns true if the time changed
   * in this frame. */
  bool render();

  /** Draws the timeline in a bar pinned to the bottom of the main window.
   * Call it from CImGuiAppShell::beforeDockspace, so the dockspace does not
   * cover it. */
  bool renderAsBottomBar(const char* name = "##Timeline");

  /** Height of the bar drawn by renderAsBottomBar() (ImGui units). */
  [[nodiscard]] float barHeight() const;
  /** @} */

 private:
  double m_tMin = 0;
  double m_tMax = 0;
  double m_t = 0;
  bool m_playing = false;
  std::vector<double> m_keys;
  /** Key index set by seekToKey()/step(), to tell apart keys with the same
   * time. Ignored once the time moves away from that key. */
  std::optional<size_t> m_keyIdx;

  /** Moves to `t` (clamped), remembering `key`, and notifies changes. */
  void moveTo(double t, std::optional<size_t> key);
  std::vector<Track> m_tracks;

  void handleKeyboard();
};

}  // namespace mrpt::imgui
