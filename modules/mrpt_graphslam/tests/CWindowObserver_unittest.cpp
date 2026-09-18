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

/** Unit tests for CWindowObserver. No GUI is needed: the window events are
 *  plain value types, so a minimal CObservable stand-in can publish them
 *  directly to the observer.
 */

#include <gtest/gtest.h>
#include <mrpt/graphslam/misc/CWindowObserver.h>
#include <mrpt/gui/CBaseGUIWindow.h>

#include <map>
#include <string>

using mrpt::graphslam::CWindowObserver;

namespace
{
/** Minimal event source: exposes publishEvent() so tests can inject any
 *  mrptEvent without creating a real window.
 */
class EventSource : public mrpt::system::CObservable
{
 public:
  void emit(const mrpt::system::mrptEvent& e) { this->publishEvent(e); }
};

/** Convenience: subscribe, publish one event, and return the resulting flags */
std::map<std::string, bool> emitAndRead(CWindowObserver& obs, const mrpt::system::mrptEvent& e)
{
  EventSource src;
  obs.observeBegin(src);
  src.emit(e);
  obs.observeEnd(src);

  std::map<std::string, bool> flags;
  obs.returnEventsStruct(&flags);
  return flags;
}
}  // namespace

TEST(CWindowObserver, default_keystrokes_are_registered_and_unpressed)
{
  CWindowObserver obs;

  std::map<std::string, bool> flags;
  obs.returnEventsStruct(&flags);

  EXPECT_EQ(flags.size(), 4U);
  for (const auto& [key, pressed] : flags)
  {
    EXPECT_FALSE(pressed) << "key: " << key;
  }
  EXPECT_EQ(flags.count("h"), 1U);
  EXPECT_EQ(flags.count("Alt+Enter"), 1U);
  EXPECT_EQ(flags.count("Ctrl+c"), 1U);
  EXPECT_EQ(flags.count("mouse_clicked"), 1U);
}

TEST(CWindowObserver, registerKeystroke_adds_a_new_flag)
{
  CWindowObserver obs;
  obs.registerKeystroke("s", "Save the current graph");

  std::map<std::string, bool> flags;
  obs.returnEventsStruct(&flags);

  ASSERT_EQ(flags.count("s"), 1U);
  EXPECT_FALSE(flags.at("s"));
  EXPECT_EQ(flags.size(), 5U);
}

TEST(CWindowObserver, char_event_lower_and_upper_case_h)
{
  for (const int code : {'h', 'H'})
  {
    CWindowObserver obs;
    const mrpt::gui::mrptEventWindowChar ev(nullptr, code, mrpt::gui::MRPTKMOD_NONE);
    const auto flags = emitAndRead(obs, ev);
    EXPECT_TRUE(flags.at("h")) << "char code: " << code;
    EXPECT_FALSE(flags.at("Ctrl+c"));
  }
}

TEST(CWindowObserver, char_event_ctrl_c_needs_the_control_modifier)
{
  // Plain 'c' must not raise the Ctrl+c flag:
  {
    CWindowObserver obs;
    const mrpt::gui::mrptEventWindowChar ev(nullptr, 'c', mrpt::gui::MRPTKMOD_NONE);
    const auto flags = emitAndRead(obs, ev);
    EXPECT_FALSE(flags.at("Ctrl+c"));
    // ...and it must not fall through to the "any other key" branch either:
    EXPECT_EQ(flags.count("c"), 0U);
  }
  // With the control modifier it must, alone or combined with others:
  const mrpt::gui::mrptKeyModifier mods[] = {
      mrpt::gui::MRPTKMOD_CONTROL, static_cast<mrpt::gui::mrptKeyModifier>(
                                       mrpt::gui::MRPTKMOD_CONTROL | mrpt::gui::MRPTKMOD_SHIFT)};
  for (const int code : {'c', 'C'})
  {
    for (const auto mod : mods)
    {
      CWindowObserver obs;
      const mrpt::gui::mrptEventWindowChar ev(nullptr, code, mod);
      const auto flags = emitAndRead(obs, ev);
      EXPECT_TRUE(flags.at("Ctrl+c")) << "char code: " << code << " modifiers: " << mod;
    }
  }
}

TEST(CWindowObserver, char_event_other_keys_are_stored_in_lower_case)
{
  CWindowObserver obs;
  const mrpt::gui::mrptEventWindowChar ev(nullptr, 'S', mrpt::gui::MRPTKMOD_NONE);
  const auto flags = emitAndRead(obs, ev);

  ASSERT_EQ(flags.count("s"), 1U);
  EXPECT_TRUE(flags.at("s"));
  EXPECT_EQ(flags.count("S"), 0U);
}

TEST(CWindowObserver, mouse_down_event_raises_the_mouse_clicked_flag)
{
  CWindowObserver obs;
  const mrpt::gui::mrptEventMouseDown ev(nullptr, mrpt::img::TPixelCoord(10, 20), true, false);
  const auto flags = emitAndRead(obs, ev);
  EXPECT_TRUE(flags.at("mouse_clicked"));
}

TEST(CWindowObserver, events_without_an_associated_flag_change_nothing)
{
  CWindowObserver obs;
  EventSource src;
  obs.observeBegin(src);

  src.emit(mrpt::gui::mrptEventWindowResize(nullptr, 640, 480));
  src.emit(mrpt::gui::mrptEventWindowClosed(nullptr, true));
  src.emit(mrpt::gui::mrptEventMouseMove(nullptr, mrpt::img::TPixelCoord(1, 2), false, false));

  std::map<std::string, bool> flags;
  obs.returnEventsStruct(&flags);
  for (const auto& [key, pressed] : flags)
  {
    EXPECT_FALSE(pressed) << "key: " << key;
  }

  obs.observeEnd(src);
}

TEST(CWindowObserver, returnEventsStruct_resets_flags_unless_asked_not_to)
{
  CWindowObserver obs;
  EventSource src;
  obs.observeBegin(src);
  src.emit(mrpt::gui::mrptEventWindowChar(nullptr, 'h', mrpt::gui::MRPTKMOD_NONE));

  std::map<std::string, bool> flags;

  // Peek without resetting: the flag stays raised for the next query.
  obs.returnEventsStruct(&flags, false /*reset_keypresses*/);
  EXPECT_TRUE(flags.at("h"));

  obs.returnEventsStruct(&flags);
  EXPECT_TRUE(flags.at("h"));

  // Now it has been consumed:
  obs.returnEventsStruct(&flags);
  EXPECT_FALSE(flags.at("h"));

  obs.observeEnd(src);
}

TEST(CWindowObserver, observer_survives_the_destruction_of_the_observed_object)
{
  CWindowObserver obs;
  {
    EventSource src;
    obs.observeBegin(src);
    src.emit(mrpt::gui::mrptEventWindowChar(nullptr, 'h', mrpt::gui::MRPTKMOD_NONE));
  }
  std::map<std::string, bool> flags;
  obs.returnEventsStruct(&flags);
  EXPECT_TRUE(flags.at("h"));
}
