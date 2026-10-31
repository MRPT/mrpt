\defgroup mrpt_imgui_grp [mrpt-imgui]

Dear ImGui application framework: app shell, 3D scene views, timeline, plots,
image views, log console and file dialogs.

[TOC]

# Library mrpt-imgui

This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-imgui-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

## Overview

`mrpt-imgui` is the basis of MRPT's graphical applications built on
[Dear ImGui](https://github.com/ocornut/imgui) (docking branch) and
[ImPlot](https://github.com/epezent/implot), both provided by the
`mrpt_imgui_vendor` package. It only holds generic widgets: it does not
depend on observation or map classes.

## The application shell pattern

An application is a window title, a set of panels and some callbacks. The
shell (mrpt::imgui::CImGuiAppShell) owns the window, the OpenGL context, and
the ImGui and ImPlot contexts, and every frame builds a full-window dockspace
whose central area is transparent, so that:

- A 3D world can be drawn as the window background, behind all the panels
  (mrpt::imgui::CImGuiBackgroundScene). This is the main view of viewers and
  simulators.
- Panels are regular ImGui windows: the user docks, tabs, floats and closes
  them, and the layout is saved to `imgui.ini` in the per-user config
  directory (mrpt::imgui::userConfigDirectory()) and restored on the next run.
  The application can provide a default layout for the first run
  (CImGuiAppShell::setupDefaultLayout).
- Bars that must not be covered by the dockspace (a toolbar, a timeline) are
  drawn from CImGuiAppShell::beforeDockspace, and the menus from
  CImGuiAppShell::mainMenuBar.

```cpp
mrpt::imgui::CImGuiAppShell app;
if (!app.init("My viewer")) { return 1; }

mrpt::imgui::CImGuiBackgroundScene world;
world.view.setScene(myScene);
mrpt::imgui::CImGuiTimeline timeline;
mrpt::imgui::CImGuiLogConsole console;

app.mainMenuBar = [&]() {
  if (ImGui::BeginMenu("File")) {
    if (ImGui::MenuItem("Quit")) { app.requestClose(); }
    ImGui::EndMenu();
  }
};
app.beforeDockspace = [&]() { timeline.renderAsBottomBar(); };
app.run([&]() {
  world.render(app);  // first: stays behind all panels
  ImGui::Begin("Console");
  console.render();
  ImGui::End();
});
```

The shell also merges the Material Symbols icon font into the default font
(`ICON_MS_*` macros, see mrpt/imgui/icons.h), scales the UI on HiDPI
monitors, and offers hooks for automated tests: the environment variables
`MRPT_IMGUI_MAX_FRAMES=N` (exit after N frames) and
`MRPT_IMGUI_SCREENSHOT=file.png` (save the last frame).

## Widgets

- mrpt::imgui::CImGuiSceneView: an mrpt::viz::Scene in a panel (or as the
  window background), with an orbit camera, mouse rays and click callbacks.
- mrpt::imgui::CImGuiTimeline: a time slider for log players, with
  play/pause/step, speed, per-sensor tick marks and keyboard shortcuts.
- mrpt::imgui::CImGuiPlotPanel: ImPlot charts of live or static series, with
  shaded bands, decimation, pause and CSV export.
- mrpt::imgui::CImGuiImageView: an mrpt::img::CImage with zoom, pan, pixel
  readout and save-as.
- mrpt::imgui::CImGuiLogConsole: messages of any number of
  mrpt::system::COutputLogger objects, with filters.
- mrpt::imgui::CImGuiFileDialog: native open/save/folder dialogs
  (portable-file-dialogs), with an ImGui fallback
  (mrpt::imgui::SimpleFileDialog) where no native one is available.
- mrpt::imgui::AsyncTask: runs long operations (e.g. loading files) on a
  background thread with progress and cancellation, so the GUI never freezes.
- mrpt/imgui/style.h: the MRPT style, and color conversions.

# Library contents
