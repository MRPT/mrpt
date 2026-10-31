\page app_SceneViewer3D Application: SceneViewer3D

# SceneViewer3D

A viewer of MRPT 3D scenes (`.3Dscene` files, as saved by
mrpt::viz::Scene::saveToFile()), 3D models in any format supported by Assimp,
PLY point clouds, and images.

    SceneViewer3D [file] [--load plugin1,plugin2] [--imgdir DIR]

- `file`: a `.3Dscene` file, or any 3D model (imported with mrpt::viz::CAssimpModel).
- `--load`: plugin libraries to load before reading the file (e.g. with custom
  visual object classes).
- `--imgdir`: base directory of externally-stored (lazy-load) images.

## User interface

The 3D scene is the background of the main window. On top of it, the
**View** panel (rendering options, camera, behavior with file sequences) and
the **Travelling** panel (automatic camera motions) can be docked anywhere,
tabbed, floated or closed (reopen them from the "View" menu). The layout is
remembered across sessions.

- **File**: new scene, open, reload, save; import 3D models, PLY point clouds
  (with options: class, intensity from coordinates, point size, rotation) or
  images; export the point clouds to PLY files; take a snapshot of the 3D view;
  high-resolution off-screen render to an image file; scene statistics.
- **Edit**: select objects by class, then re-scale or delete them.
- **Tools**: delete all objects; insert stock objects.
- **Toolbar**: previous/next file in the same directory; orthographic
  projection, shadows and SSAO toggles; **Autoplay** (load the next file
  periodically, to play sequences of scenes); **Capture** (save every frame of
  the 3D view to images in a directory, e.g. to make videos).
- Files can also be opened by dropping them onto the window, and the loaded
  file is reloaded automatically when it changes on disk.

Large files are loaded in the background, so the window stays responsive.

## Camera travelling

- **Circular**: orbits around a point between two azimuth offsets, at a given
  speed.
- **Camera path**: keyframes of the camera (looking-at point, distance,
  azimuth and elevation) with times, interpolated during playback. Grab the
  current camera as a keyframe, edit the values in the table, optionally
  re-time the path for a constant speed, and save or load it as a text file.

## Keyboard and mouse

| Input | Action |
|---|---|
| Left / Right, F7 / F8 | Previous / next `.3Dscene` file in the directory |
| F5 | Reload |
| F2 | Snapshot of the 3D view |
| Ctrl+N / Ctrl+O / Ctrl+S | New / open / save |
| F1 | About |
| Left drag | Orbit |
| Right or middle drag | Pan |
| Wheel | Zoom |
