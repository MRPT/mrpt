
======================================================
MRPT 3D viewport navigation: mouse and keyboard usage
======================================================

All MRPT applications use the following convention:

- **Orbit camera:** Left-button pressed + mouse move.

- **Zoom in / out:**

  - Mouse scroll wheel, or
  - SHIFT+Left-button pressed + mouse move up/down.

- **Look around** (pivot camera): CTRL+Left-button pressed + mouse move up/down.

- **Pan** (XY plane): Right-button pressed + mouse move.

- **Move camera along Z axis**: SHIFT+Left-button pressed + mouse move left/right,
  or (starting in MRPT 2.3.2) SHIFT+scroll wheel for faster up/down vertical motion.


The implementation of the features above by handling mouse and keyboard events
can be found in:

- `mrpt::gui::CGlCanvasBase <class_mrpt_gui_CGlCanvasBase.html>`_ for Qt and wxWidgets GUIs.

- `mrpt::gui::CGlCanvasBaseHeadless <class_mrpt_gui_CGlCanvasBaseHeadless.html>`_ for nanogui GUIs.
