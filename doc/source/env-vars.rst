.. _env_vars:

====================================================
Environment variables that MRPT looks for
====================================================

The following environment variables may be set to alter some MRPT functionality.
Note that for Boolean flags, the strings ``true``, ``True``, or any nonzero
numerical value (e.g. ``1``) will be interpreted as "true".

- ``MRPT_BACKTRACE_PRINT_ADDRESS``: Set to ``1`` to show PC (program pointer)
  addresses in mrpt::TCallStackBackTrace::asString() (e.g. in exception backtraces).

- ``MRPT_BACKTRACE_DISABLE_BFD``: Set to ``1`` to disable using BFD while
  generating stack backtraces in
  `mrpt::callStackBackTrace() <https://docs.mrpt.org/reference/latest/group_mrpt_core_grp.html?#doxid-group-mrpt-core-grp-1ga14092e6931d0d4ac17bfdf39d2a2ce94>`_,
  falling back to ``dladdr()``.

- ``MRPT_CCAMERA_KINECT_TIMEOUT_MS``: Timeout in milliseconds to wait
  for a new frame from a Kinect sensor when using the method
  mrpt::hwdrivers::CCamera::getNextFrame(). Default: ``3000``.

- ``MRPT_DEBUG_IMG_LAZY_LOAD``: Shows `std::cout` traces whenever an mrpt::img::CImage with delay-load content is created, loaded, or destructed.

- ``MRPT_DEBUG_OBSIMG_LAZY_LOAD``: Shows `std::cout` traces whenever an mrpt::obs::CObservationImage with delay-load content is loaded or unloaded.

- ``MRPT_EXPR_VERBOSE``: Set to ``1`` to enable extra verbose debug traces for
  `mrpt::expr::CRuntimeCompiledExpression::eval() <class_mrpt_expr_CRuntimeCompiledExpression.html>`_.

- ``MRPT_HWDRIVERS_DEFAULT_LEAP_SECONDS``: In mrpt::hwdrivers::CGPSInterface, the
  default offset from GPS time to UTC time, for use before any GNSS especific
  message informs on this offset. Default: 17 (valid for 01/2016)

- ``MRPT_HWDRIVERS_VERBOSE``: Set to ``1`` to enable verbose output to ``std::cout``
  from all mrpt::hwdrivers classes. Default: ``0``.

- ``MRPT_REVEAL_OPENGL_BUFFER_LEAKS``: If enabled, prints warnings whenever an
  OpenGL resource is destroyed from a thread different to that one on which it 
  was created. Trying to destroy such a resource would lead to a SEGFAULT, so
  the destroy is silently skipped. Since this might lead to memory and resource
  leaks, this variable helps debugging wrong application designs where resources
  are not created and destroyed from the main OpenGL thread.

- ``MRPT_WXSUBSYS_TIMEOUT_MS``: The timeout in milliseconds to wait
  in mrpt::gui windows constructors until the wxWidgets thread processes and
  really creates the window, before reporting an error. It's also used in
  the function mrpt::hwdrivers::prepareVideoSourceFromUserSelection().
  Default: ``5000`` in Release builds, ``30000`` in Debug builds.

- ``MRPT_YAML_PARSER_VERBOSE``: Set to ``1`` to show verbose debug information
  about YAML document parsing in
  `mrpt::containers::yaml <class_mrpt_containers_yaml.html>`_.
