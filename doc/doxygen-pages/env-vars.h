/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */

/** \page env-vars Environment variables that MRPT looks for
 *

The following environment variables may be set to alter some MRPT functionality:

- `MRPT_EXPR_VERBOSE`: Set to `1` to enable extra verbose debug traces for
  mrpt::math::CRuntimeCompiledExpression::eval().

- `MRPT_CCAMERA_KINECT_TIMEOUT_MS`: Timeout in milliseconds to wait
   for a new frame from a Kinect sensor when using the method
   mrpt::hwdrivers::CCamera::getNextFrame(). Default: "3000".

- `MRPT_HWDRIVERS_DEFAULT_LEAP_SECONDS`: In mrpt::hwdrivers::CGPSInterface, the
    default offset from GPS time to UTC time, for use before any GNSS especific
    message informs on this offset. Default: 17 (valid for 01/2016)

- `MRPT_HWDRIVERS_VERBOSE`: Set to "1" to enable verbose output to cout
   from all mrpt::hwdrivers classes. Default: "0".

- `MRPT_WXSUBSYS_TIMEOUT_MS`: The timeout in milliseconds to wait
  in mrpt::gui windows constructors until the wxWidgets thread processes and
  really creates the window, before reporting an error. It's also used in
  the function mrpt::hwdrivers::prepareVideoSourceFromUserSelection().
  Default: "5000" in Release builds, "30000" in Debug.

*/
