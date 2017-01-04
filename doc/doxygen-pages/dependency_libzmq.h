/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** \page dep-libzmq External dependency: ZeroMQ (libzmq)
 *

<small> <a href="dependencies.html">Back to list of dependencies</a> </small>
<br>

<h2>Dependency: <code>ZeroMQ</code></h2>
<hr>

ZeroMQ is not required to compile MRPT, but if the user application uses it,
then MRPT offers a few functions for sending and receiving MRPT objects via
ZMQ sockets.

See:
- Available functions in: \ref noncstream_serialization_zmq
- [Example code](https://github.com/MRPT/mrpt/tree/master/doc/mrpt-zeromq-example):
	- Publisher: [main_pub.cpp](https://github.com/MRPT/mrpt/blob/master/doc/mrpt-zeromq-example/main_pub.cpp)
	- Subscriber: [main_sub.cpp](https://github.com/MRPT/mrpt/blob/master/doc/mrpt-zeromq-example/main_sub.cpp)

*/
