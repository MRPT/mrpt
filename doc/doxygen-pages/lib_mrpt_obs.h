/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

/** \defgroup mrpt_obs_grp [mrpt-obs]

<small> <a href="index.html#libs">Back to list of all libraries</a> | <a href="modules.html" >See all modules</a> </small>
<br>

<h2>Library <code>mrpt-obs</code></h2>
<hr>

In this library there are <b>five</b> key elements or groups of elements:

<ul>

<li><b>Sensor observations:</b> All sensor observations share a common virtual base
class (mrpt::slam::CObservation). There are classes to store laser scanners, 3D range images,
monocular and stereo images, GPS data, odometry, etc. A concept very related to observations
is a mrpt::slam::CSensoryFrame, a set of observations which were collected approximately at the same instant. </li>

<li><b>Rawlogs (datasets):</b> A robotics dataset can be loaded, edited and explored by means 
of the class mrpt::slam::CRawlog. See also: http://www.mrpt.org/Rawlog_Format </li>

<li><b>Actions:</b> For convenience in many Bayesian filtering algorithms, robot actions 
(like 2D displacement characterized by an odometry increment) can be represented by means
of "actions". See mrpt::slam::CAction.</li>

<li><b>"Simple maps":</b> In MRPT, a "simple map" is a set of pairs: "position", "sensory frames" (read above).
The advantage of maintaining such a "simple map" instead a metric map is that the metric maps 
can be rebuilt when needed with different parameters from the raw observations, which are never lost. </li>

<li><b>CARMEN logs tools:</b> Utilities to read from CARMEN log files and load the observations there
as MRPT observations. See mrpt::slam::carmen_log_parse_line and the applications: carmen2rawlog, carmen2simplemap. </li>

</ul>




*/

