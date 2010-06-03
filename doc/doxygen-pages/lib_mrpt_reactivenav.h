/** \page mrpt-reactivenav Library overview: mrpt-reactivenav
 *

<small> <a href="index.html#libs">Back to list of libraries</a> </small>
<br>

<h2>mrpt-reactivenav</h2>
<hr>

This library implements:

<ul>

<li>Holonomic navigation algorithms:  
   Virtual Force Fields (VFF), Nearness Diagram (ND), ... 
   See mrpt::reactivenav::CAbstractHolonomicReactiveMethod </li>


<li>A complex reactive navigator: Using space transformations (PTGs) to drive
  a robot using an internal simpler holonomic algorithm. See mrpt::reactivenav::CReactiveNavigationSystem </li>

<li>A number of different PTGs: See mrpt::reactivenav::CParameterizedTrajectoryGenerator</li>

</ul>

See the full list of classes in mrpt::reactivenav, or the online
page http://www.mrpt.org/Application:ReactiveNavigationDemo for a discussion
of a working application (see MRPT/apps/ReactiveNavigationDemo).


*/

