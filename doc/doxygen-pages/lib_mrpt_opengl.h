/** \page mrpt-opengl Library overview: mrpt-opengl
 *

<small> <a href="index.html#libs">Back to list of libraries</a> </small>
<br>

<h2>mrpt-opengl</h2>
<hr>


This library includes several data classes that represent objects that can be 
inserted into a 3D scene, which can be then rendered or streamed to disk or whatever.

A good starting point to explore this library is the base class for all the 
3D objects: mrpt::opengl::CRenderizable

A 3D scene is represented by an object of the type mrpt::opengl::COpenGLScene, 
which in turn can contain one or several "viewports" in such a way that the 
rendering area is divided into several spaces, each displaying the same or different
objects. See the tutorial online: http://www.mrpt.org/Tutorial_3D_Scenes


See the full list of classes in mrpt::opengl.


*/

