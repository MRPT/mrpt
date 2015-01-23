#include "bindings.h"

/* MRPT */
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/COpenGLScene.h>

using namespace boost::python;

using namespace mrpt::gui;
using namespace mrpt::opengl;

// CDisplayWindow3D
COpenGLScene& CDisplayWindow3D_get3DSceneAndLock(CDisplayWindow3D &self)
{
    return *(self.get3DSceneAndLock());
}

void CDisplayWindow3D_update3DScene(CDisplayWindow3D &self, COpenGLScene& scene)
{
    COpenGLScenePtr current_scene = self.get3DSceneAndLock();
    current_scene = COpenGLScenePtr(&scene);
    self.unlockAccess3DScene();
    self.forceRepaint();
}
// end of CDisplayWindow3D

void export_gui()
{
    // map namespace to be submodule of mrpt package
    object gui_module(handle<>(borrowed(PyImport_AddModule("mrpt.gui"))));
    scope().attr("gui") = gui_module;
    scope gui_scope = gui_module;

    // CDisplayWindow3D
    {
        scope s = class_<CDisplayWindow3D>("CDisplayWindow3D", "A graphical user interface (GUI) for efficiently rendering 3D scenes in real-time.", init<optional<std::string, unsigned int, unsigned int> >(args("windowCaption", "initialWindowWidth", "initialWindowHeight")))
            .def("get3DSceneAndLock", &CDisplayWindow3D_get3DSceneAndLock, return_value_policy<reference_existing_object>(), "Gets a reference to the smart shared pointer that holds the internal scene (carefuly read introduction in gui::CDisplayWindow3D before use!) This also locks the critical section for accesing the scene, thus the window will not be repainted until it is unlocked.")
            .def("unlockAccess3DScene", &CDisplayWindow3D::unlockAccess3DScene, "Unlocks the access to the internal 3D scene.")
            .def("update3DScene", &CDisplayWindow3D_update3DScene, "Locks window, sets scene, unlocks access and repaints afterwards. Strongly recommended!")
            .def("forceRepaint", &CDisplayWindow3D::forceRepaint, "Repaints the window.")
            .def("repaint", &CDisplayWindow3D::repaint, "Repaints the window.")
            .def("updateWindow", &CDisplayWindow3D::updateWindow, "Repaints the window.")
            .def("setCameraElevationDeg", &CDisplayWindow3D::setCameraElevationDeg, args("deg"), "Changes the camera parameters programatically.")
            .def("setCameraAzimuthDeg", &CDisplayWindow3D::setCameraAzimuthDeg, args("deg"), "Changes the camera parameters programatically.")
            .def("setCameraPointingToPoint", &CDisplayWindow3D::setCameraPointingToPoint, args("x","y","z"), "Changes the camera parameters programatically.")
            .def("setCameraZoom", &CDisplayWindow3D::setCameraZoom, args("zoom"), "Changes the camera parameters programatically.")
            .def("setCameraProjective", &CDisplayWindow3D::setCameraProjective, args("isProjective"), "Sets the camera as projective, or orthogonal.")
            .def("isOpen", &CDisplayWindow3D::isOpen, "Returns false if the user has closed the window.")
        ;
    }
}
