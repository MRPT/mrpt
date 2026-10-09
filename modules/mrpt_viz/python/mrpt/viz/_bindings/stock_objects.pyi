"""
Pre-built 3D objects
"""
from __future__ import annotations
import mrpt.viz
__all__: list[str] = ['BumblebeeCamera', 'CornerXYSimple', 'CornerXYZ', 'CornerXYZEye', 'CornerXYZSimple', 'Hokuyo_URG', 'Hokuyo_UTM', 'RobotGiraff', 'RobotPioneer', 'RobotRhodon']
def BumblebeeCamera() -> mrpt.viz.CSetOfObjects:
    """
    Returns a 3D model of a Bumblebee stereo camera.
    """
def CornerXYSimple(scale: float = 1.0, lineWidth: float = 1.0) -> mrpt.viz.CSetOfObjects:
    """
    Returns two lines for the X, Y axes of a 2D frame.
    """
def CornerXYZ(scale: float = 1.0) -> mrpt.viz.CSetOfObjects:
    """
    Returns three arrows for the X, Y, Z axes of a 3D frame.
    """
def CornerXYZEye() -> mrpt.viz.CSetOfObjects:
    """
    Returns three arrows for the X, Y, Z axes, with the Z arrowhead at the origin (to show a camera pose).
    """
def CornerXYZSimple(scale: float = 1.0, lineWidth: float = 1.0) -> mrpt.viz.CSetOfObjects:
    """
    Returns three lines for the X, Y, Z axes of a 3D frame (faster to render than CornerXYZ).
    """
def Hokuyo_URG() -> mrpt.viz.CSetOfObjects:
    """
    Returns a 3D model of a Hokuyo URG laser scanner.
    """
def Hokuyo_UTM() -> mrpt.viz.CSetOfObjects:
    """
    Returns a 3D model of a Hokuyo UTM laser scanner.
    """
def RobotGiraff() -> mrpt.viz.CSetOfObjects:
    """
    Returns a 3D model of the Giraff mobile robot.
    """
def RobotPioneer() -> mrpt.viz.CSetOfObjects:
    """
    Returns a 3D model of a Pioneer II mobile robot.
    """
def RobotRhodon() -> mrpt.viz.CSetOfObjects:
    """
    Returns a 3D model of the Rhodon mobile robot.
    """
