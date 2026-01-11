from . import _bindings as _b

# Core Classes
Scene = _b.Scene
Viewport = _b.Viewport
CSetOfObjects = _b.CSetOfObjects
CCamera = _b.CCamera
CPointCloud = _b.CPointCloud
CAssimpModel = _b.CAssimpModel

# 1. Make Scene building intuitive with '<<' style (using __lshift__)


def _scene_lshift(self, obj):
    self.insert(obj)
    return self


Scene.__lshift__ = _scene_lshift
CSetOfObjects.__lshift__ = _scene_lshift
Viewport.__lshift__ = _scene_lshift

# 2. Add shorthand for creating colored points


def create_point_cloud(pts_array, color=(255, 255, 255)):
    pc = CPointCloud()
    pc.setPoints(pts_array)
    pc.setColor(*color)
    return pc


__all__ = ['Scene', 'Viewport', 'CSetOfObjects', 'CCamera',
           'CPointCloud', 'CAssimpModel', 'create_point_cloud']
