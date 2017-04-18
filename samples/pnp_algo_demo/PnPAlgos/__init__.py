# Provide type-checking in Python where it is nice and easy.
import numpy

# First step, import the raw classes.
from _PnPAlgos import PnPAlgos


def _typecheck(a):
    assert (type(a) == numpy.ndarray) or (type(a) == numpy.core.memmap), 'Input should be a numpy array or memmap object!'
    assert a.dtype == numpy.dtype('float64'), 'Array needs to be 64 bit floating point!'
    #assert a.flags['F_CONTIGUOUS'], 'Array needs to have column major storage!'
    assert a.flags['ALIGNED'], 'Array needs to be aligned!'

def _typecheck_output(a):
    _typecheck(a)
    assert a.flags['WRITEABLE'] # The inputs might not be writeable.

def _typecheckify(unsafeFunction):
    # Take an unsafe verion of a function, and return a typesafe version.
    
    def safeFunction(self, obj_pts, img_pts, n, cam_intrinsic, pose_mat):
        map(_typecheck, [obj_pts])  # Add input variables to this list
        map(_typecheck, [img_pts]) # Add output variables to this list
        map(_typecheck, [cam_intrinsic]) # Add output variables to this list
        map(_typecheck, [pose_mat]) # Add output variables to this list
               
        
        # Make sure same number of object and image points are passed in
        assert obj_pts.shape[0] == img_pts.shape[0]
        
        # Make the unsafe call
        return unsafeFunction(self, obj_pts, img_pts, n, cam_intrinsic, pose_mat)
        
    return safeFunction
PnPAlgos.pnpalgo1 = _typecheckify(PnPAlgos.pnpalgo1)

# Note: This machinery is absolutely overkill for this simple example, but can be handy if
# you are wrapping multiple functions that share an interface containing lots of variables.

