\defgroup geometry_grp Lightweight SE(2)/SE(3) types, geometry functions.
\ingroup mrpt_math_grp

Lightweight SE(2)/SE(3) data types, geometry functions, etc.

The "lightweight" adjective is used here in contrast to classes derived
from mrpt::poses::CPoseOrPoint. 
The "lightweight" alternative types here, defined in mrpt::math, are simple
C++ structures without special memory alignment requirements and without 
a deep hiearchy of class inheritance, as the "heavier" classes in mrpt::poses have.
In turn, the latter ones offer:
 - Serialization (see: \ref mrpt_serialization_grp)
 - Buffered trigronometric calculations (e.g. mrpt::poses::CPose3D), hence they
will be preferred to lightweight alternaives (e.g. mrpt::math::TPose3D) if the
same pose is to be used over and over again to transform multiple points/poses.

See list of classes below.

