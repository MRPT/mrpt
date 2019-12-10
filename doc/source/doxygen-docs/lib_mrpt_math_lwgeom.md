\defgroup geometry_grp Lightweight SE(2)/SE(3) types, geometry functions.
\ingroup mrpt_math_grp

Lightweight SE(2)/SE(3) data types, geometry functions, etc.

The "lightweight" name comes from the fact that these classes are simple
structures without special memory alignment requirements and do not have a deep
hiearchy of class heritance.

This is in contrast to classes derived from mrpt::poses::CPoseOrPoint, which in
turn offer:
 - Serialization (see: \ref mrpt_serialization_grp)
 - Buffered trigronometric calculations (e.g. mrpt::poses::CPose3D), hence they
will be preferred to lightweight alternaives (e.g. mrpt::math::TPose3D) if the
same pose is to be used over and over again to transform multiple points/poses.

See list of classes below.

