.. _tutorial-ransac:

===================================================================
RANSAC C++ examples
===================================================================

1. RANSAC algorithm
----------------------

MRPT comprises a generic, template-based C++ implementation of this
robust model fit algorithm, useful for outliers rejection.
For a theoretical description of the algorithm, refer to 
`this Wikipedia article <https://en.wikipedia.org/wiki/Random_sample_consensus>`_
and the cites herein.
See also `this excellent MATLAB toolkit <https://www.peterkovesi.com/matlabfns/>`_
by Peter Kovesi, on which MRPT implementation is strongly based.

2. C++ API
----------------------

The base C++ API for RANSAC in MRPT
is `mrpt::math::RANSAC_Template <class_mrpt_math_RANSAC_Template.html>`_,
while some specialized classes exist for particular instances of common problems, 
e.g. fit a plane or a line to a point cloud, as shown below.

A simple genetic-like modification of RANSAC is also available 
through the template class `mrpt::math::ModelSearch <class_mrpt_math_ModelSearch.html>`_.

3. Particular applications
----------------------------


3.1. Fit a 3D plane
~~~~~~~~~~~~~~~~~~~~~

3.2. Fit many 3D planes
~~~~~~~~~~~~~~~~~~~~~~~~~~

3.3. Fit many 2D lines
~~~~~~~~~~~~~~~~~~~~~~~~~~


3.4. Data association with RANSAC
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

`This example <page_maps_ransac_data_association.html>`_ illustrates how RANSAC
can be used to establish the pairings (the "data association" problem)
between a set of 2D noisy observations and another set of 2D predictions from a map.
For example it could be used to match planar range-bearing landmarks against a 2D map, 
or a subset of image keypoints against a larger image mosaic.

This method was discussed in our paper :cite:`blanco2013robust`.


.. image:: maps_ransac_data_association_screenshot.png
	:alt: MRPT RANSAC data association screenshot

