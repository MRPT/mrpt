#!/usr/bin/env python3

# ---------------------------------------------------------------------
# Install python3-pymrpt, ros-$ROS_DISTRO-python-mrpt,
# ros-$ROS_DISTRO-mrpt2, or test with a local build with:
# export PYTHONPATH=$HOME/code/mrpt/build-Release/:$PYTHONPATH
# ---------------------------------------------------------------------

# More matrix classes available in the module mrpt.math.
# See: https://mrpt.github.io/pymrpt-docs/mrpt.pymrpt.mrpt.math.html

from mrpt.pymrpt import mrpt
import numpy as np

# Create a numpy matrix from a list:
m1_np = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])

print('m1_np   :\n' + str(m1_np))
print()

# Create an MRPT matrix from a list:
m1_mrpt = mrpt.math.CMatrixDynamic_double_t(
    [[11, 12, 13], [14, 15, 16], [17, 18, 19]])

print('m1_mrpt :\n' + str(m1_mrpt))
print('m1_mrpt.size() :' + str(m1_mrpt.size()))
print()

# Convert an MRPT matrix to numpy via an intermediary list:
m2_mrpt = mrpt.math.CMatrixDynamic_double_t.Identity(3)
m2_data = m2_mrpt.to_list()
print('m2_data: {}'.format(m2_data))
m2_np = np.array(m2_data)
print('m2_np:\n{}'.format(m2_np))
print()

# Read/write access an MRPT matrix (0-based indices)
m2_mrpt[0, 2] = 99.0  # modify the entry (0,2)
print('m2 modified:\n{}'.format(m2_mrpt))
m2_mrpt[0, 2] = 99.0  # modify the entry (0,2)
print('m2[1,1]={}'.format(m2_mrpt[1, 1]))
print()

# Convert a numpy matrix to MRPT:
m3_np = np.array([[1, 2], [3, 4]])
m3_mrpt = mrpt.math.CMatrixDynamic_double_t(m3_np.tolist())
print('m3_np:\n{}'.format(m3_np))
print('m3_mrpt:\n{}'.format(m3_mrpt))
