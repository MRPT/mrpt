#!/usr/bin/env python3

# More matrix classes available in mrpt.math.

from mrpt.math import CMatrixDouble
import numpy as np

# Create a numpy matrix from a list:
m1_np = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
print('m1_np   :\n' + str(m1_np))
print()

# Create an MRPT matrix from a list:
m1_mrpt = CMatrixDouble([[11, 12, 13], [14, 15, 16], [17, 18, 19]])
print('m1_mrpt :\n' + str(m1_mrpt))
print('m1_mrpt shape:', np.array(m1_mrpt.as_numpy()).shape)
print()

# Convert an MRPT matrix to numpy (CMatrixDouble has as_numpy but not to_list or Identity):
m2_mrpt = CMatrixDouble(np.eye(3).tolist())  # 3x3 identity
m2_np = np.array(m2_mrpt.as_numpy())
print('m2_np (identity):\n{}'.format(m2_np))
print()

# Read/write access: CMatrixDouble doesn't support __setitem__, use numpy round-trip
m2_arr = np.array(m2_mrpt.as_numpy())
m2_arr[0, 2] = 99.0  # modify the entry (0,2)
m2_mrpt = CMatrixDouble(m2_arr.tolist())
print('m2 modified:\n{}'.format(m2_mrpt))
print('m2[1,1]={}'.format(m2_arr[1, 1]))  # index via numpy array
print()

# Convert a numpy matrix to MRPT:
m3_np = np.array([[1, 2], [3, 4]])
m3_mrpt = CMatrixDouble(m3_np.tolist())
print('m3_np:\n{}'.format(m3_np))
print('m3_mrpt:\n{}'.format(m3_mrpt))
