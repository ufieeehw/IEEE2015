__author__ = 'patrickemami'

import numpy as np

a = np.random.randn(3, 3)

c = np.matrix([10 - i for i in a])
d = np.matrix([12 - i for i in a])

print c * d

# Calculate the SVD of 3 x 3 matrix h
u, s, v = np.linalg.svd(a, full_matrices=True)

# Generate an orthonormal matrix x
x = u * v

x_det = np.linalg.det(x)

print x_det