import math
import numpy as np
import time

t1 = time.time()
for i in xrange(1000000):
    math.sqrt(10**2 + 11 ** 2)
t2 = time.time()
print t2 - t1
t1 = time.time()
for i in xrange(1000000):
    np.linalg.norm(np.array([10, 11]))
t2 = time.time()
print t2 - t1
