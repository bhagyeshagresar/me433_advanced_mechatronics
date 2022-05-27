from math import sin
from ulab import numpy as np
import time
import random

array = np.empty([1, 1024])

for t in range(1024):
    array[0][t] = np.sin(t) + np.sin(3*t) + np.sin(6*t)


a, b = np.fft.fft(array[0])


for t in range(1024):
    print("hello")
    time.sleep(0.1)
    print((a[t], ))





