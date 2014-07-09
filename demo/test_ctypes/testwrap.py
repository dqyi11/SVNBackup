import ctypes
from numpy.ctypeslib import ndpointer

lib = ctypes.CDLL('./library.so')
lib.function.restype = ndpointer(dtype=ctypes.c_int, shape=(10,))

res = lib.function()

