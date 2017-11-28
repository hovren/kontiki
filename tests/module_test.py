import pytest

import numpy as np

import taser.python_example as m

def test_vector_add():
    print(m)
    a = np.array([1., 2, 3])
    b = np.array([2., 3, 1])
    c = m.vector_add(a, b)
    np.testing.assert_equal(c, [3, 5, 4])