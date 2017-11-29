import pytest

import numpy as np

import taser
import taser.dummy as m

def test_vector_add():
    print(m)
    print(taser)
    a = np.array([1., 2, 3])
    b = np.array([2., 3, 1])
    c = m.vector_add(a, b)
    np.testing.assert_equal(c, [3, 5, 4])