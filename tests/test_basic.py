import numpy as np
from pyquaternion import Quaternion

import phycpp as pc
import phyplay.particle.physics as pp


def test_phycpp():
    assert pc.add(3, 5) == 8
    assert pc.subtract(3, 5) == -2
    assert pc.multiply(3, 5) == 16
#    assert round(pc.camera(0), 1) == 0.5


def test_quaternion():
    print()
    w = np.array([0.0, 0.0, np.pi])

    q0 = Quaternion(np.array([1.0, 0.0, 0.0, 0.0]))
    dq = pp._to_update_quat(w)

    q1 = q0 * dq
    q2 = dq * q0
    q3 = q0.rotate(w)

    assert q1 == q2
    print(f"q0 * dq: {q1}")
    print(f"dq * q0: {q2}")
    print(f"q0.rotate(w): {q3}")

    assert True
