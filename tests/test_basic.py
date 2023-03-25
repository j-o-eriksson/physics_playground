import numpy as np
from pyquaternion import Quaternion

import phycpp as pc
import phyplay.particle.physics as pp


def test_phycpp():
    assert pc.add(3, 5) == 8
    assert pc.subtract(3, 5) == -2
    assert pc.multiply(3, 5) == 75
    assert round(pc.camera(0), 1) == 0.5


def test_quaternion():
    p1 = np.array([1.0, 1.0, 0.0])

    q0 = Quaternion(np.array([1.0, 0.0, 0.0, 0.0]))
    dq = pp._to_update_quat(np.array([0.0, 0.0, np.pi]))

    q1 = q0 * dq
    q2 = dq * q0

    print(f"q0 * dq: {q1.rotate(p1)}")
    print(f"dq * q0: {q2.rotate(p1)}")
    print(f"q1: {q1}, q2: {q2.unit}")

    assert True
