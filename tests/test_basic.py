import numpy as np

import phyplay.particle.physics as pp


def _is_same(b1, b2):
    return b1.particles == pp.RigidBody.from_phycpp(b2).particles


def _make_example_particles(pos=np.zeros(3)):
    positions = [
        pos + np.array([-20, 10, 0]),
        pos + np.array([-40, 10, 0]),
        pos + np.array([20, 10, 0]),
        pos + np.array([40, -10, 0]),
        pos + np.array([40, -30, 0]),
    ]
    return [pp.Particle(pos=pos, radius=10.0, mass=1.0) for pos in positions]


def _make_example_body(pos: float, vel: float, w: float):
    return pp.make_rigid_body(
        _make_example_particles(np.array([pos, 0.0, 0.0])),
        np.array([vel, 0.0, 0.0]),
        np.array([0.0, 0.0, w]),
    )


def test_particle_conversions():
    particles_orig = _make_example_particles()
    particles_phycpp = [p.to_phycpp() for p in particles_orig]
    particles_converted = [pp.Particle.from_phycpp(p) for p in particles_phycpp]

    assert all(p1 == p2 for p1, p2 in zip(particles_orig, particles_converted))


def test_rigid_body_collisions():
    bodies = [_make_example_body(0.0, 10.0, 1.0), _make_example_body(30.0, 0.0, 0.0)]
    bodies_cpp = [b.to_phycpp() for b in bodies]
    for b1, b2 in zip(bodies, bodies_cpp):
        b1.update(0.0)
        b2.update(0.0)

    dt = 0.1
    for i in range(60):
        pp.process2(bodies, [], dt)
        pp.process3(bodies_cpp, [], dt)

        for b1, b2 in zip(bodies, bodies_cpp):
            assert _is_same(b1, b2)
