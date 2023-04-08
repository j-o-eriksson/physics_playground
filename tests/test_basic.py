import numpy as np

import phycpp as pc
import phyplay.particle.physics as pp


def _get_particles():
    positions = [
        np.array([-20, 10, 0]),
        np.array([-40, 10, 0]),
        np.array([20, 10, 0]),
        np.array([40, -10, 0]),
        np.array([40, -30, 0]),
    ]
    return [pp.Particle(pos=pos, radius=10.0, mass=1.0) for pos in positions]


def test_particle_conversions():
    particles = _get_particles()
    particles_phycpp = [p.to_phycpp() for p in particles]
    particles2 = [pp.Particle.from_phycpp(p) for p in particles_phycpp]

    assert all(p1 == p2 for p1, p2 in zip(particles, particles2))


def test_rigid_bodies():
    particles = _get_particles()
    v = np.array([30.0, 0.0, 0.0])
    w = np.array([0.0, 0.0, 1.0])

    body1 = pp.make_rigid_body(particles, v, w)
    body2 = body1.to_phycpp()

    dt = 0.1
    for _ in range(10):
        body1.update_particles(dt)
        body2.update(dt)

    assert body1.particles == pp.RigidBody.from_phycpp(body2).particles


def test_phycpp():
    print()
    cp = pc.CollisionParams(50.0, 10.0, 2.0)
    assert cp.k == 50.0

    pps = [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)]

    v1 = pc.Vec3(1.0, 0.0, 0.0)
    particles_pc = [pc.Particle(position=pc.Vec3(x, y, 0.0)) for x, y in pps]
    body1 = pc.make_rigid_body(particles_pc, vel=v1)
    print(body1)

    v2 = np.array([1.0, 0.0, 0.0])
    particles_pp = [pp.Particle(np.array([x, y, 0.0])) for x, y in pps]
    body2 = pp.make_rigid_body(particles_pp, vel=v2, w=np.zeros(3))
    print(body2.mass, body2.p, body2.inertia)

    assert body1.mass == body2.mass
    assert body1.position.x == body2.p[0]
    assert body1.velocity.x == body2.v[0]
    assert body1.angular_velocity.z == body2.w[2]


# def test_collisions():
#     print()
#     pps = [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)]
#     ms = [4.0, 3.0, 2.0, 2.5]
#     particles = [
#         pc.Particle(position=pc.Vec3(x, y, 0.0), mass=m) for (x, y), m in zip(pps, ms)
#     ]
#     body1 = pc.make_rigid_body(particles, vel=pc.Vec3(3.0, 0.0, 0.0))
#     body2 = pc.make_rigid_body(particles, vel=pc.Vec3(0.0, 0.0, 0.0))
#     p0 = body1.particles[0]
#     print(p0.vel)
#
#     bodies = [body1, body2]
#     for b in bodies:
#         b.update(1.0)
#     print(p0.vel)
#     print()
#
#     for p in body1.particles:
#         print(p.pos)
#         # print(p.r0)
#     print()
#
#     for p in body2.particles:
#         print(p.pos)
#         # print(p.r0)
#     print()
#
#     collisions = pc.find_collisions(bodies)
#     for c in collisions:
#         print("collision:")
#         print(c.p1)
#         print(c.p2)
#         print()
