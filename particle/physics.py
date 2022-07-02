"""Particle physics system."""
from itertools import combinations, product

import numpy as np
from pyquaternion import Quaternion


class Particle:
    def __init__(self, pos, vel=np.zeros(3), mass=1.0, radius=1.0):
        self.mass = mass
        self.radius = radius

        self.pos = pos  # world position
        self.vel = vel

        self.r0 = np.zeros(3)  # relative position
        self.r = np.zeros(3)  # rotated relative position

    def apply_force(self, force, dt):
        """Only used on free particles."""
        # dv = a * dt = F / m * dt
        self.vel += force / self.mass * dt
        assert self.vel[2] == 0.0


class RigidBody:
    def __init__(self, particles, pos, vel, w, q=np.array([1.0, 0.0, 0.0, 0.0])):
        self.particles = particles

        self.mass = sum([p.mass for p in particles])
        self.intertia = RigidBody._compute_inertia(particles)

        self.p = pos
        self.q = q  # quaternion / orientation

        self.v = vel
        self.w = w  # angular velocity

        self.F = np.zeros(3)
        self.T = np.zeros(3)

        self.update_particles(dt=0.0)

    def _compute_inertia(particles):
        return np.identity(3)

    def apply_force(self, dt):

        # dv/dt = F / m
        # Velocity update: dv = a * dt = F / m * dt

        self.v += self.F / self.mass * dt
        self.F = np.zeros(3)

    def apply_torque(self, dt):

        # dL/dt = T = r x F, Iw = L
        # Angular velocity update: dw = I_inv * T * dt

        # (i) rotate inertia and (ii) update angular velocity

        R = self.q.rotation_matrix
        I_inv = R @ np.linalg.inv(self.inertia) @ R

        self.w += I_inv @ self.T * dt
        self.T = np.zeros(3)

    def update_particles(self, dt):

        # (i) update position and orientation of self
        # (ii) update positions and velocities of all particles

        self.p += self.v * dt
        self.q = update_quaternion(self.q, self.w * dt)

        R = self.q.rotation_matrix
        for p in self.particles:
            p.r = R @ p.r0
            p.pos = self.p + p.r
            # p.vel = self.v + np.cross(p.r, self.w)


def make_rigid_body(particles, vel, w):
    mass = sum(p.mass for p in particles)
    center = 1.0 / mass * np.sum([p.pos * p.mass for p in particles], axis=0)
    for p in particles:
        p.r0 = p.pos - center

    # compute inertia?

    return RigidBody(particles, pos=center, vel=vel, w=w)


class Surface:
    def __init__(self, pos, norm):
        self.pos = np.r_[pos, 0.0]
        self.norm = np.r_[norm, 0.0]
        self.v = pos @ norm


class Collision:
    def __init__(
        self, p1: Particle, p2: Particle, b1: RigidBody = None, b2: RigidBody = None
    ):
        self.p1 = p1
        self.p2 = p2
        self.b1 = b1
        self.b2 = b2

    def force(self):
        K, C, T = 80.0, 5.0, 1.0

        r = self.p1.pos - self.p2.pos
        v = self.p1.vel - self.p2.vel
        r_norm = np.linalg.norm(r)
        r_unit = r / r_norm

        # penetration force
        fs = -K * (self.p1.radius + self.p2.radius - r_norm) * r_unit
        # damping force
        fd = C * v
        # shear (tangential) force
        ft = T * (v - (v @ r_unit) * r_unit)

        return fs + fd + ft


class SurfaceCollision:
    def __init__(self, p: Particle, s: Surface):
        self.p = p
        self.s = s

    def force(self):
        K = 1000

        # penetration force
        fs = -K * (self.s.v - self.p.pos @ self.s.norm) * self.s.norm

        return fs


def solve_collisions(collisions):
    """Increment forces and torques of colliding bodies."""
    for c in collision:
        f1 = -c.force()
        f2 = c.force()

        c.b1.F += f1
        c.b2.F += f2

        c.b1.T += np.cross(c.p1.r, f1)
        c.b2.T += np.cross(c.p2.r, f2)


def _is_colliding(p1: Particle, p2: Particle):
    """Particle to particle collision detection."""
    return np.linalg.norm(p1.pos - p2.pos) < p1.radius + p2.radius


def find_particle_collisions(particles):
    """Brute-force collision detection."""
    return [
        Collision(p1, p2)
        for p1, p2 in combinations(particles, 2)
        if _is_colliding(p1, p2)
    ]


def find_rigid_body_collisions(bodies):
    """Brute-force collision detection."""
    return [
        Collision(p1, p2, b1, b2)
        for b1, b2 in combinations(bodies, 2)
        for p1, p2 in product(b1.particles, b2.particles)
        if _is_colliding(p1, p2)
    ]


def find_surface_collisions(particles, surfaces):
    def _is_colliding_surface(p, s):
        return p.pos @ s.norm < s.v

    return [
        SurfaceCollision(p, s)
        for p, s in product(particles, surfaces)
        if _is_colliding_surface(p, s)
    ]


def process(particles, surfaces, dt):
    # particle to particle collisions
    collisions = find_particle_collisions(particles)
    for c in collisions:
        f = c.force()
        c.p1.apply_force(-f, dt)
        c.p2.apply_force(f, dt)

    # particle to surface collisions
    surface_collisions = find_surface_collisions(particles, surfaces)
    for c in surface_collisions:
        f = c.force()
        c.p.apply_force(-f, dt)


def process2(rigid_bodies, dt):
    # body-to-body collisions
    solve_collisions(find_rigid_body_collisions(rigid_bodies))

    for rigid_body in rigid_bodies:
        rigid_body.apply_force(dt)
        rigid_body.apply_torque(dt)
        rigid_body.update_particles(dt)


def update_quaternion(q0: Quaternion, w):
    dq = _to_update_quat(w)
    q1 = dq * q0
    return q1.unit


def _to_update_quat(w):
    w_len = np.linalg.norm(w)
    if w_len < 1e-10:
        return Quaternion(np.array([1.0, 0.0, 0.0, 0.0]))

    theta = 0.5 * w_len
    a = np.cos(theta)
    v = np.sin(theta) * w / w_len

    return Quaternion(np.r_[a, v])


def test_collision():
    p1 = Particle(np.array([0.0, 0.0]), np.array([1.0, 1.0]))
    p2 = Particle(np.array([0.5, 0.5]), -np.array([1.0, 1.0]))
    c = Collision(p1, p2)
    print(c.force())


def test_quaternion():
    p1 = np.array([1.0, 1.0, 0.0])

    q0 = Quaternion(np.array([1.0, 0.0, 0.0, 0.0]))
    dq = _to_update_quat(np.array([0.0, 0.0, np.pi]))

    q1 = q0 * dq
    q2 = dq * q0

    print(f"q0 * dq: {q1.rotate(p1)}")
    print(f"dq * q0: {q2.rotate(p1)}")
    print(f"q1: {q1}, q2: {q2.unit}")


def test_rigid_body():
    positions = [
        np.array([100.0, 110.0, 0.0]),
        np.array([120.0, 110.0, 0.0]),
        np.array([110.0, 100.0, 0.0]),
        np.array([110.0, 120.0, 0.0]),
    ]

    vel = np.zeros(3)
    angular_velocity = np.zeros(3)

    particles = [Particle(pos=pos, vel=vel, radius=10.0) for pos in positions]
    body = make_rigid_body(particles, vel=velocity, w=angular_velocity)
    print(body)


if __name__ == "__main__":
    test_quaternion()
    test_rigid_body()
