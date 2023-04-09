"""Particle physics system."""
from itertools import combinations, product
from typing import List, Optional

import numpy as np
from pyquaternion import Quaternion

import phycpp


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

    def __eq__(p1, p2) -> bool:
        return (
            np.allclose(p1.pos, p2.pos, rtol=1e-3)
            and np.allclose(p1.vel, p2.vel, rtol=1e-3)
            and p1.mass == p2.mass
            and p1.radius == p2.radius
        )

    def __repr__(self):
        return f"(pos: {np.round(self.pos, 2)}, vel: {np.round(self.vel, 2)})"

    def to_phycpp(self) -> phycpp.Particle:
        return phycpp.Particle(
            _to_vec3(self.pos), _to_vec3(self.vel), self.mass, self.radius
        )

    @staticmethod
    def from_phycpp(p: phycpp.Particle):
        return Particle(_from_vec3(p.pos), _from_vec3(p.vel), p.mass, p.radius)


class RigidBody:
    def __init__(
        self, particles, pos, vel, w, q=Quaternion(np.array([1.0, 0.0, 0.0, 0.0]))
    ):
        self.particles = particles

        self.mass = sum([p.mass for p in particles])
        self.inertia = RigidBody._compute_inertia(particles)

        self.p = pos
        self.q = q  # quaternion / orientation

        self.v = vel
        self.w = w  # angular velocity

        self.F = np.zeros(3)
        self.T = np.zeros(3)

        self.update(dt=0.0)

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

    def update(self, dt):
        # (i) update position and orientation of self
        # (ii) update positions and velocities of all particles

        self.p += self.v * dt
        self.q = update_quaternion(self.q, self.w * dt)

        R = self.q.rotation_matrix
        for p in self.particles:
            p.r = R @ p.r0
            p.pos = self.p + p.r
            p.vel = self.v + np.cross(self.w, p.r)

    @staticmethod
    def _compute_inertia(particles: List[Particle]) -> np.ndarray:
        Ixx = sum(p.mass * (p.r0[1] ** 2 + p.r0[2] ** 2) for p in particles)
        Iyy = sum(p.mass * (p.r0[0] ** 2 + p.r0[2] ** 2) for p in particles)
        Izz = sum(p.mass * (p.r0[0] ** 2 + p.r0[1] ** 2) for p in particles)

        Ixy = -sum(p.mass * p.r0[0] * p.r0[1] for p in particles)
        Ixz = -sum(p.mass * p.r0[0] * p.r0[2] for p in particles)
        Iyz = -sum(p.mass * p.r0[1] * p.r0[2] for p in particles)

        return np.array(
            [
                [Ixx, Ixy, Ixz],
                [Ixy, Iyy, Iyz],
                [Ixz, Iyz, Izz],
            ]
        )

    def to_phycpp(self) -> phycpp.RigidBody:
        # set orientation?
        ps = [p.to_phycpp() for p in self.particles]
        return phycpp.make_rigid_body(ps, _to_vec3(self.v), _to_vec3(self.w))

    @staticmethod
    def from_phycpp(body: phycpp.RigidBody):
        # set orientation?
        ps = [Particle.from_phycpp(p) for p in body.particles]
        return make_rigid_body(
            ps, _from_vec3(body.velocity), _from_vec3(body.angular_velocity)
        )


def make_rigid_body(particles, vel, w) -> RigidBody:
    mass = sum(p.mass for p in particles)
    center = 1.0 / mass * np.sum([p.pos * p.mass for p in particles], axis=0)
    for p in particles:
        p.r0 = p.pos - center

    return RigidBody(particles, pos=center, vel=vel, w=w)


class Surface:
    def __init__(self, pos, norm):
        self.pos = np.r_[pos, 0.0]
        self.norm = np.r_[norm, 0.0]
        self.v = pos @ norm


class Collision:
    def __init__(
        self,
        p1: Particle,
        p2: Particle,
        b1: Optional[RigidBody] = None,
        b2: Optional[RigidBody] = None,
    ):
        self.p1 = p1
        self.p2 = p2
        self.b1 = b1
        self.b2 = b2

    def force(self):
        K, C, T = 120.0, 5.0, 1.0

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
        K, C, T = 1000.0, 5.0, 1.0
        v = self.p.vel
        r_unit = -self.s.norm

        # penetration force
        fs = -K * (self.p.pos @ self.s.norm - self.s.v) * r_unit
        # damping force
        fd = C * v
        # shear (tangential) force
        ft = T * (v - (v @ r_unit) * r_unit)

        return fs + fd + ft


def solve_collisions(collisions):
    """Increment forces and torques of colliding bodies."""
    for c in collisions:
        f1 = -c.force()
        f2 = c.force()

        c.b1.F += f1
        c.b2.F += f2

        c.b1.T += np.cross(c.p1.r, f1)
        c.b2.T += np.cross(c.p2.r, f2)


def solve_surface_collisions(collisions):
    for c, body in collisions:
        f = -c.force()
        body.F += f
        body.T += np.cross(c.p.r, f)


def _is_colliding(p1: Particle, p2: Particle):
    """Particle-to-particle collision detection."""
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


def find_surface_collisions(body_particles, surfaces):
    def _is_colliding_surface(p, s):
        return p.pos @ s.norm < s.v

    return [
        (SurfaceCollision(p, surf), body)
        for (p, body), surf in product(body_particles, surfaces)
        if _is_colliding_surface(p, surf)
    ]


def process(particles, surfaces, dt):
    # particle-to-particle collisions
    collisions = find_particle_collisions(particles)
    for c in collisions:
        f = c.force()
        c.p1.apply_force(-f, dt)
        c.p2.apply_force(f, dt)

    # particle to surface collisions
    surface_collisions = find_surface_collisions(particles, surfaces)
    for c, _ in surface_collisions:
        f = c.force()
        c.p.apply_force(-f, dt)


def process2(rigid_bodies, surfaces, dt):
    # body-to-body collisions
    solve_collisions(find_rigid_body_collisions(rigid_bodies))

    # surface collisions
    ps = [(p, body) for body in rigid_bodies for p in body.particles]
    solve_surface_collisions(find_surface_collisions(ps, surfaces))

    for rigid_body in rigid_bodies:
        rigid_body.apply_force(dt)
        rigid_body.apply_torque(dt)
        rigid_body.update(dt)


def process3(rigid_bodies: phycpp.RigidBody, planes: phycpp.Plane, dt: float):
    collisions = phycpp.find_collisions(rigid_bodies)
    params1 = phycpp.CollisionParams(120.0, 5.0, 1.0)
    for c in collisions:
        c.resolve(params1)

    plane_collisions = phycpp.find_plane_collisions(rigid_bodies, planes)
    params2 = phycpp.CollisionParams(1000.0, 5.0, 1.0)
    for c in plane_collisions:
        c.resolve(params2)

    for body in rigid_bodies:
        body.apply_force(dt)
        body.apply_torque(dt)
        body.update(dt)


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


def _to_vec3(a: np.ndarray) -> phycpp.Vec3:
    return phycpp.Vec3(a[0], a[1], a[2])


def _from_vec3(a: phycpp.Vec3) -> np.ndarray:
    return np.array([a.x, a.y, a.z])


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

    velocity = np.zeros(3)
    angular_velocity = np.zeros(3)

    particles = [Particle(pos=pos, vel=velocity, radius=10.0) for pos in positions]
    body = make_rigid_body(particles, vel=velocity, w=angular_velocity)
    print([p.r0 for p in body.particles])
    print(body.inertia)

    # compute inertia?
    particles = [
        Particle(pos=pos, vel=velocity, radius=10.0)
        for pos in [np.array([-1, 1, 1]), np.array([1, -1, -1])]
    ]
    body2 = make_rigid_body(particles, vel=velocity, w=angular_velocity)
    print(body2.inertia)


if __name__ == "__main__":
    test_quaternion()
    test_rigid_body()
