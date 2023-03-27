"""The Pong Game."""
import time
import random

import sdl2
import sdl2.ext
import numpy as np

import phycpp
from phyplay.particle.physics import (
    Particle,
    Surface,
    process,
    process2,
    process3,
    make_rigid_body,
)


YELLOW = sdl2.ext.Color(255, 255, 0)
RED = sdl2.ext.Color(255, 0, 0)
GREEN = sdl2.ext.Color(0, 255, 0)
BLUE = sdl2.ext.Color(0, 0, 255)
PURPLE = sdl2.ext.Color(255, 0, 255)
CYAN = sdl2.ext.Color(0, 255, 255)
COLORS = [RED, GREEN, BLUE, YELLOW, PURPLE, CYAN]


class SoftwareRenderer(sdl2.ext.SoftwareSpriteRenderSystem):
    def __init__(self, window):
        super(SoftwareRenderer, self).__init__(window)

    def render(self, components):
        sdl2.ext.fill(self.surface, sdl2.ext.Color(0, 0, 0))
        super(SoftwareRenderer, self).render(components)


class Player(sdl2.ext.Entity):
    def __init__(self, world, sprite, posx=0, posy=0):
        self.sprite = sprite
        self.sprite.position = posx, posy


class ParticleSprite:
    def __init__(self, world, factory, particle):
        pos = particle.pos[:2]
        sz = int(2 * particle.radius)

        sprite = factory.from_color(random.choice(COLORS), size=(sz, sz))
        sprite.position = int(pos[0]), int(pos[1])

        self.player = Player(world, sprite, int(pos[0]), int(pos[1]))
        self.p = particle

    def move(self):
        px, py = self.p.pos[:2].round().astype(int)
        self.player.sprite.position = px, py


class ParticleSprite2:
    def __init__(self, world, factory, particle):
        pos = particle.pos
        sz = int(2 * particle.radius)

        sprite = factory.from_color(random.choice(COLORS), size=(sz, sz))
        sprite.position = int(pos.x), int(pos.y)

        self.player = Player(world, sprite, int(pos.x), int(pos.y))
        self.p = particle

    def move(self):
        px = round(self.p.pos.x)
        py = round(self.p.pos.y)
        self.player.sprite.position = px, py


def make_example_body(pos, vel, wz, mass=1.0):
    positions = [
        pos + np.array([-20, 10, 0]),
        pos + np.array([-20, -10, 0]),
        pos + np.array([-40, 10, 0]),
        pos + np.array([-40, -10, 0]),
        pos + np.array([20, 10, 0]),
        pos + np.array([20, -10, 0]),
        pos + np.array([40, 10, 0]),
        pos + np.array([40, -10, 0]),
        pos + np.array([40, 30, 0]),
        pos + np.array([40, -30, 0]),
    ]
    particles = [Particle(pos=pos, radius=10.0, mass=mass) for pos in positions]

    return make_rigid_body(particles, vel=vel, w=np.array([0.0, 0.0, wz]))


def make_example_body2(pos, vel, wz, mass=1.0):
    def _vec3(v):
        return phycpp.Vec3(v[0], v[1], v[2])

    positions = [
        pos + np.array([-40, 10, 0]),
        pos + np.array([-40, -10, 0]),
        pos + np.array([-20, 10, 0]),
        pos + np.array([-20, -10, 0]),
        pos + np.array([0, 10, 0]),
        pos + np.array([0, -10, 0]),
    ]
    particles = [
        phycpp.Particle(position=_vec3(pos), radius=10.0, mass=mass)
        for pos in positions
    ]
    return phycpp.make_rigid_body(
        particles, vel=_vec3(vel), w=phycpp.Vec3(0.0, 0.0, wz)
    )


def run():
    sdl2.ext.init()
    window = sdl2.ext.Window("The Pong Game", size=(800, 600))
    window.show()

    world = sdl2.ext.World()

    spriterenderer = SoftwareRenderer(window)
    world.add_system(spriterenderer)
    factory = sdl2.ext.SpriteFactory(sdl2.ext.SOFTWARE)

    init_data = [
        (np.random.uniform(0, 550, 2), np.random.uniform(-300, 300, 2))
        for _ in range(50)
    ]

    rb1 = make_example_body2(
        pos=np.array([100, 300.0, 0]), vel=np.array([200.0, 0.0, 0.0]), wz=4.0, mass=1.0
    )
    rb2 = make_example_body2(
        pos=np.array([500, 300, 0]), vel=np.array([00.0, 0.0, 0.0]), wz=0.0, mass=1.0
    )
    rbs = [rb1, rb2]

    # particles = [
    #     Particle(np.r_[pos, 0.0], np.r_[vel, 0.0], radius=10.0)
    #     for pos, vel in init_data
    # ]
    particle_sprites = [
        ParticleSprite2(world, factory, p) for rb in rbs for p in rb.particles
    ]

    # s1 = Surface(pos=np.array([0.0, 0.0]), norm=np.array([1.0, 0.0]))  # left
    # s2 = Surface(pos=np.array([0.0, 0.0]), norm=np.array([0.0, 1.0]))  # top
    # s3 = Surface(pos=np.array([780.0, 0.0]), norm=np.array([-1.0, 0.0]))  # right
    # s4 = Surface(pos=np.array([0.0, 580.0]), norm=np.array([0.0, -1.0]))  # bottom
    # surfaces = [s1, s2, s3, s4]

    planes = _make_planes()

    prev_time = time.time()
    running = True
    while running:
        events = sdl2.ext.get_events()
        for event in events:
            if event.type == sdl2.SDL_QUIT:
                running = False
                break

        # compute time delta
        curr_time = time.time()
        dt = curr_time - prev_time
        prev_time = curr_time

        # process(particles, surfaces, dt)
        # process2(rbs, surfaces, dt)
        process3(rbs, planes, dt)

        # render
        for ps in particle_sprites:
            ps.move()
        world.process()

        sdl2.SDL_Delay(10)


def _make_planes():
    def _vec3(a):
        return phycpp.Vec3(a[0], a[1], a[2])

    def _surface(pos, norm):
        return phycpp.Plane(_vec3(pos), _vec3(norm), pos @ norm)

    poss = [(0.0, 0.0), (0.0, 0.0), (780.0, 0.0), (0.0, 580.0)]
    norms = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]
    return [_surface(np.r_[p, 0.0], np.r_[n, 0.0]) for p, n in zip(poss, norms)]


if __name__ == "__main__":
    run()
