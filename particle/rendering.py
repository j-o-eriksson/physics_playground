"""The Pong Game."""
import time
import random

import sdl2
import sdl2.ext
import numpy as np

from physics import (
        Particle,
        Surface,
        RigidBody,
        process,
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
        vel = particle.vel
        sz = int(2 * particle.radius)

        sprite = factory.from_color(random.choice(COLORS), size=(sz, sz))
        sprite.position = int(pos[0]), int(pos[1])

        self.player = Player(world, sprite, int(pos[0]), int(pos[1]))
        self.p = particle

    def move(self, dt):
        self.p.pos += self.p.vel * dt
        px, py = self.p.pos[:2].round().astype(int)
        self.player.sprite.position = px, py


def make_example_body():
    positions = [
            np.array([100., 130.]),
            np.array([160., 130.]),
            np.array([130., 100.]),
            np.array([130., 160.]),
            ]
    particles = [Particle(pos=np.r_[pos, 0.], radius=10.) for pos in positions]

    return make_rigid_body(particles,
            vel=np.array([10., 0., 0.]),
            w=np.array([0., 0., 1.]))


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
            for _ in range(50)]

    rb = make_example_body()
    particles = rb.particles

    particles = [Particle(np.r_[pos, 0.], np.r_[vel, 0.], radius=10.)
            for pos, vel in init_data]
    particle_sprites = [ParticleSprite(world, factory, p) for p in particles]

    s1 = Surface(pos=np.array([0.0, 0.0]), norm=np.array([1.0, 0.0]))  # left
    s2 = Surface(pos=np.array([0.0, 0.0]), norm=np.array([0.0, 1.0]))  # top
    s3 = Surface(pos=np.array([780.0, 0.0]), norm=np.array([-1.0, 0.0]))  # right
    s4 = Surface(pos=np.array([0.0, 580.0]), norm=np.array([0.0, -1.0]))  # bottom
    surfaces = [s1, s2, s3, s4]

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

        # update particles
        rb.update_particles(dt)
        process(particles, surfaces, dt)
        for ps in particle_sprites:
            ps.move(dt)

        world.process()

        sdl2.SDL_Delay(10)


if __name__ == '__main__':
    run()
