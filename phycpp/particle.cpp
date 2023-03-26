#include "particle.hpp"

#include <numeric>

using glm::mat3;
using glm::vec3;

namespace phycpp {

void RigidBody::apply_force(float dt) {
  // dv/dt = F / m
  // Velocity update: dv = a * dt = F / m * dt
  v += force / mass * dt;
  force = vec3(0.f);
}

void RigidBody::apply_torque(float dt) {
  // dL/dt = T = r x F, Iw = L
  // Angular velocity update: dw = I_inv * T * dt

  // (i) rotate inertia and (ii) update angular velocity
  const mat3 R = glm::mat3_cast(q);
  const mat3 I_inv = R * glm::inverse(I) * R;

  w += I_inv * torque * dt;
  torque = vec3(0.f);
}

void RigidBody::update(float dt) {
  // (i) update position and orientation of self
  p += v * dt;
  q = glm::normalize(pertubation_quat(w * dt) * q);

  // (ii) update positions and velocities of all particles
  const auto R = glm::mat3_cast(q);
  for (auto& particle : particles) {
    particle.r = R * particle.r0;
    particle.pos = p + particle.r;
    particle.vel = v + glm::cross(w, particle.r);
  }
}

void Collision::resolve(const CollisionParams& params) {
  const vec3 f = compute_force(p1, p2, params);
  const vec3 f1 = -f;
  const vec3 f2 = f;

  b1->force += f1;
  b2->force += f2;

  b1->torque += glm::cross(p1.r, f1);
  b2->torque += glm::cross(p2.r, f2);
}

RigidBody make_rigid_body(std::vector<Particle> particles,
                          const vec3& vel,
                          const vec3& w) {
  const float mass =
      std::accumulate(particles.begin(), particles.end(), 0.f,
                      [](float m, const auto& p) { return m + p.mass; });

  const vec3 center =
      1.f / mass *
      std::accumulate(
          particles.begin(), particles.end(), vec3(0.f),
          [](const auto& v, const auto& p) { return v + p.mass * p.pos; });

  for (auto& p : particles)
    p.r0 = p.pos - center;

  const mat3 I = compute_inertia_matrix(particles);

  return RigidBody{particles, mass, I, center, vel, w};
}

mat3 compute_inertia_matrix(const std::vector<Particle>& particles) {
  auto f1 = [&particles](size_t i, size_t j) {
    float out = 0.f;
    for (const auto& p : particles)
      out += p.mass * (p.r0[i] * p.r0[i] + p.r0[j] * p.r0[j]);
    return out;
  };

  auto f2 = [&particles](size_t i, size_t j) {
    float out = 0.f;
    for (const auto& p : particles)
      out += p.mass * (p.r0[i] * p.r0[j]);
    return out;
  };

  float Ixx = f1(1, 2);
  float Iyy = f1(0, 2);
  float Izz = f1(0, 1);

  float Ixy = -f2(0, 1);
  float Ixz = -f2(0, 2);
  float Iyz = -f2(1, 2);

  return mat3{
      Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz,
  };
}

vec3 compute_force(const Particle& p1,
                   const Particle& p2,
                   const CollisionParams& params) {
  const vec3 r = p1.pos - p2.pos;
  const vec3 v = p1.vel - p2.vel;
  const float r_norm = glm::length(r);
  const vec3 r_unit = r / r_norm;

  const vec3 fs = params.k * (r_norm - p1.radius - p2.radius) * r_unit;
  const vec3 fd = params.c * v;
  const vec3 ft = params.t * (v - (glm::dot(v, r_unit)) * r_unit);

  return fs + fd + ft;
}

bool are_colliding(const Particle& p1, const Particle& p2) {
  return glm::length(p1.pos - p2.pos) < p1.radius + p2.radius;
}

void append_collisions(const std::vector<Particle>& ps1,
                       const std::vector<Particle>& ps2,
                       RigidBody* b1,
                       RigidBody* b2,
                       std::vector<Collision>& collisions) {
  for (const auto& p1 : ps1) {
    for (const auto& p2 : ps2) {
      if (are_colliding(p1, p2)) {
        collisions.push_back(Collision{p1, p2, b1, b2});
      }
    }
  }
}

std::vector<Collision> find_collisions(const std::vector<RigidBody*>& bodies) {
  std::vector<Collision> collisions;
  for (size_t i = 0; i < bodies.size(); ++i) {
    auto b_i = bodies[i];
    for (size_t j = i + 1; j < bodies.size(); ++j) {
      auto b_j = bodies[j];
      append_collisions(b_i->particles, b_j->particles, b_i, b_j, collisions);
    }
  }
  return collisions;
}

glm::quat pertubation_quat(const vec3& w) {
  const auto w_norm = glm::length(w);
  if (w_norm < 1e-10) {
    return glm::quat(1.f, 0.f, 0.f, 0.f);
  }

  const float alpha = 0.5f * w_norm;
  const float scalar = cos(alpha);
  const float beta = sin(alpha);
  const vec3 axis = beta * w / w_norm;

  return glm::quat(scalar, axis);
}

}  // namespace phycpp
