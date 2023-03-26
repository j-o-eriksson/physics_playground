#include "particle.hpp"

#include <glm/ext/matrix_clip_space.hpp>  // glm::perspective
#include <glm/ext/matrix_transform.hpp>  // glm::translate, glm::rotate, glm::scale
#include <glm/ext/scalar_constants.hpp>  // glm::pi
#include <glm/vec4.hpp>                  // glm::vec4

namespace phycpp {

void RigidBody::apply_force(float dt) {
  // dv/dt = F / m
  // Velocity update: dv = a * dt = F / m * dt
  v += force / mass * dt;
  force = glm::vec3(0.f);
}

void RigidBody::apply_torque(float dt) {
  // dL/dt = T = r x F, Iw = L
  // Angular velocity update: dw = I_inv * T * dt

  // (i) rotate inertia and (ii) update angular velocity
  const glm::mat3 R = glm::mat3_cast(q);
  const glm::mat3 I_inv = R * glm::inverse(I) * R;

  w += I_inv * torque * dt;
  torque = glm::vec3(0.f);
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

RigidBody make_rigid_body(const std::vector<Particle>& particles,
                          const glm::vec3& p,
                          const glm::vec3& v,
                          const glm::vec3& w) {
  return RigidBody{};
}

glm::mat3 compute_inertia_matrix(const std::vector<Particle>& particles) {
  return glm::mat3{};
}

glm::vec3 compute_force(const Particle& p1,
                        const Particle& p2,
                        const CollisionParams& params) {
  const glm::vec3 r = p1.pos - p2.pos;
  const glm::vec3 v = p1.vel - p2.vel;
  const float r_norm = glm::length(r);
  const glm::vec3 r_unit = r / r_norm;

  const glm::vec3 fs = params.k * (r_norm - p1.radius - p2.radius) * r_unit;
  const glm::vec3 fd = params.c * v;
  const glm::vec3 ft = params.t * (v - (v * r_unit) * r_unit);

  return fs + fd + ft;
}

glm::quat pertubation_quat(const glm::vec3& w) {
  const auto w_norm = glm::length(w);
  if (w_norm < 1e-10) {
    return glm::quat(1.f, 0.f, 0.f, 0.f);
  }

  const float alpha = 0.5f * w_norm;
  const float scalar = cos(alpha);
  const float beta = sin(alpha);
  const glm::vec3 axis = beta * w / w_norm;

  return glm::quat(scalar, axis);
}

}  // namespace phycpp
