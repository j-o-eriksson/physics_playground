#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>

namespace phycpp {

struct Particle {
  float mass;
  float radius;

  glm::vec3 pos;
  glm::vec3 vel;

  /* Initial position relative to rigid body. */
  glm::vec3 r0;
  /* Momentaneous position relative to rigid body. */
  glm::vec3 r;
};

struct RigidBody {
  std::vector<Particle> particles;

  float mass;
  glm::mat3 I;

  glm::vec3 p;
  glm::quat q;
  glm::vec3 v;
  glm::vec3 w;

  glm::vec3 force;
  glm::vec3 torque;

  void apply_force(float dt);
  void apply_torque(float dt);
  void update(float dt);
};

struct Collision {
  const Particle* p1;
  const Particle* p2;
  const RigidBody* b1;
  const RigidBody* b2;
};

struct CollisionParams {
  /* Spring (k), dampening (c), and shearing (t) constants. */
  float k;
  float c;
  float t;
};

/* Initialize rigid body. */
RigidBody make_rigid_body(const std::vector<Particle>& particles,
                          const glm::vec3& p,
                          const glm::vec3& v = glm::vec3(0.f),
                          const glm::vec3& w = glm::vec3(0.f));

/* Compute a rigid body's inertia matrix. */
glm::mat3 compute_inertia_matrix(const std::vector<Particle>& particles);

/* Compute particle-to-particle collision force. */
glm::vec3 compute_force(const Particle& p1,
                        const Particle& p2,
                        const CollisionParams& params);

/* Compute pertubation quaternion from angular velocity. */
glm::quat pertubation_quat(const glm::vec3& w);

}  // namespace phycpp

#endif
