#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>

namespace phycpp {

struct Particle {
  glm::vec3 pos;
  glm::vec3 vel;

  float mass;
  float radius;

  /* Initial position relative to rigid body. */
  glm::vec3 r0 = glm::vec3(0.f);
  /* Momentaneous position relative to rigid body. */
  glm::vec3 r = glm::vec3(0.f);
};

struct RigidBody {
  std::vector<Particle> particles;

  float mass;
  glm::mat3 I;

  glm::vec3 p;
  glm::vec3 v;
  glm::vec3 w;
  glm::quat q = glm::quat(1.f, 0.f, 0.f, 0.f);

  glm::vec3 force = glm::vec3(0.f);
  glm::vec3 torque = glm::vec3(0.f);

  void apply_force(float dt);
  void apply_torque(float dt);
  void update(float dt);
};

struct CollisionParams {
  /* Spring (k), dampening (c), and shearing (t) constants. */
  float k;
  float c;
  float t;
};

struct Collision {
  Particle p1;
  Particle p2;
  RigidBody* b1;
  RigidBody* b2;

  void resolve(const CollisionParams& params);
};

/* Initialize rigid body. */
RigidBody make_rigid_body(std::vector<Particle> particles,
                          const glm::vec3& vel,
                          const glm::vec3& w);

/* Compute the inertia matrix of a rigid body from its particles. */
glm::mat3 compute_inertia_matrix(const std::vector<Particle>& particles);

/* Compute particle-to-particle collision force. */
glm::vec3 compute_force(const Particle& p1,
                        const Particle& p2,
                        const CollisionParams& params);

/* Find all colliding particles between a set of rigid bodies. */
std::vector<Collision> find_collisions(const std::vector<RigidBody*>& bodies);

/* Compute pertubation quaternion from angular velocity. */
glm::quat pertubation_quat(const glm::vec3& w);

}  // namespace phycpp

#endif
