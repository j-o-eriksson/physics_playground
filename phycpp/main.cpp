#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <glm/gtx/string_cast.hpp>

#include "particle.hpp"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

int add(int i, int j) {
  return i + j;
}

using std::to_string;

namespace py = pybind11;

PYBIND11_MODULE(phycpp, m) {
  m.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------
        .. currentmodule:: python_example
        .. autosummary::
           :toctree: _generate
           add
           subtract
    )pbdoc";

  py::class_<glm::vec3>(m, "Vec3")
      .def(py::init<float, float, float>())
      .def_readonly("x", &glm::vec3::x)
      .def_readonly("y", &glm::vec3::y)
      .def_readonly("z", &glm::vec3::z)
      .def("__repr__", [](const glm::vec3& v) {
        return "phycpp.Vec3: " + glm::to_string(v) + ">";
      });

  py::class_<phycpp::Particle>(m, "Particle")
      .def(py::init<glm::vec3, glm::vec3, float, float>(), py::arg("position"),
           py::arg("velocity") = glm::vec3(0.f), py::arg("mass") = 1.f,
           py::arg("radius") = 1.f)
      .def_readonly("pos", &phycpp::Particle::pos)
      .def_readonly("vel", &phycpp::Particle::vel)
      .def_readonly("mass", &phycpp::Particle::mass)
      .def_readonly("radius", &phycpp::Particle::radius)
      .def_readonly("r0", &phycpp::Particle::r0)
      .def_readonly("r", &phycpp::Particle::r)
      .def("__repr__", [](const phycpp::Particle& p) {
        return "phycpp.Particle: mass: " + to_string(p.mass) +
               ", radius: " + to_string(p.radius) +
               ", position: " + glm::to_string(p.pos) +
               ", velocity: " + glm::to_string(p.vel) + ">";
      });

  py::class_<phycpp::RigidBody>(m, "RigidBody")
      .def(py::init<>())
      .def_readonly("particles", &phycpp::RigidBody::particles)
      .def_readonly("mass", &phycpp::RigidBody::mass)
      .def_readonly("inertia", &phycpp::RigidBody::I)
      .def_readonly("position", &phycpp::RigidBody::p)
      .def_readonly("velocity", &phycpp::RigidBody::v)
      .def_readonly("angular_velocity", &phycpp::RigidBody::w)
      .def_readonly("force", &phycpp::RigidBody::force)
      .def("apply_force", &phycpp::RigidBody::apply_force)
      .def("apply_torque", &phycpp::RigidBody::apply_torque)
      .def("update", &phycpp::RigidBody::update)
      .def("__repr__", [](const phycpp::RigidBody& b) {
        return "phycpp.RigidBody: mass: " + to_string(b.mass) +
               ", inertia: " + glm::to_string(b.I) +
               ", position: " + glm::to_string(b.p) + ">";
      });

  py::class_<phycpp::Plane>(m, "Plane")
      .def(py::init<glm::vec3, glm::vec3, float>())
      .def_readonly("pos", &phycpp::Plane::pos)
      .def_readonly("norm", &phycpp::Plane::norm)
      .def_readonly("r", &phycpp::Plane::r)
      .def("__repr__", [](const phycpp::Plane& p) {
        return "phycpp.CollisionParams: pos: " + glm::to_string(p.pos) +
               ", norm: " + glm::to_string(p.norm) + ", r: " + to_string(p.r) +
               ">";
      });

  py::class_<phycpp::CollisionParams>(m, "CollisionParams")
      .def(py::init<float, float, float>())
      .def_readonly("k", &phycpp::CollisionParams::k)
      .def_readonly("c", &phycpp::CollisionParams::c)
      .def_readonly("t", &phycpp::CollisionParams::t)
      .def("__repr__", [](const phycpp::CollisionParams& p) {
        return "phycpp.CollisionParams: k: " + to_string(p.k) +
               ", c: " + to_string(p.c) + ", t: " + to_string(p.t) + ">";
      });

  py::class_<phycpp::Collision>(m, "Collision")
      .def(py::init<>())
      .def_readonly("p1", &phycpp::Collision::p1)
      .def_readonly("p2", &phycpp::Collision::p2)
      .def_readonly("b1", &phycpp::Collision::b1)
      .def_readonly("b2", &phycpp::Collision::b2)
      .def("resolve", &phycpp::Collision::resolve)
      .def("__repr__",
           [](const phycpp::Collision& c) { return "<phycpp.Collision>"; });

  py::class_<phycpp::PlaneCollision>(m, "PlaneCollision")
      .def(py::init<>())
      .def_readonly("particle", &phycpp::PlaneCollision::particle)
      .def_readonly("body", &phycpp::PlaneCollision::body)
      .def_readonly("plane", &phycpp::PlaneCollision::plane)
      .def("resolve", &phycpp::PlaneCollision::resolve)
      .def("__repr__",
           [](const phycpp::PlaneCollision& c) { return "<phycpp.PlaneCollision>"; });

  m.def("make_rigid_body", &phycpp::make_rigid_body, R"pbdoc(
        Initialize rigid body.
        Some other explanation about the add function.
    )pbdoc",
        py::arg("particles"), py::arg("vel") = glm::vec3(0.f),
        py::arg("w") = glm::vec3(0.f));

  m.def("particle_particle_force", &phycpp::particle_particle_force,
        R"pbdoc(Compute force between two particles.)pbdoc");

  m.def("find_collisions", &phycpp::find_collisions,
        R"pbdoc(Compute force between two particles.)pbdoc");

  m.def("find_plane_collisions", &phycpp::find_plane_collisions,
        R"pbdoc(Compute force between two particles.)pbdoc");

#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
}
