// pybind11 binding of the SOLiD C++ core (../cpp).
// Exposes Config / Descriptor / Candidate / Extractor / Database to Python.
// Built PCL-free (internal voxel) so the wheel is light and importable anywhere.
#include <stdexcept>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include "solid/config.hpp"
#include "solid/database.hpp"
#include "solid/descriptor.hpp"
#include "solid/extractor.hpp"

namespace py = pybind11;
using namespace solid;

namespace {

std::vector<Eigen::Vector3f> npToPoints(
    const py::array_t<float, py::array::c_style | py::array::forcecast>& arr) {
  if (arr.ndim() != 2 || arr.shape(1) != 3) {
    throw std::runtime_error("points must be a (N, 3) float array");
  }
  auto b = arr.unchecked<2>();
  std::vector<Eigen::Vector3f> pts;
  pts.reserve(static_cast<std::size_t>(arr.shape(0)));
  for (py::ssize_t i = 0; i < arr.shape(0); ++i) {
    pts.emplace_back(b(i, 0), b(i, 1), b(i, 2));
  }
  return pts;
}

Descriptor extractNp(
    const Extractor& e,
    const py::array_t<float, py::array::c_style | py::array::forcecast>& pts,
    py::object weights) {
  const std::vector<Eigen::Vector3f> p = npToPoints(pts);
  if (weights.is_none()) return e.extract(p);
  const std::vector<float> w = weights.cast<std::vector<float>>();
  if (w.size() != p.size()) {
    throw std::runtime_error("weights length must match number of points");
  }
  return e.extract(p, &w);
}

}  // namespace

PYBIND11_MODULE(_core, m) {
  m.doc() = "SOLiD descriptor core (C++ binding)";

  py::class_<Config>(m, "Config")
      .def(py::init<>())
      .def_readwrite("fov_up", &Config::fov_up)
      .def_readwrite("fov_down", &Config::fov_down)
      .def_readwrite("num_angle", &Config::num_angle)
      .def_readwrite("num_range", &Config::num_range)
      .def_readwrite("num_height", &Config::num_height)
      .def_readwrite("min_range", &Config::min_range)
      .def_readwrite("max_range", &Config::max_range)
      .def_readwrite("voxel_size", &Config::voxel_size)
      .def_readwrite("use_weight", &Config::use_weight)
      .def_readwrite("knn", &Config::knn)
      .def_readwrite("min_similarity", &Config::min_similarity);

  py::class_<Descriptor>(m, "Descriptor")
      .def(py::init<>())
      .def_readwrite("rsolid", &Descriptor::rsolid)
      .def_readwrite("asolid", &Descriptor::asolid)
      .def("empty", &Descriptor::empty)
      .def("combined", &Descriptor::combined);

  py::class_<Candidate>(m, "Candidate")
      .def_readonly("id", &Candidate::id)
      .def_readonly("score", &Candidate::score)
      .def_readonly("yaw_rad", &Candidate::yaw_rad)
      .def("__repr__", [](const Candidate& c) {
        return "<Candidate id=" + std::to_string(c.id) +
               " score=" + std::to_string(c.score) +
               " yaw_rad=" + std::to_string(c.yaw_rad) + ">";
      });

  py::class_<Extractor>(m, "Extractor")
      .def(py::init<const Config&>(), py::arg("config"))
      .def("config", &Extractor::config)
      .def("extract", &extractNp, py::arg("points"),
           py::arg("weights") = py::none(),
           "Extract a SOLiD descriptor from an (N,3) float point array.")
      .def_static("loop_similarity", &Extractor::loop_similarity,
                  py::arg("rsolid_query"), py::arg("rsolid_candidate"))
      .def_static("pose_yaw_deg", &Extractor::pose_yaw_deg,
                  py::arg("asolid_query"), py::arg("asolid_candidate"));

  py::class_<Database>(m, "Database")
      .def(py::init<const Config&>(), py::arg("config"))
      .def("add", &Database::add, py::arg("id"), py::arg("descriptor"))
      .def("query", &Database::query, py::arg("descriptor"))
      .def("size", &Database::size)
      .def("empty", &Database::empty);
}
