// pybind11 binding of the SOLiD C++ core (../cpp).
// Exposes Config / Descriptor / Candidate / Extractor / Database, and the
// solid::slam keyframe/loop detector and session writer as solid.slam.
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
#include "solid/slam/loop_detector.hpp"
#include "solid/slam/session_writer.hpp"

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

Eigen::Isometry3d toIso(const Eigen::Matrix4d& m) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.matrix() = m;
  return T;
}

std::vector<float> optFloats(const py::object& o, std::size_t n, const char* what) {
  if (o.is_none()) return {};
  std::vector<float> v = o.cast<std::vector<float>>();
  if (v.size() != n) throw std::runtime_error(std::string(what) + " length must match number of points");
  return v;
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

  // ---- solid.slam : the same keyframe/loop logic and session writer as the ROS nodes ----
  py::module_ sm = m.def_submodule("slam", "Keyframe/loop detection and LT-mapper sessions (solid::slam)");
  using solid::slam::LoopDetector;
  using solid::slam::Params;
  using solid::slam::SessionWriter;
  py::class_<Params>(sm, "Params")
      .def(py::init<>())
      .def_readwrite("solid", &Params::solid)
      .def_readwrite("keyframe_dist", &Params::keyframe_dist)
      .def_readwrite("keyframe_angle_deg", &Params::keyframe_angle_deg)
      .def_readwrite("submap_scans", &Params::submap_scans)
      .def_readwrite("submap_max_dist", &Params::submap_max_dist)
      .def_readwrite("exclude_recent", &Params::exclude_recent)
      .def_readwrite("min_score", &Params::min_score)
      .def_readwrite("max_candidates", &Params::max_candidates)
      .def_readwrite("cloud_in_world", &Params::cloud_in_world)
      .def_readwrite("weight_field", &Params::weight_field)
      .def_property("extrinsic",
                    [](const Params& p) { return Eigen::Matrix4d(p.extrinsic.matrix()); },
                    [](Params& p, const Eigen::Matrix4d& m) { p.extrinsic = toIso(m); });
  py::class_<solid::slam::Loop>(sm, "Loop")
      .def_readonly("query_id", &solid::slam::Loop::query_id)
      .def_readonly("match_id", &solid::slam::Loop::match_id)
      .def_readonly("score", &solid::slam::Loop::score)
      .def_readonly("yaw_rad", &solid::slam::Loop::yaw_rad);
  py::class_<LoopDetector>(sm, "LoopDetector")
      .def(py::init<const Params&>(), py::arg("params"))
      .def("is_keyframe", [](const LoopDetector& d, const Eigen::Matrix4d& pose) {
             return d.is_keyframe(toIso(pose));
           }, py::arg("pose"))
      .def("add_keyframe",
           [](LoopDetector& d, double stamp, const Eigen::Matrix4d& pose,
              const py::array_t<float, py::array::c_style | py::array::forcecast>& pts,
              py::object weights) {
             const auto p = npToPoints(pts);
             const auto w = optFloats(weights, p.size(), "weights");
             return d.add_keyframe(stamp, toIso(pose), p, w.empty() ? nullptr : &w);
           },
           py::arg("stamp"), py::arg("pose"), py::arg("points"), py::arg("weights") = py::none())
      .def("process",
           [](LoopDetector& d, double stamp, const Eigen::Matrix4d& pose,
              const py::array_t<float, py::array::c_style | py::array::forcecast>& pts,
              py::object weights, py::object intensity) {
             const auto p = npToPoints(pts);
             const auto w = optFloats(weights, p.size(), "weights");
             const auto in = optFloats(intensity, p.size(), "intensity");
             std::vector<solid::slam::Loop> loops;
             const bool kf = d.process(stamp, toIso(pose), p, w.empty() ? nullptr : &w,
                                       in.empty() ? nullptr : &in, loops);
             return py::make_tuple(kf, loops);
           },
           py::arg("stamp"), py::arg("pose"), py::arg("points"), py::arg("weights") = py::none(),
           py::arg("intensity") = py::none(),
           "Feed one scan; returns (is_keyframe, loops). Merges the last submap_scans scans.")
      .def("sensor_pose", [](const LoopDetector& d, const Eigen::Matrix4d& odom_pose) {
             return Eigen::Matrix4d(d.sensor_pose(toIso(odom_pose)).matrix());
           }, py::arg("odom_pose"))
      .def("last_points", [](const LoopDetector& d) {
        const auto& p = d.last_points();
        py::array_t<float> a({static_cast<py::ssize_t>(p.size()), static_cast<py::ssize_t>(3)});
        auto m = a.mutable_unchecked<2>();
        for (std::size_t i = 0; i < p.size(); ++i)
          for (int k = 0; k < 3; ++k) m(static_cast<py::ssize_t>(i), k) = p[i](k);
        return a;
      })
      .def("last_intensity", &LoopDetector::last_intensity)
      .def_property_readonly("num_keyframes", [](const LoopDetector& d) { return d.keyframes().size(); })
      .def_property_readonly("stamps", [](const LoopDetector& d) {
        std::vector<double> s;
        for (const auto& k : d.keyframes()) s.push_back(k.stamp);
        return s;
      })
      .def_property_readonly("poses", [](const LoopDetector& d) {
        std::vector<Eigen::Matrix4d> s;
        for (const auto& k : d.keyframes()) s.push_back(k.pose.matrix());
        return s;
      })
      .def_property_readonly("loops", &LoopDetector::loops)
      .def("last_descriptor", &LoopDetector::last_descriptor)
      .def("save", &LoopDetector::save, py::arg("dir"));
  py::class_<SessionWriter>(sm, "SessionWriter")
      .def(py::init<std::string>(), py::arg("dir"))
      .def("add_keyframe",
           [](SessionWriter& w, std::size_t id,
              const py::array_t<float, py::array::c_style | py::array::forcecast>& pts,
              py::object intensity, const Descriptor& d) {
             const auto p = npToPoints(pts);
             w.add_keyframe(id, p, optFloats(intensity, p.size(), "intensity"), d);
           },
           py::arg("id"), py::arg("points"), py::arg("intensity"), py::arg("descriptor"))
      .def("finish", [](const SessionWriter& w, const LoopDetector& d) { return w.finish(d); },
           py::arg("detector"));
  sm.def("make_scan_context",
         [](const py::array_t<float, py::array::c_style | py::array::forcecast>& pts) {
           return solid::slam::make_scan_context(npToPoints(pts));
         }, py::arg("points"));
}
