#include "solid/backend/aligner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>

#include "solid/database.hpp"

namespace solid {
namespace backend {

namespace {

gtsam::Pose3 to_gtsam(const Eigen::Isometry3d& T) { return gtsam::Pose3(T.matrix()); }

Eigen::Isometry3d to_eigen(const gtsam::Pose3& P) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.matrix() = P.matrix();
  return T;
}

gtsam::Key key(int session, int kf) {
  return gtsam::Symbol(static_cast<unsigned char>('a' + session), static_cast<std::uint64_t>(kf));
}

solid::Descriptor normalized(const solid::Descriptor& d) {
  solid::Descriptor out = d;
  const double n = out.rsolid.norm();
  if (n > 0.0) out.rsolid /= n;
  return out;
}

double rot_deg(const Eigen::Isometry3d& T) {
  return Eigen::AngleAxisd(T.rotation()).angle() * 180.0 / M_PI;
}

gtsam::SharedNoiseModel diag(double trans, double rot_deg) {
  const double r = rot_deg * M_PI / 180.0;
  return gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << r, r, r, trans, trans, trans).finished());
}

}  // namespace

bool Aligner::verify(const Session& central, const Session& query, InterLoop& l,
                     double yaw) const {
  pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
  gicp.setMaxCorrespondenceDistance(p_.icp_max_dist);
  gicp.setMaximumIterations(50);
  gicp.setTransformationEpsilon(1e-6);
  auto down = [this](Cloud::Ptr c) {
    if (p_.icp_voxel <= 0.0) return c;
    Cloud::Ptr out(new Cloud);
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(p_.icp_voxel, p_.icp_voxel, p_.icp_voxel);
    vg.setInputCloud(c);
    vg.filter(*out);
    return out;
  };
  gicp.setInputSource(down(query.submap(l.query_kf, p_.submap_keyframes)));
  gicp.setInputTarget(down(central.submap(l.central_kf, p_.submap_keyframes)));
  Cloud aligned;
  const Eigen::Matrix4f guess =
      Eigen::Affine3f(Eigen::AngleAxisf(static_cast<float>(yaw), Eigen::Vector3f::UnitZ())).matrix();
  gicp.align(aligned, guess);
  if (!gicp.hasConverged()) return false;
  const double fit = gicp.getFitnessScore(p_.icp_max_dist);
  if (l.fitness >= 0.0 && fit >= l.fitness) return l.verified;  // keep the better guess
  l.fitness = fit;
  l.T_cq.matrix() = gicp.getFinalTransformation().cast<double>();
  l.verified = fit < p_.icp_max_fitness;
  return l.verified;
}

std::vector<InterLoop> Aligner::find_loops(const Session& central, const Session& query,
                                           int qidx) const {
  solid::Database db(p_.solid);
  for (std::size_t i = 0; i < central.size(); ++i) db.add(i, normalized(central.descriptors()[i]));

  std::vector<InterLoop> loops;
  std::vector<double> yaws;
  for (std::size_t j = 0; j < query.size(); j += std::max(1, p_.candidate_stride)) {
    const auto cands = db.query(normalized(query.descriptors()[j]));
    if (cands.empty() || cands.front().score < p_.min_score) continue;
    InterLoop l;
    l.session = qidx;
    l.central_kf = static_cast<int>(cands.front().id);
    l.query_kf = static_cast<int>(j);
    l.score = cands.front().score;
    l.fitness = -1.0;
    loops.push_back(l);
    yaws.push_back(cands.front().yaw_rad);
  }
  // GICP-check the most similar candidates first.
  std::vector<std::size_t> order(loops.size());
  for (std::size_t k = 0; k < order.size(); ++k) order[k] = k;
  std::sort(order.begin(), order.end(),
            [&](std::size_t a, std::size_t b) { return loops[a].score > loops[b].score; });
  const std::size_t n = std::min<std::size_t>(order.size(), std::max(0, p_.max_verifications));
  for (std::size_t k = 0; k < n; ++k) {
    InterLoop& l = loops[order[k]];
    // The SOLiD yaw sign depends on which scan is the reference; try the other if needed.
    if (!verify(central, query, l, yaws[order[k]])) verify(central, query, l, -yaws[order[k]]);
  }
  return loops;
}

AlignResult Aligner::align(const std::vector<std::shared_ptr<Session>>& sessions) const {
  const auto t0 = std::chrono::steady_clock::now();
  AlignResult r;
  r.sessions.resize(sessions.size());
  r.sessions[0].aligned = true;

  // 1-3. loops and consensus transform per query session
  for (std::size_t s = 1; s < sessions.size(); ++s) {
    auto loops = find_loops(*sessions[0], *sessions[s], static_cast<int>(s));
    std::vector<Eigen::Isometry3d> hyp;
    std::vector<std::size_t> idx;
    for (std::size_t k = 0; k < loops.size(); ++k) {
      if (!loops[k].verified) continue;
      const auto& l = loops[k];
      hyp.push_back(sessions[0]->poses()[l.central_kf] * l.T_cq *
                    sessions[s]->poses()[l.query_kf].inverse());
      idx.push_back(k);
    }
    std::vector<int> best;
    for (std::size_t a = 0; a < hyp.size(); ++a) {
      std::vector<int> in;
      for (std::size_t b = 0; b < hyp.size(); ++b) {
        const Eigen::Isometry3d d = hyp[a].inverse() * hyp[b];
        if (d.translation().norm() < p_.consensus_trans && rot_deg(d) < p_.consensus_rot_deg)
          in.push_back(static_cast<int>(b));
      }
      if (in.size() > best.size()) best = in;
    }
    SessionResult& sr = r.sessions[s];
    sr.inliers = static_cast<int>(best.size());
    if (sr.inliers >= p_.min_inliers) {
      sr.aligned = true;
      // Pick the member closest to all others (a robust "median" transform).
      double best_cost = 1e18;
      for (int a : best) {
        double cost = 0.0;
        for (int b : best) cost += (hyp[a].inverse() * hyp[b]).translation().norm();
        if (cost < best_cost) best_cost = cost, sr.T_central_session = hyp[a];
      }
      for (int b : best) loops[idx[b]].inlier = true;
    }
    r.loops.insert(r.loops.end(), loops.begin(), loops.end());
  }

  // 4. joint pose graph
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values init;
  const auto odom_noise = diag(p_.odom_sigma_trans, p_.odom_sigma_rot_deg);
  const auto loop_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(1.0),
      diag(p_.loop_sigma_trans, p_.loop_sigma_rot_deg));
  for (std::size_t s = 0; s < sessions.size(); ++s) {
    const Session& S = *sessions[s];
    const SessionResult& sr = r.sessions[s];
    for (std::size_t i = 0; i < S.size(); ++i)
      init.insert(key(static_cast<int>(s), static_cast<int>(i)),
                  to_gtsam(sr.T_central_session * S.poses()[i]));
    // Each session is anchored: central tightly, unaligned ones on their own frame.
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        key(static_cast<int>(s), 0), to_gtsam(sr.T_central_session * S.poses()[0]),
        s == 0 || !sr.aligned ? diag(1e-4, 1e-4) : diag(1e3, 1e3)));
    for (const OdomEdge& e : S.edges())
      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(key(static_cast<int>(s), e.from),
                                                   key(static_cast<int>(s), e.to), to_gtsam(e.T),
                                                   odom_noise));
  }
  for (const InterLoop& l : r.loops)
    if (l.inlier)
      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(key(0, l.central_kf), key(l.session, l.query_kf),
                                                   to_gtsam(l.T_cq), loop_noise));
  gtsam::LevenbergMarquardtParams lm;
  lm.setMaxIterations(p_.optimizer_iterations);
  const gtsam::Values est = gtsam::LevenbergMarquardtOptimizer(graph, init, lm).optimize();
  for (std::size_t s = 0; s < sessions.size(); ++s)
    for (std::size_t i = 0; i < sessions[s]->size(); ++i)
      r.sessions[s].poses.push_back(
          to_eigen(est.at<gtsam::Pose3>(key(static_cast<int>(s), static_cast<int>(i)))));

  r.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  return r;
}

Cloud::Ptr build_map(const std::vector<std::shared_ptr<Session>>& sessions, const AlignResult& r,
                     double voxel, int keyframe_stride) {
  Cloud::Ptr out(new Cloud);
  for (std::size_t s = 0; s < sessions.size(); ++s) {  // per session, so colors do not mix
    Cloud::Ptr one(new Cloud);
    for (std::size_t i = 0; i < r.sessions[s].poses.size(); i += std::max(1, keyframe_stride)) {
      Cloud tmp;
      pcl::transformPointCloud(*sessions[s]->scan(static_cast<int>(i)), tmp,
                               r.sessions[s].poses[i].matrix().cast<float>());
      *one += tmp;
    }
    if (voxel > 0.0) {
      Cloud::Ptr down(new Cloud);
      pcl::VoxelGrid<PointT> vg;
      vg.setLeafSize(voxel, voxel, voxel);
      vg.setInputCloud(one);
      vg.filter(*down);
      one = down;
    }
    for (auto& p : *one) p.intensity = static_cast<float>(s);
    *out += *one;
  }
  return out;
}

bool Aligner::save(const std::string& dir, const std::vector<std::shared_ptr<Session>>& sessions,
                   const AlignResult& r) {
  std::filesystem::create_directories(dir);
  for (std::size_t s = 0; s < sessions.size(); ++s) {
    std::ofstream kitti(dir + "/" + sessions[s]->name() + "_aligned.txt");
    std::ofstream tum(dir + "/" + sessions[s]->name() + "_aligned_tum.txt");
    if (!kitti || !tum) return false;
    kitti << std::setprecision(9);
    tum << std::fixed << std::setprecision(9);
    const auto& st = sessions[s]->stamps();
    for (std::size_t i = 0; i < r.sessions[s].poses.size(); ++i) {
      const Eigen::Isometry3d& T = r.sessions[s].poses[i];
      for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 4; ++b) kitti << T(a, b) << ((a == 2 && b == 3) ? '\n' : ' ');
      const Eigen::Quaterniond q(T.rotation());
      tum << (i < st.size() ? st[i] : static_cast<double>(i)) << ' ' << T.translation().x() << ' '
          << T.translation().y() << ' ' << T.translation().z() << ' ' << q.x() << ' ' << q.y()
          << ' ' << q.z() << ' ' << q.w() << '\n';
    }
  }
  std::ofstream csv(dir + "/inter_session_loops.csv");
  csv << "query_session,central_kf,query_kf,score,fitness,verified,inlier\n";
  for (const InterLoop& l : r.loops)
    csv << sessions[l.session]->name() << ',' << l.central_kf << ',' << l.query_kf << ','
        << l.score << ',' << l.fitness << ',' << l.verified << ',' << l.inlier << '\n';
  return true;
}

}  // namespace backend
}  // namespace solid
