// ROS-free loop closing on a saved session (macOS / Linux).
//
//   solid_close_loops <session_dir> <config yaml> [out_dir]
//
// Replays the session keyframe by keyframe through solid::backend::LoopCloser: the
// odometry poses from singlesession_posegraph.g2o, the scans from Scans/ and the SOLiD
// candidates from loops.csv. Writes the optimized poses (KITTI + TUM) and every checked
// candidate to out_dir (default: <session_dir>/closed).
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>

#include "solid/backend/loop_closer.hpp"
#include "solid/backend/params_yaml.hpp"
#include "solid/backend/session.hpp"

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: solid_close_loops <session_dir> <config.yaml> [out_dir]\n";
    return 1;
  }
  const std::string dir = argv[1];
  const std::string out = argc > 3 ? argv[3] : dir + "/closed";
  auto lp = solid::backend::load_loop_closing(argv[2]);
  lp.enable = true;
  const solid::backend::Session session(dir, "session", 0.0);

  // SOLiD candidates: query_id,match_id,query_stamp,match_stamp,score,yaw_rad
  std::multimap<std::size_t, std::pair<std::size_t, double>> cands;
  std::ifstream csv(dir + "/loops.csv");
  std::string line;
  std::getline(csv, line);
  while (std::getline(csv, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream ss(line);
    std::size_t q, m;
    double qs, ms, score, yaw;
    if (ss >> q >> m >> qs >> ms >> score >> yaw) cands.insert({q, {m, yaw}});
  }

  solid::backend::LoopCloser closer(lp);
  std::filesystem::create_directories(out);
  std::ofstream checked(out + "/loops_checked.csv");
  checked << "query_id,match_id,fitness,accepted\n";
  for (std::size_t i = 0; i < session.size(); ++i) {
    closer.add_keyframe(i, session.poses()[i], *session.scan(static_cast<int>(i)));
    auto range = cands.equal_range(i);
    for (auto it = range.first; it != range.second; ++it) {
      const auto r = closer.try_loop(i, it->second.first, it->second.second);
      checked << r.query << ',' << r.match << ',' << r.fitness << ',' << r.accepted << '\n';
    }
  }

  std::ofstream kitti(out + "/optimized_poses.txt"), tum(out + "/optimized_tum.txt");
  kitti << std::setprecision(9);
  tum << std::fixed << std::setprecision(9);
  const auto& st = session.stamps();
  for (std::size_t i = 0; i < closer.poses().size(); ++i) {
    const Eigen::Isometry3d& T = closer.poses()[i];
    for (int a = 0; a < 3; ++a)
      for (int b = 0; b < 4; ++b) kitti << T(a, b) << ((a == 2 && b == 3) ? '\n' : ' ');
    const Eigen::Quaterniond q(T.rotation());
    tum << (i < st.size() ? st[i] : double(i)) << ' ' << T.translation().x() << ' '
        << T.translation().y() << ' ' << T.translation().z() << ' ' << q.x() << ' ' << q.y() << ' '
        << q.z() << ' ' << q.w() << '\n';
  }
  std::cout << "[solid] " << session.size() << " keyframes, " << cands.size() << " candidates, "
            << closer.accepted_loops().size() << " loops accepted -> " << out << "\n";
  return 0;
}
