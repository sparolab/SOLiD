// ROS-free multi-session alignment (macOS / Linux).
//
//   solid_align_sessions <config.yaml> <out_dir> <central_session> <session> [session ...]
#include <iostream>

#include <pcl/console/print.h>

#include "solid/backend/aligner.hpp"
#include "solid/backend/params_yaml.hpp"

int main(int argc, char** argv) {
  // Without OpenMP (e.g. Apple clang) PCL warns on every GICP call; keep errors only.
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  if (argc < 5) {
    std::cerr << "usage: solid_align_sessions <config.yaml> <out_dir> <central> <session> [...]\n";
    return 1;
  }
  const auto params = solid::backend::load_params(argv[1]);
  std::vector<std::shared_ptr<solid::backend::Session>> sessions;
  for (int i = 3; i < argc; ++i) {
    std::string d = argv[i];
    while (d.size() > 1 && d.back() == '/') d.pop_back();
    sessions.push_back(std::make_shared<solid::backend::Session>(d, d.substr(d.find_last_of('/') + 1),
                                                                 params.scan_voxel));
  }
  const auto r = solid::backend::Aligner(params).align(sessions);
  for (std::size_t s = 1; s < sessions.size(); ++s) {
    int cands = 0, verified = 0;
    for (const auto& l : r.loops)
      if (l.session == static_cast<int>(s)) cands++, verified += l.verified;
    std::cout << "[solid] " << sessions[s]->name() << " -> " << sessions[0]->name() << ": " << cands
              << " candidates, " << verified << " verified, " << r.sessions[s].inliers << " inliers, "
              << (r.sessions[s].aligned ? "ALIGNED" : "not aligned") << "\n";
  }
  solid::backend::Aligner::save(argv[2], sessions, r);
  std::cout << "[solid] done in " << r.seconds << " s -> " << argv[2] << "\n";
  return 0;
}
