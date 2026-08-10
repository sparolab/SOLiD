// SOLiD core — runtime configuration.
//
// Every descriptor parameter that was a compile-time constant in the original
// `SOLiDModule` (solid_module.h) is exposed here as a runtime field so a single
// library can serve LiDAR / 4D-radar / GS-depth profiles by swapping a config.
//
// Defaults reproduce the original SOLiDModule constants exactly, so that
// Extractor(Config{}) is numerically identical to the reference implementation.
#ifndef SOLID_CONFIG_HPP
#define SOLID_CONFIG_HPP

namespace solid {

struct Config {
  // --- descriptor geometry (degrees / meters) ---
  float fov_up = 2.0f;        // FOV_u : upper vertical FOV bound
  float fov_down = -24.8f;    // FOV_d : lower vertical FOV bound
  int num_angle = 60;         // NUM_ANGLE : A-SOLiD length (yaw bins)
  int num_range = 40;         // NUM_RANGE : R-SOLiD length
  int num_height = 32;        // NUM_HEIGHT : elevation bins
  float min_range = 3.0f;     // MIN_DISTANCE : discard points closer than this
  float max_range = 80.0f;    // MAX_DISTANCE : discard points farther than this;
                              //                also sets the range-bin spacing.
  float voxel_size = 0.4f;    // VOXEL_SIZE : voxel leaf; <= 0 disables downsampling.

  // --- weighting ---
  // false : occupancy histogram (each point contributes 1) — LiDAR default.
  // true  : each point contributes its supplied weight (e.g. radar RCS/power).
  //         Requires voxel_size <= 0 so per-point weights stay aligned.
  bool use_weight = false;

  // --- matching (used by Database) ---
  int knn = 30;               // number of nearest candidates to retrieve
  double min_similarity = 0.0;  // keep candidates with cosine similarity >= this
                                // (0.0 = return all knn; caller may filter)
};

}  // namespace solid

#endif  // SOLID_CONFIG_HPP
