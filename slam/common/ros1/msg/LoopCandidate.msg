# One SOLiD loop candidate: the query keyframe looks like an older match keyframe.
# header.stamp = query keyframe stamp, header.frame_id = odometry frame.
std_msgs/Header header
uint32 query_id
uint32 match_id
time match_stamp
float64 score                # R-SOLiD cosine similarity (higher is better)
float64 yaw                  # rad, SOLiD yaw of the query relative to the match
geometry_msgs/Pose query_pose
geometry_msgs/Pose match_pose
bool verified                # true when the back-end checked the loop with GICP (loop_closing.enable)
geometry_msgs/Pose relative_pose  # verified: pose of the query keyframe in the match keyframe
