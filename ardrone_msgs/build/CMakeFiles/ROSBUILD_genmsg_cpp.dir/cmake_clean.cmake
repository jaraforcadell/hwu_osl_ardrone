FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ardrone_msgs/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/ardrone_msgs/Vector3.h"
  "../msg_gen/cpp/include/ardrone_msgs/Priority.h"
  "../msg_gen/cpp/include/ardrone_msgs/Vel.h"
  "../msg_gen/cpp/include/ardrone_msgs/Pose.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
