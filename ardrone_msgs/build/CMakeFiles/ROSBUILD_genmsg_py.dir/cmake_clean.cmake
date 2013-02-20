FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ardrone_msgs/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/ardrone_msgs/msg/__init__.py"
  "../src/ardrone_msgs/msg/_Vector3.py"
  "../src/ardrone_msgs/msg/_Priority.py"
  "../src/ardrone_msgs/msg/_Vel.py"
  "../src/ardrone_msgs/msg/_Pose.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
