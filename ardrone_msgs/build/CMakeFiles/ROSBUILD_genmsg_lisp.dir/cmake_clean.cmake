FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ardrone_msgs/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/Vector3.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Vector3.lisp"
  "../msg_gen/lisp/Priority.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Priority.lisp"
  "../msg_gen/lisp/Vel.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Vel.lisp"
  "../msg_gen/lisp/Pose.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Pose.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)