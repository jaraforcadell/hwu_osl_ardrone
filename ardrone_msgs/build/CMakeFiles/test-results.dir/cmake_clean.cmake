FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ardrone_msgs/msg"
  "../msg_gen"
  "CMakeFiles/test-results"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/test-results.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
