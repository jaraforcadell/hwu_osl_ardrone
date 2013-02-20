FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/visual_servoing/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
  "../msg/Tag_poseAction.msg"
  "../msg/Tag_poseGoal.msg"
  "../msg/Tag_poseActionGoal.msg"
  "../msg/Tag_poseResult.msg"
  "../msg/Tag_poseActionResult.msg"
  "../msg/Tag_poseFeedback.msg"
  "../msg/Tag_poseActionFeedback.msg"
  "../msg/trackingAction.msg"
  "../msg/trackingGoal.msg"
  "../msg/trackingActionGoal.msg"
  "../msg/trackingResult.msg"
  "../msg/trackingActionResult.msg"
  "../msg/trackingFeedback.msg"
  "../msg/trackingActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
