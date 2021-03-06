
(cl:in-package :asdf)

(defsystem "visual_servoing-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "trackingActionResult" :depends-on ("_package_trackingActionResult"))
    (:file "_package_trackingActionResult" :depends-on ("_package"))
    (:file "Tag_poseAction" :depends-on ("_package_Tag_poseAction"))
    (:file "_package_Tag_poseAction" :depends-on ("_package"))
    (:file "Tag_poseFeedback" :depends-on ("_package_Tag_poseFeedback"))
    (:file "_package_Tag_poseFeedback" :depends-on ("_package"))
    (:file "Tag_poseActionGoal" :depends-on ("_package_Tag_poseActionGoal"))
    (:file "_package_Tag_poseActionGoal" :depends-on ("_package"))
    (:file "trackingResult" :depends-on ("_package_trackingResult"))
    (:file "_package_trackingResult" :depends-on ("_package"))
    (:file "trackingActionFeedback" :depends-on ("_package_trackingActionFeedback"))
    (:file "_package_trackingActionFeedback" :depends-on ("_package"))
    (:file "Tag_poseActionResult" :depends-on ("_package_Tag_poseActionResult"))
    (:file "_package_Tag_poseActionResult" :depends-on ("_package"))
    (:file "Tag_poseGoal" :depends-on ("_package_Tag_poseGoal"))
    (:file "_package_Tag_poseGoal" :depends-on ("_package"))
    (:file "Tag_poseResult" :depends-on ("_package_Tag_poseResult"))
    (:file "_package_Tag_poseResult" :depends-on ("_package"))
    (:file "trackingActionGoal" :depends-on ("_package_trackingActionGoal"))
    (:file "_package_trackingActionGoal" :depends-on ("_package"))
    (:file "trackingAction" :depends-on ("_package_trackingAction"))
    (:file "_package_trackingAction" :depends-on ("_package"))
    (:file "trackingGoal" :depends-on ("_package_trackingGoal"))
    (:file "_package_trackingGoal" :depends-on ("_package"))
    (:file "Tag_poseActionFeedback" :depends-on ("_package_Tag_poseActionFeedback"))
    (:file "_package_Tag_poseActionFeedback" :depends-on ("_package"))
    (:file "trackingFeedback" :depends-on ("_package_trackingFeedback"))
    (:file "_package_trackingFeedback" :depends-on ("_package"))
  ))