
(cl:in-package :asdf)

(defsystem "ardrone_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Vector3" :depends-on ("_package_Vector3"))
    (:file "_package_Vector3" :depends-on ("_package"))
    (:file "Priority" :depends-on ("_package_Priority"))
    (:file "_package_Priority" :depends-on ("_package"))
    (:file "Vel" :depends-on ("_package_Vel"))
    (:file "_package_Vel" :depends-on ("_package"))
    (:file "Pose" :depends-on ("_package_Pose"))
    (:file "_package_Pose" :depends-on ("_package"))
  ))