; Auto-generated. Do not edit!


(cl:in-package visual_servoing-msg)


;//! \htmlinclude trackingGoal.msg.html

(cl:defclass <trackingGoal> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass trackingGoal (<trackingGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <trackingGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'trackingGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name visual_servoing-msg:<trackingGoal> is deprecated: use visual_servoing-msg:trackingGoal instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <trackingGoal>) ostream)
  "Serializes a message object of type '<trackingGoal>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <trackingGoal>) istream)
  "Deserializes a message object of type '<trackingGoal>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<trackingGoal>)))
  "Returns string type for a message object of type '<trackingGoal>"
  "visual_servoing/trackingGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'trackingGoal)))
  "Returns string type for a message object of type 'trackingGoal"
  "visual_servoing/trackingGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<trackingGoal>)))
  "Returns md5sum for a message object of type '<trackingGoal>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'trackingGoal)))
  "Returns md5sum for a message object of type 'trackingGoal"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<trackingGoal>)))
  "Returns full string definition for message of type '<trackingGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'trackingGoal)))
  "Returns full string definition for message of type 'trackingGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <trackingGoal>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <trackingGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'trackingGoal
))