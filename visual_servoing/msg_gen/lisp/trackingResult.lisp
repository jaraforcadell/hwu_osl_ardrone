; Auto-generated. Do not edit!


(cl:in-package visual_servoing-msg)


;//! \htmlinclude trackingResult.msg.html

(cl:defclass <trackingResult> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass trackingResult (<trackingResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <trackingResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'trackingResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name visual_servoing-msg:<trackingResult> is deprecated: use visual_servoing-msg:trackingResult instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <trackingResult>) ostream)
  "Serializes a message object of type '<trackingResult>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <trackingResult>) istream)
  "Deserializes a message object of type '<trackingResult>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<trackingResult>)))
  "Returns string type for a message object of type '<trackingResult>"
  "visual_servoing/trackingResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'trackingResult)))
  "Returns string type for a message object of type 'trackingResult"
  "visual_servoing/trackingResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<trackingResult>)))
  "Returns md5sum for a message object of type '<trackingResult>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'trackingResult)))
  "Returns md5sum for a message object of type 'trackingResult"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<trackingResult>)))
  "Returns full string definition for message of type '<trackingResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'trackingResult)))
  "Returns full string definition for message of type 'trackingResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <trackingResult>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <trackingResult>))
  "Converts a ROS message object to a list"
  (cl:list 'trackingResult
))
