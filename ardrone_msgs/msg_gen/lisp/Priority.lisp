; Auto-generated. Do not edit!


(cl:in-package ardrone_msgs-msg)


;//! \htmlinclude Priority.msg.html

(cl:defclass <Priority> (roslisp-msg-protocol:ros-message)
  ((priority
    :reader priority
    :initarg :priority
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Priority (<Priority>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Priority>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Priority)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ardrone_msgs-msg:<Priority> is deprecated: use ardrone_msgs-msg:Priority instead.")))

(cl:ensure-generic-function 'priority-val :lambda-list '(m))
(cl:defmethod priority-val ((m <Priority>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_msgs-msg:priority-val is deprecated.  Use ardrone_msgs-msg:priority instead.")
  (priority m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Priority>) ostream)
  "Serializes a message object of type '<Priority>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'priority) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Priority>) istream)
  "Deserializes a message object of type '<Priority>"
    (cl:setf (cl:slot-value msg 'priority) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Priority>)))
  "Returns string type for a message object of type '<Priority>"
  "ardrone_msgs/Priority")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Priority)))
  "Returns string type for a message object of type 'Priority"
  "ardrone_msgs/Priority")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Priority>)))
  "Returns md5sum for a message object of type '<Priority>"
  "ccd365fab5966953d747f1c995f86413")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Priority)))
  "Returns md5sum for a message object of type 'Priority"
  "ccd365fab5966953d747f1c995f86413")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Priority>)))
  "Returns full string definition for message of type '<Priority>"
  (cl:format cl:nil "bool priority~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Priority)))
  "Returns full string definition for message of type 'Priority"
  (cl:format cl:nil "bool priority~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Priority>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Priority>))
  "Converts a ROS message object to a list"
  (cl:list 'Priority
    (cl:cons ':priority (priority msg))
))
