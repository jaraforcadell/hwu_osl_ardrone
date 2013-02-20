; Auto-generated. Do not edit!


(cl:in-package ardrone_msgs-msg)


;//! \htmlinclude Pose.msg.html

(cl:defclass <Pose> (roslisp-msg-protocol:ros-message)
  ((real
    :reader real
    :initarg :real
    :type ardrone_msgs-msg:Vector3
    :initform (cl:make-instance 'ardrone_msgs-msg:Vector3))
   (pixels
    :reader pixels
    :initarg :pixels
    :type ardrone_msgs-msg:Vector3
    :initform (cl:make-instance 'ardrone_msgs-msg:Vector3))
   (priority
    :reader priority
    :initarg :priority
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Pose (<Pose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Pose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Pose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ardrone_msgs-msg:<Pose> is deprecated: use ardrone_msgs-msg:Pose instead.")))

(cl:ensure-generic-function 'real-val :lambda-list '(m))
(cl:defmethod real-val ((m <Pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_msgs-msg:real-val is deprecated.  Use ardrone_msgs-msg:real instead.")
  (real m))

(cl:ensure-generic-function 'pixels-val :lambda-list '(m))
(cl:defmethod pixels-val ((m <Pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_msgs-msg:pixels-val is deprecated.  Use ardrone_msgs-msg:pixels instead.")
  (pixels m))

(cl:ensure-generic-function 'priority-val :lambda-list '(m))
(cl:defmethod priority-val ((m <Pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_msgs-msg:priority-val is deprecated.  Use ardrone_msgs-msg:priority instead.")
  (priority m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Pose>) ostream)
  "Serializes a message object of type '<Pose>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'real) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pixels) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'priority) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Pose>) istream)
  "Deserializes a message object of type '<Pose>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'real) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pixels) istream)
    (cl:setf (cl:slot-value msg 'priority) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Pose>)))
  "Returns string type for a message object of type '<Pose>"
  "ardrone_msgs/Pose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Pose)))
  "Returns string type for a message object of type 'Pose"
  "ardrone_msgs/Pose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Pose>)))
  "Returns md5sum for a message object of type '<Pose>"
  "c4a280f87b43aa759c89cab10eed497e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Pose)))
  "Returns md5sum for a message object of type 'Pose"
  "c4a280f87b43aa759c89cab10eed497e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Pose>)))
  "Returns full string definition for message of type '<Pose>"
  (cl:format cl:nil "Vector3 real~%~%Vector3 pixels~%~%bool priority~%~%================================================================================~%MSG: ardrone_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Pose)))
  "Returns full string definition for message of type 'Pose"
  (cl:format cl:nil "Vector3 real~%~%Vector3 pixels~%~%bool priority~%~%================================================================================~%MSG: ardrone_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Pose>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'real))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pixels))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Pose>))
  "Converts a ROS message object to a list"
  (cl:list 'Pose
    (cl:cons ':real (real msg))
    (cl:cons ':pixels (pixels msg))
    (cl:cons ':priority (priority msg))
))
