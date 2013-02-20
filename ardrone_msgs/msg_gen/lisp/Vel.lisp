; Auto-generated. Do not edit!


(cl:in-package ardrone_msgs-msg)


;//! \htmlinclude Vel.msg.html

(cl:defclass <Vel> (roslisp-msg-protocol:ros-message)
  ((linear
    :reader linear
    :initarg :linear
    :type ardrone_msgs-msg:Vector3
    :initform (cl:make-instance 'ardrone_msgs-msg:Vector3))
   (angular
    :reader angular
    :initarg :angular
    :type ardrone_msgs-msg:Vector3
    :initform (cl:make-instance 'ardrone_msgs-msg:Vector3))
   (priority
    :reader priority
    :initarg :priority
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Vel (<Vel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Vel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Vel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ardrone_msgs-msg:<Vel> is deprecated: use ardrone_msgs-msg:Vel instead.")))

(cl:ensure-generic-function 'linear-val :lambda-list '(m))
(cl:defmethod linear-val ((m <Vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_msgs-msg:linear-val is deprecated.  Use ardrone_msgs-msg:linear instead.")
  (linear m))

(cl:ensure-generic-function 'angular-val :lambda-list '(m))
(cl:defmethod angular-val ((m <Vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_msgs-msg:angular-val is deprecated.  Use ardrone_msgs-msg:angular instead.")
  (angular m))

(cl:ensure-generic-function 'priority-val :lambda-list '(m))
(cl:defmethod priority-val ((m <Vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_msgs-msg:priority-val is deprecated.  Use ardrone_msgs-msg:priority instead.")
  (priority m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Vel>) ostream)
  "Serializes a message object of type '<Vel>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'linear) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'angular) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'priority) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Vel>) istream)
  "Deserializes a message object of type '<Vel>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'linear) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'angular) istream)
    (cl:setf (cl:slot-value msg 'priority) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Vel>)))
  "Returns string type for a message object of type '<Vel>"
  "ardrone_msgs/Vel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Vel)))
  "Returns string type for a message object of type 'Vel"
  "ardrone_msgs/Vel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Vel>)))
  "Returns md5sum for a message object of type '<Vel>"
  "5470c96b9338413a0955b59873846a5f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Vel)))
  "Returns md5sum for a message object of type 'Vel"
  "5470c96b9338413a0955b59873846a5f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Vel>)))
  "Returns full string definition for message of type '<Vel>"
  (cl:format cl:nil "Vector3 linear~%   ~%Vector3 angular~%~%bool priority~%~%================================================================================~%MSG: ardrone_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Vel)))
  "Returns full string definition for message of type 'Vel"
  (cl:format cl:nil "Vector3 linear~%   ~%Vector3 angular~%~%bool priority~%~%================================================================================~%MSG: ardrone_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Vel>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'linear))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'angular))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Vel>))
  "Converts a ROS message object to a list"
  (cl:list 'Vel
    (cl:cons ':linear (linear msg))
    (cl:cons ':angular (angular msg))
    (cl:cons ':priority (priority msg))
))
