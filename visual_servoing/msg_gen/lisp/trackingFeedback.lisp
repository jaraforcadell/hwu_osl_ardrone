; Auto-generated. Do not edit!


(cl:in-package visual_servoing-msg)


;//! \htmlinclude trackingFeedback.msg.html

(cl:defclass <trackingFeedback> (roslisp-msg-protocol:ros-message)
  ((working
    :reader working
    :initarg :working
    :type cl:boolean
    :initform cl:nil)
   (vel_x
    :reader vel_x
    :initarg :vel_x
    :type cl:float
    :initform 0.0)
   (vel_y
    :reader vel_y
    :initarg :vel_y
    :type cl:float
    :initform 0.0)
   (centre_x
    :reader centre_x
    :initarg :centre_x
    :type cl:integer
    :initform 0)
   (centre_y
    :reader centre_y
    :initarg :centre_y
    :type cl:integer
    :initform 0))
)

(cl:defclass trackingFeedback (<trackingFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <trackingFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'trackingFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name visual_servoing-msg:<trackingFeedback> is deprecated: use visual_servoing-msg:trackingFeedback instead.")))

(cl:ensure-generic-function 'working-val :lambda-list '(m))
(cl:defmethod working-val ((m <trackingFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visual_servoing-msg:working-val is deprecated.  Use visual_servoing-msg:working instead.")
  (working m))

(cl:ensure-generic-function 'vel_x-val :lambda-list '(m))
(cl:defmethod vel_x-val ((m <trackingFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visual_servoing-msg:vel_x-val is deprecated.  Use visual_servoing-msg:vel_x instead.")
  (vel_x m))

(cl:ensure-generic-function 'vel_y-val :lambda-list '(m))
(cl:defmethod vel_y-val ((m <trackingFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visual_servoing-msg:vel_y-val is deprecated.  Use visual_servoing-msg:vel_y instead.")
  (vel_y m))

(cl:ensure-generic-function 'centre_x-val :lambda-list '(m))
(cl:defmethod centre_x-val ((m <trackingFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visual_servoing-msg:centre_x-val is deprecated.  Use visual_servoing-msg:centre_x instead.")
  (centre_x m))

(cl:ensure-generic-function 'centre_y-val :lambda-list '(m))
(cl:defmethod centre_y-val ((m <trackingFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visual_servoing-msg:centre_y-val is deprecated.  Use visual_servoing-msg:centre_y instead.")
  (centre_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <trackingFeedback>) ostream)
  "Serializes a message object of type '<trackingFeedback>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'working) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vel_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vel_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'centre_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'centre_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <trackingFeedback>) istream)
  "Deserializes a message object of type '<trackingFeedback>"
    (cl:setf (cl:slot-value msg 'working) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'centre_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'centre_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<trackingFeedback>)))
  "Returns string type for a message object of type '<trackingFeedback>"
  "visual_servoing/trackingFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'trackingFeedback)))
  "Returns string type for a message object of type 'trackingFeedback"
  "visual_servoing/trackingFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<trackingFeedback>)))
  "Returns md5sum for a message object of type '<trackingFeedback>"
  "ee22359594f93668d00814bfda66b0cc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'trackingFeedback)))
  "Returns md5sum for a message object of type 'trackingFeedback"
  "ee22359594f93668d00814bfda66b0cc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<trackingFeedback>)))
  "Returns full string definition for message of type '<trackingFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback definition~%bool working~%float32 vel_x~%float32 vel_y~%int32 centre_x~%int32 centre_y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'trackingFeedback)))
  "Returns full string definition for message of type 'trackingFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback definition~%bool working~%float32 vel_x~%float32 vel_y~%int32 centre_x~%int32 centre_y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <trackingFeedback>))
  (cl:+ 0
     1
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <trackingFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'trackingFeedback
    (cl:cons ':working (working msg))
    (cl:cons ':vel_x (vel_x msg))
    (cl:cons ':vel_y (vel_y msg))
    (cl:cons ':centre_x (centre_x msg))
    (cl:cons ':centre_y (centre_y msg))
))
