; Auto-generated. Do not edit!


(cl:in-package visual_servoing-msg)


;//! \htmlinclude Tag_poseFeedback.msg.html

(cl:defclass <Tag_poseFeedback> (roslisp-msg-protocol:ros-message)
  ((found
    :reader found
    :initarg :found
    :type cl:boolean
    :initform cl:nil)
   (foundSure
    :reader foundSure
    :initarg :foundSure
    :type cl:boolean
    :initform cl:nil)
   (tag_x
    :reader tag_x
    :initarg :tag_x
    :type cl:integer
    :initform 0)
   (tag_y
    :reader tag_y
    :initarg :tag_y
    :type cl:integer
    :initform 0))
)

(cl:defclass Tag_poseFeedback (<Tag_poseFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Tag_poseFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Tag_poseFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name visual_servoing-msg:<Tag_poseFeedback> is deprecated: use visual_servoing-msg:Tag_poseFeedback instead.")))

(cl:ensure-generic-function 'found-val :lambda-list '(m))
(cl:defmethod found-val ((m <Tag_poseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visual_servoing-msg:found-val is deprecated.  Use visual_servoing-msg:found instead.")
  (found m))

(cl:ensure-generic-function 'foundSure-val :lambda-list '(m))
(cl:defmethod foundSure-val ((m <Tag_poseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visual_servoing-msg:foundSure-val is deprecated.  Use visual_servoing-msg:foundSure instead.")
  (foundSure m))

(cl:ensure-generic-function 'tag_x-val :lambda-list '(m))
(cl:defmethod tag_x-val ((m <Tag_poseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visual_servoing-msg:tag_x-val is deprecated.  Use visual_servoing-msg:tag_x instead.")
  (tag_x m))

(cl:ensure-generic-function 'tag_y-val :lambda-list '(m))
(cl:defmethod tag_y-val ((m <Tag_poseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visual_servoing-msg:tag_y-val is deprecated.  Use visual_servoing-msg:tag_y instead.")
  (tag_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Tag_poseFeedback>) ostream)
  "Serializes a message object of type '<Tag_poseFeedback>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'found) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'foundSure) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'tag_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'tag_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Tag_poseFeedback>) istream)
  "Deserializes a message object of type '<Tag_poseFeedback>"
    (cl:setf (cl:slot-value msg 'found) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'foundSure) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tag_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tag_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Tag_poseFeedback>)))
  "Returns string type for a message object of type '<Tag_poseFeedback>"
  "visual_servoing/Tag_poseFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Tag_poseFeedback)))
  "Returns string type for a message object of type 'Tag_poseFeedback"
  "visual_servoing/Tag_poseFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Tag_poseFeedback>)))
  "Returns md5sum for a message object of type '<Tag_poseFeedback>"
  "b03174ea5876aa5520960cf25bc4b995")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Tag_poseFeedback)))
  "Returns md5sum for a message object of type 'Tag_poseFeedback"
  "b03174ea5876aa5520960cf25bc4b995")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Tag_poseFeedback>)))
  "Returns full string definition for message of type '<Tag_poseFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback definition~%bool found~%bool foundSure~%int32 tag_x~%int32 tag_y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Tag_poseFeedback)))
  "Returns full string definition for message of type 'Tag_poseFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback definition~%bool found~%bool foundSure~%int32 tag_x~%int32 tag_y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Tag_poseFeedback>))
  (cl:+ 0
     1
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Tag_poseFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'Tag_poseFeedback
    (cl:cons ':found (found msg))
    (cl:cons ':foundSure (foundSure msg))
    (cl:cons ':tag_x (tag_x msg))
    (cl:cons ':tag_y (tag_y msg))
))
