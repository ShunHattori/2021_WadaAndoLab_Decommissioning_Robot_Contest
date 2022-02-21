; Auto-generated. Do not edit!


(cl:in-package ds4_driver-msg)


;//! \htmlinclude Trackpad.msg.html

(cl:defclass <Trackpad> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (active
    :reader active
    :initarg :active
    :type cl:integer
    :initform 0)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0))
)

(cl:defclass Trackpad (<Trackpad>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Trackpad>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Trackpad)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ds4_driver-msg:<Trackpad> is deprecated: use ds4_driver-msg:Trackpad instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Trackpad>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:id-val is deprecated.  Use ds4_driver-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'active-val :lambda-list '(m))
(cl:defmethod active-val ((m <Trackpad>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:active-val is deprecated.  Use ds4_driver-msg:active instead.")
  (active m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Trackpad>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:x-val is deprecated.  Use ds4_driver-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Trackpad>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:y-val is deprecated.  Use ds4_driver-msg:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Trackpad>) ostream)
  "Serializes a message object of type '<Trackpad>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'active)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Trackpad>) istream)
  "Deserializes a message object of type '<Trackpad>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'active) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Trackpad>)))
  "Returns string type for a message object of type '<Trackpad>"
  "ds4_driver/Trackpad")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Trackpad)))
  "Returns string type for a message object of type 'Trackpad"
  "ds4_driver/Trackpad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Trackpad>)))
  "Returns md5sum for a message object of type '<Trackpad>"
  "7f8d46ab2334dfb3664bed321f9eaf05")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Trackpad)))
  "Returns md5sum for a message object of type 'Trackpad"
  "7f8d46ab2334dfb3664bed321f9eaf05")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Trackpad>)))
  "Returns full string definition for message of type '<Trackpad>"
  (cl:format cl:nil "# Trackpad message for DualShock 4~%uint16 id       # Touch ID (increments every touch)~%int32 active    # 0: inactive, 1: active~%float32 x       # 0.0: left edge, 1.0: right edge~%float32 y       # 0.0: left edge, 1.0: right edge~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Trackpad)))
  "Returns full string definition for message of type 'Trackpad"
  (cl:format cl:nil "# Trackpad message for DualShock 4~%uint16 id       # Touch ID (increments every touch)~%int32 active    # 0: inactive, 1: active~%float32 x       # 0.0: left edge, 1.0: right edge~%float32 y       # 0.0: left edge, 1.0: right edge~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Trackpad>))
  (cl:+ 0
     2
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Trackpad>))
  "Converts a ROS message object to a list"
  (cl:list 'Trackpad
    (cl:cons ':id (id msg))
    (cl:cons ':active (active msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
