; Auto-generated. Do not edit!


(cl:in-package ds4_driver-msg)


;//! \htmlinclude Feedback.msg.html

(cl:defclass <Feedback> (roslisp-msg-protocol:ros-message)
  ((set_led
    :reader set_led
    :initarg :set_led
    :type cl:boolean
    :initform cl:nil)
   (led_r
    :reader led_r
    :initarg :led_r
    :type cl:float
    :initform 0.0)
   (led_g
    :reader led_g
    :initarg :led_g
    :type cl:float
    :initform 0.0)
   (led_b
    :reader led_b
    :initarg :led_b
    :type cl:float
    :initform 0.0)
   (set_led_flash
    :reader set_led_flash
    :initarg :set_led_flash
    :type cl:boolean
    :initform cl:nil)
   (led_flash_on
    :reader led_flash_on
    :initarg :led_flash_on
    :type cl:float
    :initform 0.0)
   (led_flash_off
    :reader led_flash_off
    :initarg :led_flash_off
    :type cl:float
    :initform 0.0)
   (set_rumble
    :reader set_rumble
    :initarg :set_rumble
    :type cl:boolean
    :initform cl:nil)
   (rumble_duration
    :reader rumble_duration
    :initarg :rumble_duration
    :type cl:float
    :initform 0.0)
   (rumble_small
    :reader rumble_small
    :initarg :rumble_small
    :type cl:float
    :initform 0.0)
   (rumble_big
    :reader rumble_big
    :initarg :rumble_big
    :type cl:float
    :initform 0.0))
)

(cl:defclass Feedback (<Feedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Feedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Feedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ds4_driver-msg:<Feedback> is deprecated: use ds4_driver-msg:Feedback instead.")))

(cl:ensure-generic-function 'set_led-val :lambda-list '(m))
(cl:defmethod set_led-val ((m <Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:set_led-val is deprecated.  Use ds4_driver-msg:set_led instead.")
  (set_led m))

(cl:ensure-generic-function 'led_r-val :lambda-list '(m))
(cl:defmethod led_r-val ((m <Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:led_r-val is deprecated.  Use ds4_driver-msg:led_r instead.")
  (led_r m))

(cl:ensure-generic-function 'led_g-val :lambda-list '(m))
(cl:defmethod led_g-val ((m <Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:led_g-val is deprecated.  Use ds4_driver-msg:led_g instead.")
  (led_g m))

(cl:ensure-generic-function 'led_b-val :lambda-list '(m))
(cl:defmethod led_b-val ((m <Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:led_b-val is deprecated.  Use ds4_driver-msg:led_b instead.")
  (led_b m))

(cl:ensure-generic-function 'set_led_flash-val :lambda-list '(m))
(cl:defmethod set_led_flash-val ((m <Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:set_led_flash-val is deprecated.  Use ds4_driver-msg:set_led_flash instead.")
  (set_led_flash m))

(cl:ensure-generic-function 'led_flash_on-val :lambda-list '(m))
(cl:defmethod led_flash_on-val ((m <Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:led_flash_on-val is deprecated.  Use ds4_driver-msg:led_flash_on instead.")
  (led_flash_on m))

(cl:ensure-generic-function 'led_flash_off-val :lambda-list '(m))
(cl:defmethod led_flash_off-val ((m <Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:led_flash_off-val is deprecated.  Use ds4_driver-msg:led_flash_off instead.")
  (led_flash_off m))

(cl:ensure-generic-function 'set_rumble-val :lambda-list '(m))
(cl:defmethod set_rumble-val ((m <Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:set_rumble-val is deprecated.  Use ds4_driver-msg:set_rumble instead.")
  (set_rumble m))

(cl:ensure-generic-function 'rumble_duration-val :lambda-list '(m))
(cl:defmethod rumble_duration-val ((m <Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:rumble_duration-val is deprecated.  Use ds4_driver-msg:rumble_duration instead.")
  (rumble_duration m))

(cl:ensure-generic-function 'rumble_small-val :lambda-list '(m))
(cl:defmethod rumble_small-val ((m <Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:rumble_small-val is deprecated.  Use ds4_driver-msg:rumble_small instead.")
  (rumble_small m))

(cl:ensure-generic-function 'rumble_big-val :lambda-list '(m))
(cl:defmethod rumble_big-val ((m <Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:rumble_big-val is deprecated.  Use ds4_driver-msg:rumble_big instead.")
  (rumble_big m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Feedback>) ostream)
  "Serializes a message object of type '<Feedback>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'set_led) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'led_r))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'led_g))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'led_b))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'set_led_flash) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'led_flash_on))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'led_flash_off))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'set_rumble) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rumble_duration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rumble_small))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rumble_big))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Feedback>) istream)
  "Deserializes a message object of type '<Feedback>"
    (cl:setf (cl:slot-value msg 'set_led) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'led_r) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'led_g) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'led_b) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'set_led_flash) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'led_flash_on) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'led_flash_off) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'set_rumble) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rumble_duration) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rumble_small) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rumble_big) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Feedback>)))
  "Returns string type for a message object of type '<Feedback>"
  "ds4_driver/Feedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Feedback)))
  "Returns string type for a message object of type 'Feedback"
  "ds4_driver/Feedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Feedback>)))
  "Returns md5sum for a message object of type '<Feedback>"
  "e6ff655a0461aca2c8725df811fe5c33")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Feedback)))
  "Returns md5sum for a message object of type 'Feedback"
  "e6ff655a0461aca2c8725df811fe5c33")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Feedback>)))
  "Returns full string definition for message of type '<Feedback>"
  (cl:format cl:nil "# Message file for controlling DualShock4~%# LED intensities [0.0, 1.0]~%bool set_led~%float32 led_r~%float32 led_g~%float32 led_b~%~%# LED flash durations in seconds (max is 2.5 sec)~%bool set_led_flash~%# Note: Set led_flash_off to 0 to stop flashing~%float32 led_flash_on~%float32 led_flash_off~%~%# Rumble intensities [0.0, 1.0] (continues rumbling for 5 sec by default)~%bool set_rumble~%float32 rumble_duration~%float32 rumble_small~%float32 rumble_big~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Feedback)))
  "Returns full string definition for message of type 'Feedback"
  (cl:format cl:nil "# Message file for controlling DualShock4~%# LED intensities [0.0, 1.0]~%bool set_led~%float32 led_r~%float32 led_g~%float32 led_b~%~%# LED flash durations in seconds (max is 2.5 sec)~%bool set_led_flash~%# Note: Set led_flash_off to 0 to stop flashing~%float32 led_flash_on~%float32 led_flash_off~%~%# Rumble intensities [0.0, 1.0] (continues rumbling for 5 sec by default)~%bool set_rumble~%float32 rumble_duration~%float32 rumble_small~%float32 rumble_big~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Feedback>))
  (cl:+ 0
     1
     4
     4
     4
     1
     4
     4
     1
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Feedback>))
  "Converts a ROS message object to a list"
  (cl:list 'Feedback
    (cl:cons ':set_led (set_led msg))
    (cl:cons ':led_r (led_r msg))
    (cl:cons ':led_g (led_g msg))
    (cl:cons ':led_b (led_b msg))
    (cl:cons ':set_led_flash (set_led_flash msg))
    (cl:cons ':led_flash_on (led_flash_on msg))
    (cl:cons ':led_flash_off (led_flash_off msg))
    (cl:cons ':set_rumble (set_rumble msg))
    (cl:cons ':rumble_duration (rumble_duration msg))
    (cl:cons ':rumble_small (rumble_small msg))
    (cl:cons ':rumble_big (rumble_big msg))
))
