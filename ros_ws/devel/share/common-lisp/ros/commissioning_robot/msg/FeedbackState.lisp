; Auto-generated. Do not edit!


(cl:in-package commissioning_robot-msg)


;//! \htmlinclude FeedbackState.msg.html

(cl:defclass <FeedbackState> (roslisp-msg-protocol:ros-message)
  ((is_ended
    :reader is_ended
    :initarg :is_ended
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (current
    :reader current
    :initarg :current
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (reference_feedbackside
    :reader reference_feedbackside
    :initarg :reference_feedbackside
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (mode_feedbackside
    :reader mode_feedbackside
    :initarg :mode_feedbackside
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass FeedbackState (<FeedbackState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FeedbackState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FeedbackState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name commissioning_robot-msg:<FeedbackState> is deprecated: use commissioning_robot-msg:FeedbackState instead.")))

(cl:ensure-generic-function 'is_ended-val :lambda-list '(m))
(cl:defmethod is_ended-val ((m <FeedbackState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader commissioning_robot-msg:is_ended-val is deprecated.  Use commissioning_robot-msg:is_ended instead.")
  (is_ended m))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <FeedbackState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader commissioning_robot-msg:current-val is deprecated.  Use commissioning_robot-msg:current instead.")
  (current m))

(cl:ensure-generic-function 'reference_feedbackside-val :lambda-list '(m))
(cl:defmethod reference_feedbackside-val ((m <FeedbackState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader commissioning_robot-msg:reference_feedbackside-val is deprecated.  Use commissioning_robot-msg:reference_feedbackside instead.")
  (reference_feedbackside m))

(cl:ensure-generic-function 'mode_feedbackside-val :lambda-list '(m))
(cl:defmethod mode_feedbackside-val ((m <FeedbackState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader commissioning_robot-msg:mode_feedbackside-val is deprecated.  Use commissioning_robot-msg:mode_feedbackside instead.")
  (mode_feedbackside m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FeedbackState>) ostream)
  "Serializes a message object of type '<FeedbackState>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'is_ended))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'is_ended))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'current))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'reference_feedbackside))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'reference_feedbackside))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'mode_feedbackside))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'mode_feedbackside))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FeedbackState>) istream)
  "Deserializes a message object of type '<FeedbackState>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'is_ended) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'is_ended)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'current) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'current)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'reference_feedbackside) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'reference_feedbackside)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'mode_feedbackside) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'mode_feedbackside)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FeedbackState>)))
  "Returns string type for a message object of type '<FeedbackState>"
  "commissioning_robot/FeedbackState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FeedbackState)))
  "Returns string type for a message object of type 'FeedbackState"
  "commissioning_robot/FeedbackState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FeedbackState>)))
  "Returns md5sum for a message object of type '<FeedbackState>"
  "73c7091a2c6badbe1419f63c8f01c277")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FeedbackState)))
  "Returns md5sum for a message object of type 'FeedbackState"
  "73c7091a2c6badbe1419f63c8f01c277")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FeedbackState>)))
  "Returns full string definition for message of type '<FeedbackState>"
  (cl:format cl:nil "uint8[] is_ended~%float64[] current~%float64[] reference_feedbackside~%uint8[] mode_feedbackside~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FeedbackState)))
  "Returns full string definition for message of type 'FeedbackState"
  (cl:format cl:nil "uint8[] is_ended~%float64[] current~%float64[] reference_feedbackside~%uint8[] mode_feedbackside~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FeedbackState>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'is_ended) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'current) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'reference_feedbackside) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'mode_feedbackside) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FeedbackState>))
  "Converts a ROS message object to a list"
  (cl:list 'FeedbackState
    (cl:cons ':is_ended (is_ended msg))
    (cl:cons ':current (current msg))
    (cl:cons ':reference_feedbackside (reference_feedbackside msg))
    (cl:cons ':mode_feedbackside (mode_feedbackside msg))
))
