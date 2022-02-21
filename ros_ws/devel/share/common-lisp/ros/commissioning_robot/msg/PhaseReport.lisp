; Auto-generated. Do not edit!


(cl:in-package commissioning_robot-msg)


;//! \htmlinclude PhaseReport.msg.html

(cl:defclass <PhaseReport> (roslisp-msg-protocol:ros-message)
  ((phase
    :reader phase
    :initarg :phase
    :type cl:fixnum
    :initform 0)
   (description
    :reader description
    :initarg :description
    :type cl:string
    :initform ""))
)

(cl:defclass PhaseReport (<PhaseReport>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PhaseReport>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PhaseReport)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name commissioning_robot-msg:<PhaseReport> is deprecated: use commissioning_robot-msg:PhaseReport instead.")))

(cl:ensure-generic-function 'phase-val :lambda-list '(m))
(cl:defmethod phase-val ((m <PhaseReport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader commissioning_robot-msg:phase-val is deprecated.  Use commissioning_robot-msg:phase instead.")
  (phase m))

(cl:ensure-generic-function 'description-val :lambda-list '(m))
(cl:defmethod description-val ((m <PhaseReport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader commissioning_robot-msg:description-val is deprecated.  Use commissioning_robot-msg:description instead.")
  (description m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PhaseReport>) ostream)
  "Serializes a message object of type '<PhaseReport>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'phase)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'description))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PhaseReport>) istream)
  "Deserializes a message object of type '<PhaseReport>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'phase)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PhaseReport>)))
  "Returns string type for a message object of type '<PhaseReport>"
  "commissioning_robot/PhaseReport")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PhaseReport)))
  "Returns string type for a message object of type 'PhaseReport"
  "commissioning_robot/PhaseReport")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PhaseReport>)))
  "Returns md5sum for a message object of type '<PhaseReport>"
  "73ce18b297c87cc885b69b7c94ec8474")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PhaseReport)))
  "Returns md5sum for a message object of type 'PhaseReport"
  "73ce18b297c87cc885b69b7c94ec8474")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PhaseReport>)))
  "Returns full string definition for message of type '<PhaseReport>"
  (cl:format cl:nil "uint8 phase~%string description~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PhaseReport)))
  "Returns full string definition for message of type 'PhaseReport"
  (cl:format cl:nil "uint8 phase~%string description~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PhaseReport>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'description))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PhaseReport>))
  "Converts a ROS message object to a list"
  (cl:list 'PhaseReport
    (cl:cons ':phase (phase msg))
    (cl:cons ':description (description msg))
))
