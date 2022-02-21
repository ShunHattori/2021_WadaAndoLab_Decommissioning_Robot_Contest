; Auto-generated. Do not edit!


(cl:in-package ds4_driver-msg)


;//! \htmlinclude Report.msg.html

(cl:defclass <Report> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (left_analog_x
    :reader left_analog_x
    :initarg :left_analog_x
    :type cl:fixnum
    :initform 0)
   (left_analog_y
    :reader left_analog_y
    :initarg :left_analog_y
    :type cl:fixnum
    :initform 0)
   (right_analog_x
    :reader right_analog_x
    :initarg :right_analog_x
    :type cl:fixnum
    :initform 0)
   (right_analog_y
    :reader right_analog_y
    :initarg :right_analog_y
    :type cl:fixnum
    :initform 0)
   (l2_analog
    :reader l2_analog
    :initarg :l2_analog
    :type cl:fixnum
    :initform 0)
   (r2_analog
    :reader r2_analog
    :initarg :r2_analog
    :type cl:fixnum
    :initform 0)
   (dpad_up
    :reader dpad_up
    :initarg :dpad_up
    :type cl:boolean
    :initform cl:nil)
   (dpad_down
    :reader dpad_down
    :initarg :dpad_down
    :type cl:boolean
    :initform cl:nil)
   (dpad_left
    :reader dpad_left
    :initarg :dpad_left
    :type cl:boolean
    :initform cl:nil)
   (dpad_right
    :reader dpad_right
    :initarg :dpad_right
    :type cl:boolean
    :initform cl:nil)
   (button_cross
    :reader button_cross
    :initarg :button_cross
    :type cl:boolean
    :initform cl:nil)
   (button_circle
    :reader button_circle
    :initarg :button_circle
    :type cl:boolean
    :initform cl:nil)
   (button_square
    :reader button_square
    :initarg :button_square
    :type cl:boolean
    :initform cl:nil)
   (button_triangle
    :reader button_triangle
    :initarg :button_triangle
    :type cl:boolean
    :initform cl:nil)
   (button_l1
    :reader button_l1
    :initarg :button_l1
    :type cl:boolean
    :initform cl:nil)
   (button_l2
    :reader button_l2
    :initarg :button_l2
    :type cl:boolean
    :initform cl:nil)
   (button_l3
    :reader button_l3
    :initarg :button_l3
    :type cl:boolean
    :initform cl:nil)
   (button_r1
    :reader button_r1
    :initarg :button_r1
    :type cl:boolean
    :initform cl:nil)
   (button_r2
    :reader button_r2
    :initarg :button_r2
    :type cl:boolean
    :initform cl:nil)
   (button_r3
    :reader button_r3
    :initarg :button_r3
    :type cl:boolean
    :initform cl:nil)
   (button_share
    :reader button_share
    :initarg :button_share
    :type cl:boolean
    :initform cl:nil)
   (button_options
    :reader button_options
    :initarg :button_options
    :type cl:boolean
    :initform cl:nil)
   (button_trackpad
    :reader button_trackpad
    :initarg :button_trackpad
    :type cl:boolean
    :initform cl:nil)
   (button_ps
    :reader button_ps
    :initarg :button_ps
    :type cl:boolean
    :initform cl:nil)
   (lin_acc_x
    :reader lin_acc_x
    :initarg :lin_acc_x
    :type cl:fixnum
    :initform 0)
   (lin_acc_y
    :reader lin_acc_y
    :initarg :lin_acc_y
    :type cl:fixnum
    :initform 0)
   (lin_acc_z
    :reader lin_acc_z
    :initarg :lin_acc_z
    :type cl:fixnum
    :initform 0)
   (ang_vel_x
    :reader ang_vel_x
    :initarg :ang_vel_x
    :type cl:fixnum
    :initform 0)
   (ang_vel_y
    :reader ang_vel_y
    :initarg :ang_vel_y
    :type cl:fixnum
    :initform 0)
   (ang_vel_z
    :reader ang_vel_z
    :initarg :ang_vel_z
    :type cl:fixnum
    :initform 0)
   (trackpad_touch0_id
    :reader trackpad_touch0_id
    :initarg :trackpad_touch0_id
    :type cl:fixnum
    :initform 0)
   (trackpad_touch0_active
    :reader trackpad_touch0_active
    :initarg :trackpad_touch0_active
    :type cl:fixnum
    :initform 0)
   (trackpad_touch0_x
    :reader trackpad_touch0_x
    :initarg :trackpad_touch0_x
    :type cl:fixnum
    :initform 0)
   (trackpad_touch0_y
    :reader trackpad_touch0_y
    :initarg :trackpad_touch0_y
    :type cl:fixnum
    :initform 0)
   (trackpad_touch1_id
    :reader trackpad_touch1_id
    :initarg :trackpad_touch1_id
    :type cl:fixnum
    :initform 0)
   (trackpad_touch1_active
    :reader trackpad_touch1_active
    :initarg :trackpad_touch1_active
    :type cl:fixnum
    :initform 0)
   (trackpad_touch1_x
    :reader trackpad_touch1_x
    :initarg :trackpad_touch1_x
    :type cl:fixnum
    :initform 0)
   (trackpad_touch1_y
    :reader trackpad_touch1_y
    :initarg :trackpad_touch1_y
    :type cl:fixnum
    :initform 0)
   (timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:fixnum
    :initform 0)
   (battery
    :reader battery
    :initarg :battery
    :type cl:fixnum
    :initform 0)
   (plug_usb
    :reader plug_usb
    :initarg :plug_usb
    :type cl:boolean
    :initform cl:nil)
   (plug_audio
    :reader plug_audio
    :initarg :plug_audio
    :type cl:boolean
    :initform cl:nil)
   (plug_mic
    :reader plug_mic
    :initarg :plug_mic
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Report (<Report>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Report>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Report)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ds4_driver-msg:<Report> is deprecated: use ds4_driver-msg:Report instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:header-val is deprecated.  Use ds4_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'left_analog_x-val :lambda-list '(m))
(cl:defmethod left_analog_x-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:left_analog_x-val is deprecated.  Use ds4_driver-msg:left_analog_x instead.")
  (left_analog_x m))

(cl:ensure-generic-function 'left_analog_y-val :lambda-list '(m))
(cl:defmethod left_analog_y-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:left_analog_y-val is deprecated.  Use ds4_driver-msg:left_analog_y instead.")
  (left_analog_y m))

(cl:ensure-generic-function 'right_analog_x-val :lambda-list '(m))
(cl:defmethod right_analog_x-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:right_analog_x-val is deprecated.  Use ds4_driver-msg:right_analog_x instead.")
  (right_analog_x m))

(cl:ensure-generic-function 'right_analog_y-val :lambda-list '(m))
(cl:defmethod right_analog_y-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:right_analog_y-val is deprecated.  Use ds4_driver-msg:right_analog_y instead.")
  (right_analog_y m))

(cl:ensure-generic-function 'l2_analog-val :lambda-list '(m))
(cl:defmethod l2_analog-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:l2_analog-val is deprecated.  Use ds4_driver-msg:l2_analog instead.")
  (l2_analog m))

(cl:ensure-generic-function 'r2_analog-val :lambda-list '(m))
(cl:defmethod r2_analog-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:r2_analog-val is deprecated.  Use ds4_driver-msg:r2_analog instead.")
  (r2_analog m))

(cl:ensure-generic-function 'dpad_up-val :lambda-list '(m))
(cl:defmethod dpad_up-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:dpad_up-val is deprecated.  Use ds4_driver-msg:dpad_up instead.")
  (dpad_up m))

(cl:ensure-generic-function 'dpad_down-val :lambda-list '(m))
(cl:defmethod dpad_down-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:dpad_down-val is deprecated.  Use ds4_driver-msg:dpad_down instead.")
  (dpad_down m))

(cl:ensure-generic-function 'dpad_left-val :lambda-list '(m))
(cl:defmethod dpad_left-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:dpad_left-val is deprecated.  Use ds4_driver-msg:dpad_left instead.")
  (dpad_left m))

(cl:ensure-generic-function 'dpad_right-val :lambda-list '(m))
(cl:defmethod dpad_right-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:dpad_right-val is deprecated.  Use ds4_driver-msg:dpad_right instead.")
  (dpad_right m))

(cl:ensure-generic-function 'button_cross-val :lambda-list '(m))
(cl:defmethod button_cross-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:button_cross-val is deprecated.  Use ds4_driver-msg:button_cross instead.")
  (button_cross m))

(cl:ensure-generic-function 'button_circle-val :lambda-list '(m))
(cl:defmethod button_circle-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:button_circle-val is deprecated.  Use ds4_driver-msg:button_circle instead.")
  (button_circle m))

(cl:ensure-generic-function 'button_square-val :lambda-list '(m))
(cl:defmethod button_square-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:button_square-val is deprecated.  Use ds4_driver-msg:button_square instead.")
  (button_square m))

(cl:ensure-generic-function 'button_triangle-val :lambda-list '(m))
(cl:defmethod button_triangle-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:button_triangle-val is deprecated.  Use ds4_driver-msg:button_triangle instead.")
  (button_triangle m))

(cl:ensure-generic-function 'button_l1-val :lambda-list '(m))
(cl:defmethod button_l1-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:button_l1-val is deprecated.  Use ds4_driver-msg:button_l1 instead.")
  (button_l1 m))

(cl:ensure-generic-function 'button_l2-val :lambda-list '(m))
(cl:defmethod button_l2-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:button_l2-val is deprecated.  Use ds4_driver-msg:button_l2 instead.")
  (button_l2 m))

(cl:ensure-generic-function 'button_l3-val :lambda-list '(m))
(cl:defmethod button_l3-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:button_l3-val is deprecated.  Use ds4_driver-msg:button_l3 instead.")
  (button_l3 m))

(cl:ensure-generic-function 'button_r1-val :lambda-list '(m))
(cl:defmethod button_r1-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:button_r1-val is deprecated.  Use ds4_driver-msg:button_r1 instead.")
  (button_r1 m))

(cl:ensure-generic-function 'button_r2-val :lambda-list '(m))
(cl:defmethod button_r2-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:button_r2-val is deprecated.  Use ds4_driver-msg:button_r2 instead.")
  (button_r2 m))

(cl:ensure-generic-function 'button_r3-val :lambda-list '(m))
(cl:defmethod button_r3-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:button_r3-val is deprecated.  Use ds4_driver-msg:button_r3 instead.")
  (button_r3 m))

(cl:ensure-generic-function 'button_share-val :lambda-list '(m))
(cl:defmethod button_share-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:button_share-val is deprecated.  Use ds4_driver-msg:button_share instead.")
  (button_share m))

(cl:ensure-generic-function 'button_options-val :lambda-list '(m))
(cl:defmethod button_options-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:button_options-val is deprecated.  Use ds4_driver-msg:button_options instead.")
  (button_options m))

(cl:ensure-generic-function 'button_trackpad-val :lambda-list '(m))
(cl:defmethod button_trackpad-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:button_trackpad-val is deprecated.  Use ds4_driver-msg:button_trackpad instead.")
  (button_trackpad m))

(cl:ensure-generic-function 'button_ps-val :lambda-list '(m))
(cl:defmethod button_ps-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:button_ps-val is deprecated.  Use ds4_driver-msg:button_ps instead.")
  (button_ps m))

(cl:ensure-generic-function 'lin_acc_x-val :lambda-list '(m))
(cl:defmethod lin_acc_x-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:lin_acc_x-val is deprecated.  Use ds4_driver-msg:lin_acc_x instead.")
  (lin_acc_x m))

(cl:ensure-generic-function 'lin_acc_y-val :lambda-list '(m))
(cl:defmethod lin_acc_y-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:lin_acc_y-val is deprecated.  Use ds4_driver-msg:lin_acc_y instead.")
  (lin_acc_y m))

(cl:ensure-generic-function 'lin_acc_z-val :lambda-list '(m))
(cl:defmethod lin_acc_z-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:lin_acc_z-val is deprecated.  Use ds4_driver-msg:lin_acc_z instead.")
  (lin_acc_z m))

(cl:ensure-generic-function 'ang_vel_x-val :lambda-list '(m))
(cl:defmethod ang_vel_x-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:ang_vel_x-val is deprecated.  Use ds4_driver-msg:ang_vel_x instead.")
  (ang_vel_x m))

(cl:ensure-generic-function 'ang_vel_y-val :lambda-list '(m))
(cl:defmethod ang_vel_y-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:ang_vel_y-val is deprecated.  Use ds4_driver-msg:ang_vel_y instead.")
  (ang_vel_y m))

(cl:ensure-generic-function 'ang_vel_z-val :lambda-list '(m))
(cl:defmethod ang_vel_z-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:ang_vel_z-val is deprecated.  Use ds4_driver-msg:ang_vel_z instead.")
  (ang_vel_z m))

(cl:ensure-generic-function 'trackpad_touch0_id-val :lambda-list '(m))
(cl:defmethod trackpad_touch0_id-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:trackpad_touch0_id-val is deprecated.  Use ds4_driver-msg:trackpad_touch0_id instead.")
  (trackpad_touch0_id m))

(cl:ensure-generic-function 'trackpad_touch0_active-val :lambda-list '(m))
(cl:defmethod trackpad_touch0_active-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:trackpad_touch0_active-val is deprecated.  Use ds4_driver-msg:trackpad_touch0_active instead.")
  (trackpad_touch0_active m))

(cl:ensure-generic-function 'trackpad_touch0_x-val :lambda-list '(m))
(cl:defmethod trackpad_touch0_x-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:trackpad_touch0_x-val is deprecated.  Use ds4_driver-msg:trackpad_touch0_x instead.")
  (trackpad_touch0_x m))

(cl:ensure-generic-function 'trackpad_touch0_y-val :lambda-list '(m))
(cl:defmethod trackpad_touch0_y-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:trackpad_touch0_y-val is deprecated.  Use ds4_driver-msg:trackpad_touch0_y instead.")
  (trackpad_touch0_y m))

(cl:ensure-generic-function 'trackpad_touch1_id-val :lambda-list '(m))
(cl:defmethod trackpad_touch1_id-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:trackpad_touch1_id-val is deprecated.  Use ds4_driver-msg:trackpad_touch1_id instead.")
  (trackpad_touch1_id m))

(cl:ensure-generic-function 'trackpad_touch1_active-val :lambda-list '(m))
(cl:defmethod trackpad_touch1_active-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:trackpad_touch1_active-val is deprecated.  Use ds4_driver-msg:trackpad_touch1_active instead.")
  (trackpad_touch1_active m))

(cl:ensure-generic-function 'trackpad_touch1_x-val :lambda-list '(m))
(cl:defmethod trackpad_touch1_x-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:trackpad_touch1_x-val is deprecated.  Use ds4_driver-msg:trackpad_touch1_x instead.")
  (trackpad_touch1_x m))

(cl:ensure-generic-function 'trackpad_touch1_y-val :lambda-list '(m))
(cl:defmethod trackpad_touch1_y-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:trackpad_touch1_y-val is deprecated.  Use ds4_driver-msg:trackpad_touch1_y instead.")
  (trackpad_touch1_y m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:timestamp-val is deprecated.  Use ds4_driver-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'battery-val :lambda-list '(m))
(cl:defmethod battery-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:battery-val is deprecated.  Use ds4_driver-msg:battery instead.")
  (battery m))

(cl:ensure-generic-function 'plug_usb-val :lambda-list '(m))
(cl:defmethod plug_usb-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:plug_usb-val is deprecated.  Use ds4_driver-msg:plug_usb instead.")
  (plug_usb m))

(cl:ensure-generic-function 'plug_audio-val :lambda-list '(m))
(cl:defmethod plug_audio-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:plug_audio-val is deprecated.  Use ds4_driver-msg:plug_audio instead.")
  (plug_audio m))

(cl:ensure-generic-function 'plug_mic-val :lambda-list '(m))
(cl:defmethod plug_mic-val ((m <Report>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_driver-msg:plug_mic-val is deprecated.  Use ds4_driver-msg:plug_mic instead.")
  (plug_mic m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Report>) ostream)
  "Serializes a message object of type '<Report>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'left_analog_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'left_analog_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'right_analog_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'right_analog_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'l2_analog)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'r2_analog)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dpad_up) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dpad_down) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dpad_left) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dpad_right) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button_cross) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button_circle) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button_square) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button_triangle) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button_l1) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button_l2) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button_l3) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button_r1) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button_r2) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button_r3) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button_share) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button_options) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button_trackpad) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button_ps) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'lin_acc_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'lin_acc_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'lin_acc_z)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ang_vel_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ang_vel_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ang_vel_z)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trackpad_touch0_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'trackpad_touch0_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trackpad_touch0_active)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'trackpad_touch0_active)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trackpad_touch0_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'trackpad_touch0_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trackpad_touch0_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'trackpad_touch0_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trackpad_touch1_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'trackpad_touch1_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trackpad_touch1_active)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'trackpad_touch1_active)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trackpad_touch1_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'trackpad_touch1_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trackpad_touch1_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'trackpad_touch1_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'battery)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'plug_usb) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'plug_audio) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'plug_mic) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Report>) istream)
  "Deserializes a message object of type '<Report>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'left_analog_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'left_analog_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'right_analog_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'right_analog_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'l2_analog)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'r2_analog)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dpad_up) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dpad_down) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dpad_left) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dpad_right) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button_cross) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button_circle) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button_square) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button_triangle) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button_l1) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button_l2) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button_l3) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button_r1) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button_r2) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button_r3) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button_share) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button_options) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button_trackpad) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button_ps) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lin_acc_x) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lin_acc_y) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lin_acc_z) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ang_vel_x) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ang_vel_y) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ang_vel_z) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trackpad_touch0_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'trackpad_touch0_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trackpad_touch0_active)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'trackpad_touch0_active)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trackpad_touch0_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'trackpad_touch0_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trackpad_touch0_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'trackpad_touch0_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trackpad_touch1_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'trackpad_touch1_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trackpad_touch1_active)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'trackpad_touch1_active)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trackpad_touch1_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'trackpad_touch1_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trackpad_touch1_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'trackpad_touch1_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'battery)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'plug_usb) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'plug_audio) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'plug_mic) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Report>)))
  "Returns string type for a message object of type '<Report>"
  "ds4_driver/Report")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Report)))
  "Returns string type for a message object of type 'Report"
  "ds4_driver/Report")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Report>)))
  "Returns md5sum for a message object of type '<Report>"
  "ec2c37165ced5aec5b7a50d72696b7dc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Report)))
  "Returns md5sum for a message object of type 'Report"
  "ec2c37165ced5aec5b7a50d72696b7dc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Report>)))
  "Returns full string definition for message of type '<Report>"
  (cl:format cl:nil "# Raw report from DualShock 4~%Header header~%~%# Left: 0, Right: 255~%uint8 left_analog_x~%# Up: 0, Down: 255~%uint8 left_analog_y~%uint8 right_analog_x~%uint8 right_analog_y~%~%# Released: 0, Pressed: 255~%uint8 l2_analog~%# Released: 0, Pressed: 255~%uint8 r2_analog~%~%# Released: 0, Pressed: 1~%bool dpad_up~%bool dpad_down~%bool dpad_left~%bool dpad_right~%bool button_cross~%bool button_circle~%bool button_square~%bool button_triangle~%bool button_l1~%bool button_l2~%bool button_l3~%bool button_r1~%bool button_r2~%bool button_r3~%bool button_share~%bool button_options~%bool button_trackpad~%bool button_ps~%~%# IMU~%int16 lin_acc_x~%int16 lin_acc_y~%int16 lin_acc_z~%int16 ang_vel_x~%int16 ang_vel_y~%int16 ang_vel_z~%~%# Top-left: (0, 0), Bottom-right: (1919, 942)~%uint16 trackpad_touch0_id~%uint16 trackpad_touch0_active~%uint16 trackpad_touch0_x~%uint16 trackpad_touch0_y~%uint16 trackpad_touch1_id~%uint16 trackpad_touch1_active~%uint16 trackpad_touch1_x~%uint16 trackpad_touch1_y~%~%uint8 timestamp~%# Full: 8, Full (and charging): 11~%uint8 battery~%~%# Unused: 0, Plugged: 1~%bool plug_usb~%bool plug_audio~%bool plug_mic~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Report)))
  "Returns full string definition for message of type 'Report"
  (cl:format cl:nil "# Raw report from DualShock 4~%Header header~%~%# Left: 0, Right: 255~%uint8 left_analog_x~%# Up: 0, Down: 255~%uint8 left_analog_y~%uint8 right_analog_x~%uint8 right_analog_y~%~%# Released: 0, Pressed: 255~%uint8 l2_analog~%# Released: 0, Pressed: 255~%uint8 r2_analog~%~%# Released: 0, Pressed: 1~%bool dpad_up~%bool dpad_down~%bool dpad_left~%bool dpad_right~%bool button_cross~%bool button_circle~%bool button_square~%bool button_triangle~%bool button_l1~%bool button_l2~%bool button_l3~%bool button_r1~%bool button_r2~%bool button_r3~%bool button_share~%bool button_options~%bool button_trackpad~%bool button_ps~%~%# IMU~%int16 lin_acc_x~%int16 lin_acc_y~%int16 lin_acc_z~%int16 ang_vel_x~%int16 ang_vel_y~%int16 ang_vel_z~%~%# Top-left: (0, 0), Bottom-right: (1919, 942)~%uint16 trackpad_touch0_id~%uint16 trackpad_touch0_active~%uint16 trackpad_touch0_x~%uint16 trackpad_touch0_y~%uint16 trackpad_touch1_id~%uint16 trackpad_touch1_active~%uint16 trackpad_touch1_x~%uint16 trackpad_touch1_y~%~%uint8 timestamp~%# Full: 8, Full (and charging): 11~%uint8 battery~%~%# Unused: 0, Plugged: 1~%bool plug_usb~%bool plug_audio~%bool plug_mic~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Report>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     2
     2
     2
     2
     2
     2
     2
     2
     2
     2
     2
     2
     2
     2
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Report>))
  "Converts a ROS message object to a list"
  (cl:list 'Report
    (cl:cons ':header (header msg))
    (cl:cons ':left_analog_x (left_analog_x msg))
    (cl:cons ':left_analog_y (left_analog_y msg))
    (cl:cons ':right_analog_x (right_analog_x msg))
    (cl:cons ':right_analog_y (right_analog_y msg))
    (cl:cons ':l2_analog (l2_analog msg))
    (cl:cons ':r2_analog (r2_analog msg))
    (cl:cons ':dpad_up (dpad_up msg))
    (cl:cons ':dpad_down (dpad_down msg))
    (cl:cons ':dpad_left (dpad_left msg))
    (cl:cons ':dpad_right (dpad_right msg))
    (cl:cons ':button_cross (button_cross msg))
    (cl:cons ':button_circle (button_circle msg))
    (cl:cons ':button_square (button_square msg))
    (cl:cons ':button_triangle (button_triangle msg))
    (cl:cons ':button_l1 (button_l1 msg))
    (cl:cons ':button_l2 (button_l2 msg))
    (cl:cons ':button_l3 (button_l3 msg))
    (cl:cons ':button_r1 (button_r1 msg))
    (cl:cons ':button_r2 (button_r2 msg))
    (cl:cons ':button_r3 (button_r3 msg))
    (cl:cons ':button_share (button_share msg))
    (cl:cons ':button_options (button_options msg))
    (cl:cons ':button_trackpad (button_trackpad msg))
    (cl:cons ':button_ps (button_ps msg))
    (cl:cons ':lin_acc_x (lin_acc_x msg))
    (cl:cons ':lin_acc_y (lin_acc_y msg))
    (cl:cons ':lin_acc_z (lin_acc_z msg))
    (cl:cons ':ang_vel_x (ang_vel_x msg))
    (cl:cons ':ang_vel_y (ang_vel_y msg))
    (cl:cons ':ang_vel_z (ang_vel_z msg))
    (cl:cons ':trackpad_touch0_id (trackpad_touch0_id msg))
    (cl:cons ':trackpad_touch0_active (trackpad_touch0_active msg))
    (cl:cons ':trackpad_touch0_x (trackpad_touch0_x msg))
    (cl:cons ':trackpad_touch0_y (trackpad_touch0_y msg))
    (cl:cons ':trackpad_touch1_id (trackpad_touch1_id msg))
    (cl:cons ':trackpad_touch1_active (trackpad_touch1_active msg))
    (cl:cons ':trackpad_touch1_x (trackpad_touch1_x msg))
    (cl:cons ':trackpad_touch1_y (trackpad_touch1_y msg))
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':battery (battery msg))
    (cl:cons ':plug_usb (plug_usb msg))
    (cl:cons ':plug_audio (plug_audio msg))
    (cl:cons ':plug_mic (plug_mic msg))
))
