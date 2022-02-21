;; Auto-generated. Do not edit!


(when (boundp 'ds4_driver::Status)
  (if (not (find-package "DS4_DRIVER"))
    (make-package "DS4_DRIVER"))
  (shadow 'Status (find-package "DS4_DRIVER")))
(unless (find-package "DS4_DRIVER::STATUS")
  (make-package "DS4_DRIVER::STATUS"))

(in-package "ROS")
;;//! \htmlinclude Status.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ds4_driver::Status
  :super ros::object
  :slots (_header _axis_left_x _axis_left_y _axis_right_x _axis_right_y _axis_l2 _axis_r2 _button_dpad_up _button_dpad_down _button_dpad_left _button_dpad_right _button_cross _button_circle _button_square _button_triangle _button_l1 _button_l2 _button_l3 _button_r1 _button_r2 _button_r3 _button_share _button_options _button_trackpad _button_ps _imu _battery_percentage _battery_full_charging _touch0 _touch1 _plug_usb _plug_audio _plug_mic ))

(defmethod ds4_driver::Status
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:axis_left_x __axis_left_x) 0.0)
    ((:axis_left_y __axis_left_y) 0.0)
    ((:axis_right_x __axis_right_x) 0.0)
    ((:axis_right_y __axis_right_y) 0.0)
    ((:axis_l2 __axis_l2) 0.0)
    ((:axis_r2 __axis_r2) 0.0)
    ((:button_dpad_up __button_dpad_up) 0)
    ((:button_dpad_down __button_dpad_down) 0)
    ((:button_dpad_left __button_dpad_left) 0)
    ((:button_dpad_right __button_dpad_right) 0)
    ((:button_cross __button_cross) 0)
    ((:button_circle __button_circle) 0)
    ((:button_square __button_square) 0)
    ((:button_triangle __button_triangle) 0)
    ((:button_l1 __button_l1) 0)
    ((:button_l2 __button_l2) 0)
    ((:button_l3 __button_l3) 0)
    ((:button_r1 __button_r1) 0)
    ((:button_r2 __button_r2) 0)
    ((:button_r3 __button_r3) 0)
    ((:button_share __button_share) 0)
    ((:button_options __button_options) 0)
    ((:button_trackpad __button_trackpad) 0)
    ((:button_ps __button_ps) 0)
    ((:imu __imu) (instance sensor_msgs::Imu :init))
    ((:battery_percentage __battery_percentage) 0.0)
    ((:battery_full_charging __battery_full_charging) 0)
    ((:touch0 __touch0) (instance ds4_driver::Trackpad :init))
    ((:touch1 __touch1) (instance ds4_driver::Trackpad :init))
    ((:plug_usb __plug_usb) 0)
    ((:plug_audio __plug_audio) 0)
    ((:plug_mic __plug_mic) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _axis_left_x (float __axis_left_x))
   (setq _axis_left_y (float __axis_left_y))
   (setq _axis_right_x (float __axis_right_x))
   (setq _axis_right_y (float __axis_right_y))
   (setq _axis_l2 (float __axis_l2))
   (setq _axis_r2 (float __axis_r2))
   (setq _button_dpad_up (round __button_dpad_up))
   (setq _button_dpad_down (round __button_dpad_down))
   (setq _button_dpad_left (round __button_dpad_left))
   (setq _button_dpad_right (round __button_dpad_right))
   (setq _button_cross (round __button_cross))
   (setq _button_circle (round __button_circle))
   (setq _button_square (round __button_square))
   (setq _button_triangle (round __button_triangle))
   (setq _button_l1 (round __button_l1))
   (setq _button_l2 (round __button_l2))
   (setq _button_l3 (round __button_l3))
   (setq _button_r1 (round __button_r1))
   (setq _button_r2 (round __button_r2))
   (setq _button_r3 (round __button_r3))
   (setq _button_share (round __button_share))
   (setq _button_options (round __button_options))
   (setq _button_trackpad (round __button_trackpad))
   (setq _button_ps (round __button_ps))
   (setq _imu __imu)
   (setq _battery_percentage (float __battery_percentage))
   (setq _battery_full_charging (round __battery_full_charging))
   (setq _touch0 __touch0)
   (setq _touch1 __touch1)
   (setq _plug_usb (round __plug_usb))
   (setq _plug_audio (round __plug_audio))
   (setq _plug_mic (round __plug_mic))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:axis_left_x
   (&optional __axis_left_x)
   (if __axis_left_x (setq _axis_left_x __axis_left_x)) _axis_left_x)
  (:axis_left_y
   (&optional __axis_left_y)
   (if __axis_left_y (setq _axis_left_y __axis_left_y)) _axis_left_y)
  (:axis_right_x
   (&optional __axis_right_x)
   (if __axis_right_x (setq _axis_right_x __axis_right_x)) _axis_right_x)
  (:axis_right_y
   (&optional __axis_right_y)
   (if __axis_right_y (setq _axis_right_y __axis_right_y)) _axis_right_y)
  (:axis_l2
   (&optional __axis_l2)
   (if __axis_l2 (setq _axis_l2 __axis_l2)) _axis_l2)
  (:axis_r2
   (&optional __axis_r2)
   (if __axis_r2 (setq _axis_r2 __axis_r2)) _axis_r2)
  (:button_dpad_up
   (&optional __button_dpad_up)
   (if __button_dpad_up (setq _button_dpad_up __button_dpad_up)) _button_dpad_up)
  (:button_dpad_down
   (&optional __button_dpad_down)
   (if __button_dpad_down (setq _button_dpad_down __button_dpad_down)) _button_dpad_down)
  (:button_dpad_left
   (&optional __button_dpad_left)
   (if __button_dpad_left (setq _button_dpad_left __button_dpad_left)) _button_dpad_left)
  (:button_dpad_right
   (&optional __button_dpad_right)
   (if __button_dpad_right (setq _button_dpad_right __button_dpad_right)) _button_dpad_right)
  (:button_cross
   (&optional __button_cross)
   (if __button_cross (setq _button_cross __button_cross)) _button_cross)
  (:button_circle
   (&optional __button_circle)
   (if __button_circle (setq _button_circle __button_circle)) _button_circle)
  (:button_square
   (&optional __button_square)
   (if __button_square (setq _button_square __button_square)) _button_square)
  (:button_triangle
   (&optional __button_triangle)
   (if __button_triangle (setq _button_triangle __button_triangle)) _button_triangle)
  (:button_l1
   (&optional __button_l1)
   (if __button_l1 (setq _button_l1 __button_l1)) _button_l1)
  (:button_l2
   (&optional __button_l2)
   (if __button_l2 (setq _button_l2 __button_l2)) _button_l2)
  (:button_l3
   (&optional __button_l3)
   (if __button_l3 (setq _button_l3 __button_l3)) _button_l3)
  (:button_r1
   (&optional __button_r1)
   (if __button_r1 (setq _button_r1 __button_r1)) _button_r1)
  (:button_r2
   (&optional __button_r2)
   (if __button_r2 (setq _button_r2 __button_r2)) _button_r2)
  (:button_r3
   (&optional __button_r3)
   (if __button_r3 (setq _button_r3 __button_r3)) _button_r3)
  (:button_share
   (&optional __button_share)
   (if __button_share (setq _button_share __button_share)) _button_share)
  (:button_options
   (&optional __button_options)
   (if __button_options (setq _button_options __button_options)) _button_options)
  (:button_trackpad
   (&optional __button_trackpad)
   (if __button_trackpad (setq _button_trackpad __button_trackpad)) _button_trackpad)
  (:button_ps
   (&optional __button_ps)
   (if __button_ps (setq _button_ps __button_ps)) _button_ps)
  (:imu
   (&rest __imu)
   (if (keywordp (car __imu))
       (send* _imu __imu)
     (progn
       (if __imu (setq _imu (car __imu)))
       _imu)))
  (:battery_percentage
   (&optional __battery_percentage)
   (if __battery_percentage (setq _battery_percentage __battery_percentage)) _battery_percentage)
  (:battery_full_charging
   (&optional __battery_full_charging)
   (if __battery_full_charging (setq _battery_full_charging __battery_full_charging)) _battery_full_charging)
  (:touch0
   (&rest __touch0)
   (if (keywordp (car __touch0))
       (send* _touch0 __touch0)
     (progn
       (if __touch0 (setq _touch0 (car __touch0)))
       _touch0)))
  (:touch1
   (&rest __touch1)
   (if (keywordp (car __touch1))
       (send* _touch1 __touch1)
     (progn
       (if __touch1 (setq _touch1 (car __touch1)))
       _touch1)))
  (:plug_usb
   (&optional __plug_usb)
   (if __plug_usb (setq _plug_usb __plug_usb)) _plug_usb)
  (:plug_audio
   (&optional __plug_audio)
   (if __plug_audio (setq _plug_audio __plug_audio)) _plug_audio)
  (:plug_mic
   (&optional __plug_mic)
   (if __plug_mic (setq _plug_mic __plug_mic)) _plug_mic)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _axis_left_x
    4
    ;; float32 _axis_left_y
    4
    ;; float32 _axis_right_x
    4
    ;; float32 _axis_right_y
    4
    ;; float32 _axis_l2
    4
    ;; float32 _axis_r2
    4
    ;; int32 _button_dpad_up
    4
    ;; int32 _button_dpad_down
    4
    ;; int32 _button_dpad_left
    4
    ;; int32 _button_dpad_right
    4
    ;; int32 _button_cross
    4
    ;; int32 _button_circle
    4
    ;; int32 _button_square
    4
    ;; int32 _button_triangle
    4
    ;; int32 _button_l1
    4
    ;; int32 _button_l2
    4
    ;; int32 _button_l3
    4
    ;; int32 _button_r1
    4
    ;; int32 _button_r2
    4
    ;; int32 _button_r3
    4
    ;; int32 _button_share
    4
    ;; int32 _button_options
    4
    ;; int32 _button_trackpad
    4
    ;; int32 _button_ps
    4
    ;; sensor_msgs/Imu _imu
    (send _imu :serialization-length)
    ;; float32 _battery_percentage
    4
    ;; int32 _battery_full_charging
    4
    ;; ds4_driver/Trackpad _touch0
    (send _touch0 :serialization-length)
    ;; ds4_driver/Trackpad _touch1
    (send _touch1 :serialization-length)
    ;; int32 _plug_usb
    4
    ;; int32 _plug_audio
    4
    ;; int32 _plug_mic
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _axis_left_x
       (sys::poke _axis_left_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _axis_left_y
       (sys::poke _axis_left_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _axis_right_x
       (sys::poke _axis_right_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _axis_right_y
       (sys::poke _axis_right_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _axis_l2
       (sys::poke _axis_l2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _axis_r2
       (sys::poke _axis_r2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _button_dpad_up
       (write-long _button_dpad_up s)
     ;; int32 _button_dpad_down
       (write-long _button_dpad_down s)
     ;; int32 _button_dpad_left
       (write-long _button_dpad_left s)
     ;; int32 _button_dpad_right
       (write-long _button_dpad_right s)
     ;; int32 _button_cross
       (write-long _button_cross s)
     ;; int32 _button_circle
       (write-long _button_circle s)
     ;; int32 _button_square
       (write-long _button_square s)
     ;; int32 _button_triangle
       (write-long _button_triangle s)
     ;; int32 _button_l1
       (write-long _button_l1 s)
     ;; int32 _button_l2
       (write-long _button_l2 s)
     ;; int32 _button_l3
       (write-long _button_l3 s)
     ;; int32 _button_r1
       (write-long _button_r1 s)
     ;; int32 _button_r2
       (write-long _button_r2 s)
     ;; int32 _button_r3
       (write-long _button_r3 s)
     ;; int32 _button_share
       (write-long _button_share s)
     ;; int32 _button_options
       (write-long _button_options s)
     ;; int32 _button_trackpad
       (write-long _button_trackpad s)
     ;; int32 _button_ps
       (write-long _button_ps s)
     ;; sensor_msgs/Imu _imu
       (send _imu :serialize s)
     ;; float32 _battery_percentage
       (sys::poke _battery_percentage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _battery_full_charging
       (write-long _battery_full_charging s)
     ;; ds4_driver/Trackpad _touch0
       (send _touch0 :serialize s)
     ;; ds4_driver/Trackpad _touch1
       (send _touch1 :serialize s)
     ;; int32 _plug_usb
       (write-long _plug_usb s)
     ;; int32 _plug_audio
       (write-long _plug_audio s)
     ;; int32 _plug_mic
       (write-long _plug_mic s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _axis_left_x
     (setq _axis_left_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _axis_left_y
     (setq _axis_left_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _axis_right_x
     (setq _axis_right_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _axis_right_y
     (setq _axis_right_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _axis_l2
     (setq _axis_l2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _axis_r2
     (setq _axis_r2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _button_dpad_up
     (setq _button_dpad_up (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _button_dpad_down
     (setq _button_dpad_down (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _button_dpad_left
     (setq _button_dpad_left (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _button_dpad_right
     (setq _button_dpad_right (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _button_cross
     (setq _button_cross (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _button_circle
     (setq _button_circle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _button_square
     (setq _button_square (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _button_triangle
     (setq _button_triangle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _button_l1
     (setq _button_l1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _button_l2
     (setq _button_l2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _button_l3
     (setq _button_l3 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _button_r1
     (setq _button_r1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _button_r2
     (setq _button_r2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _button_r3
     (setq _button_r3 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _button_share
     (setq _button_share (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _button_options
     (setq _button_options (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _button_trackpad
     (setq _button_trackpad (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _button_ps
     (setq _button_ps (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; sensor_msgs/Imu _imu
     (send _imu :deserialize buf ptr-) (incf ptr- (send _imu :serialization-length))
   ;; float32 _battery_percentage
     (setq _battery_percentage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _battery_full_charging
     (setq _battery_full_charging (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; ds4_driver/Trackpad _touch0
     (send _touch0 :deserialize buf ptr-) (incf ptr- (send _touch0 :serialization-length))
   ;; ds4_driver/Trackpad _touch1
     (send _touch1 :deserialize buf ptr-) (incf ptr- (send _touch1 :serialization-length))
   ;; int32 _plug_usb
     (setq _plug_usb (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _plug_audio
     (setq _plug_audio (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _plug_mic
     (setq _plug_mic (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ds4_driver::Status :md5sum-) "410582f2082b7ace5412b7159665c723")
(setf (get ds4_driver::Status :datatype-) "ds4_driver/Status")
(setf (get ds4_driver::Status :definition-)
      "# Human-readable and more ROS-compatible status of the device
Header header

# Stick
# Left: 1.0, Right: -1.0
float32 axis_left_x
# Up: 1.0, Down: -1.0
float32 axis_left_y
float32 axis_right_x
float32 axis_right_y

# Shoulder buttons [0, 1.0]
float32 axis_l2
float32 axis_r2

# Buttons (0: Not pressed, 1: Pressed)
int32 button_dpad_up
int32 button_dpad_down
int32 button_dpad_left
int32 button_dpad_right
int32 button_cross
int32 button_circle
int32 button_square
int32 button_triangle
int32 button_l1
int32 button_l2
int32 button_l3
int32 button_r1
int32 button_r2
int32 button_r3
int32 button_share
int32 button_options
int32 button_trackpad
int32 button_ps

# Gyro
sensor_msgs/Imu imu

# Battery
float32 battery_percentage  # [0.0, 1.0]
int32 battery_full_charging # 0: No, 1: Yes

# Trackpads
ds4_driver/Trackpad touch0
ds4_driver/Trackpad touch1

# Plugs
int32 plug_usb              # 0: No, 1: Yes
int32 plug_audio            # 0: No, 1: Yes
int32 plug_mic              # 0: No, 1: Yes

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: sensor_msgs/Imu
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the 
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
# estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each 
# covariance matrix, and disregard the associated estimate.

Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z 

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: ds4_driver/Trackpad
# Trackpad message for DualShock 4
uint16 id       # Touch ID (increments every touch)
int32 active    # 0: inactive, 1: active
float32 x       # 0.0: left edge, 1.0: right edge
float32 y       # 0.0: left edge, 1.0: right edge

")



(provide :ds4_driver/Status "410582f2082b7ace5412b7159665c723")


