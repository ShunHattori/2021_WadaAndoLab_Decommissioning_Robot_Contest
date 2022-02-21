;; Auto-generated. Do not edit!


(when (boundp 'ds4_driver::Report)
  (if (not (find-package "DS4_DRIVER"))
    (make-package "DS4_DRIVER"))
  (shadow 'Report (find-package "DS4_DRIVER")))
(unless (find-package "DS4_DRIVER::REPORT")
  (make-package "DS4_DRIVER::REPORT"))

(in-package "ROS")
;;//! \htmlinclude Report.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ds4_driver::Report
  :super ros::object
  :slots (_header _left_analog_x _left_analog_y _right_analog_x _right_analog_y _l2_analog _r2_analog _dpad_up _dpad_down _dpad_left _dpad_right _button_cross _button_circle _button_square _button_triangle _button_l1 _button_l2 _button_l3 _button_r1 _button_r2 _button_r3 _button_share _button_options _button_trackpad _button_ps _lin_acc_x _lin_acc_y _lin_acc_z _ang_vel_x _ang_vel_y _ang_vel_z _trackpad_touch0_id _trackpad_touch0_active _trackpad_touch0_x _trackpad_touch0_y _trackpad_touch1_id _trackpad_touch1_active _trackpad_touch1_x _trackpad_touch1_y _timestamp _battery _plug_usb _plug_audio _plug_mic ))

(defmethod ds4_driver::Report
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:left_analog_x __left_analog_x) 0)
    ((:left_analog_y __left_analog_y) 0)
    ((:right_analog_x __right_analog_x) 0)
    ((:right_analog_y __right_analog_y) 0)
    ((:l2_analog __l2_analog) 0)
    ((:r2_analog __r2_analog) 0)
    ((:dpad_up __dpad_up) nil)
    ((:dpad_down __dpad_down) nil)
    ((:dpad_left __dpad_left) nil)
    ((:dpad_right __dpad_right) nil)
    ((:button_cross __button_cross) nil)
    ((:button_circle __button_circle) nil)
    ((:button_square __button_square) nil)
    ((:button_triangle __button_triangle) nil)
    ((:button_l1 __button_l1) nil)
    ((:button_l2 __button_l2) nil)
    ((:button_l3 __button_l3) nil)
    ((:button_r1 __button_r1) nil)
    ((:button_r2 __button_r2) nil)
    ((:button_r3 __button_r3) nil)
    ((:button_share __button_share) nil)
    ((:button_options __button_options) nil)
    ((:button_trackpad __button_trackpad) nil)
    ((:button_ps __button_ps) nil)
    ((:lin_acc_x __lin_acc_x) 0)
    ((:lin_acc_y __lin_acc_y) 0)
    ((:lin_acc_z __lin_acc_z) 0)
    ((:ang_vel_x __ang_vel_x) 0)
    ((:ang_vel_y __ang_vel_y) 0)
    ((:ang_vel_z __ang_vel_z) 0)
    ((:trackpad_touch0_id __trackpad_touch0_id) 0)
    ((:trackpad_touch0_active __trackpad_touch0_active) 0)
    ((:trackpad_touch0_x __trackpad_touch0_x) 0)
    ((:trackpad_touch0_y __trackpad_touch0_y) 0)
    ((:trackpad_touch1_id __trackpad_touch1_id) 0)
    ((:trackpad_touch1_active __trackpad_touch1_active) 0)
    ((:trackpad_touch1_x __trackpad_touch1_x) 0)
    ((:trackpad_touch1_y __trackpad_touch1_y) 0)
    ((:timestamp __timestamp) 0)
    ((:battery __battery) 0)
    ((:plug_usb __plug_usb) nil)
    ((:plug_audio __plug_audio) nil)
    ((:plug_mic __plug_mic) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _left_analog_x (round __left_analog_x))
   (setq _left_analog_y (round __left_analog_y))
   (setq _right_analog_x (round __right_analog_x))
   (setq _right_analog_y (round __right_analog_y))
   (setq _l2_analog (round __l2_analog))
   (setq _r2_analog (round __r2_analog))
   (setq _dpad_up __dpad_up)
   (setq _dpad_down __dpad_down)
   (setq _dpad_left __dpad_left)
   (setq _dpad_right __dpad_right)
   (setq _button_cross __button_cross)
   (setq _button_circle __button_circle)
   (setq _button_square __button_square)
   (setq _button_triangle __button_triangle)
   (setq _button_l1 __button_l1)
   (setq _button_l2 __button_l2)
   (setq _button_l3 __button_l3)
   (setq _button_r1 __button_r1)
   (setq _button_r2 __button_r2)
   (setq _button_r3 __button_r3)
   (setq _button_share __button_share)
   (setq _button_options __button_options)
   (setq _button_trackpad __button_trackpad)
   (setq _button_ps __button_ps)
   (setq _lin_acc_x (round __lin_acc_x))
   (setq _lin_acc_y (round __lin_acc_y))
   (setq _lin_acc_z (round __lin_acc_z))
   (setq _ang_vel_x (round __ang_vel_x))
   (setq _ang_vel_y (round __ang_vel_y))
   (setq _ang_vel_z (round __ang_vel_z))
   (setq _trackpad_touch0_id (round __trackpad_touch0_id))
   (setq _trackpad_touch0_active (round __trackpad_touch0_active))
   (setq _trackpad_touch0_x (round __trackpad_touch0_x))
   (setq _trackpad_touch0_y (round __trackpad_touch0_y))
   (setq _trackpad_touch1_id (round __trackpad_touch1_id))
   (setq _trackpad_touch1_active (round __trackpad_touch1_active))
   (setq _trackpad_touch1_x (round __trackpad_touch1_x))
   (setq _trackpad_touch1_y (round __trackpad_touch1_y))
   (setq _timestamp (round __timestamp))
   (setq _battery (round __battery))
   (setq _plug_usb __plug_usb)
   (setq _plug_audio __plug_audio)
   (setq _plug_mic __plug_mic)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:left_analog_x
   (&optional __left_analog_x)
   (if __left_analog_x (setq _left_analog_x __left_analog_x)) _left_analog_x)
  (:left_analog_y
   (&optional __left_analog_y)
   (if __left_analog_y (setq _left_analog_y __left_analog_y)) _left_analog_y)
  (:right_analog_x
   (&optional __right_analog_x)
   (if __right_analog_x (setq _right_analog_x __right_analog_x)) _right_analog_x)
  (:right_analog_y
   (&optional __right_analog_y)
   (if __right_analog_y (setq _right_analog_y __right_analog_y)) _right_analog_y)
  (:l2_analog
   (&optional __l2_analog)
   (if __l2_analog (setq _l2_analog __l2_analog)) _l2_analog)
  (:r2_analog
   (&optional __r2_analog)
   (if __r2_analog (setq _r2_analog __r2_analog)) _r2_analog)
  (:dpad_up
   (&optional (__dpad_up :null))
   (if (not (eq __dpad_up :null)) (setq _dpad_up __dpad_up)) _dpad_up)
  (:dpad_down
   (&optional (__dpad_down :null))
   (if (not (eq __dpad_down :null)) (setq _dpad_down __dpad_down)) _dpad_down)
  (:dpad_left
   (&optional (__dpad_left :null))
   (if (not (eq __dpad_left :null)) (setq _dpad_left __dpad_left)) _dpad_left)
  (:dpad_right
   (&optional (__dpad_right :null))
   (if (not (eq __dpad_right :null)) (setq _dpad_right __dpad_right)) _dpad_right)
  (:button_cross
   (&optional (__button_cross :null))
   (if (not (eq __button_cross :null)) (setq _button_cross __button_cross)) _button_cross)
  (:button_circle
   (&optional (__button_circle :null))
   (if (not (eq __button_circle :null)) (setq _button_circle __button_circle)) _button_circle)
  (:button_square
   (&optional (__button_square :null))
   (if (not (eq __button_square :null)) (setq _button_square __button_square)) _button_square)
  (:button_triangle
   (&optional (__button_triangle :null))
   (if (not (eq __button_triangle :null)) (setq _button_triangle __button_triangle)) _button_triangle)
  (:button_l1
   (&optional (__button_l1 :null))
   (if (not (eq __button_l1 :null)) (setq _button_l1 __button_l1)) _button_l1)
  (:button_l2
   (&optional (__button_l2 :null))
   (if (not (eq __button_l2 :null)) (setq _button_l2 __button_l2)) _button_l2)
  (:button_l3
   (&optional (__button_l3 :null))
   (if (not (eq __button_l3 :null)) (setq _button_l3 __button_l3)) _button_l3)
  (:button_r1
   (&optional (__button_r1 :null))
   (if (not (eq __button_r1 :null)) (setq _button_r1 __button_r1)) _button_r1)
  (:button_r2
   (&optional (__button_r2 :null))
   (if (not (eq __button_r2 :null)) (setq _button_r2 __button_r2)) _button_r2)
  (:button_r3
   (&optional (__button_r3 :null))
   (if (not (eq __button_r3 :null)) (setq _button_r3 __button_r3)) _button_r3)
  (:button_share
   (&optional (__button_share :null))
   (if (not (eq __button_share :null)) (setq _button_share __button_share)) _button_share)
  (:button_options
   (&optional (__button_options :null))
   (if (not (eq __button_options :null)) (setq _button_options __button_options)) _button_options)
  (:button_trackpad
   (&optional (__button_trackpad :null))
   (if (not (eq __button_trackpad :null)) (setq _button_trackpad __button_trackpad)) _button_trackpad)
  (:button_ps
   (&optional (__button_ps :null))
   (if (not (eq __button_ps :null)) (setq _button_ps __button_ps)) _button_ps)
  (:lin_acc_x
   (&optional __lin_acc_x)
   (if __lin_acc_x (setq _lin_acc_x __lin_acc_x)) _lin_acc_x)
  (:lin_acc_y
   (&optional __lin_acc_y)
   (if __lin_acc_y (setq _lin_acc_y __lin_acc_y)) _lin_acc_y)
  (:lin_acc_z
   (&optional __lin_acc_z)
   (if __lin_acc_z (setq _lin_acc_z __lin_acc_z)) _lin_acc_z)
  (:ang_vel_x
   (&optional __ang_vel_x)
   (if __ang_vel_x (setq _ang_vel_x __ang_vel_x)) _ang_vel_x)
  (:ang_vel_y
   (&optional __ang_vel_y)
   (if __ang_vel_y (setq _ang_vel_y __ang_vel_y)) _ang_vel_y)
  (:ang_vel_z
   (&optional __ang_vel_z)
   (if __ang_vel_z (setq _ang_vel_z __ang_vel_z)) _ang_vel_z)
  (:trackpad_touch0_id
   (&optional __trackpad_touch0_id)
   (if __trackpad_touch0_id (setq _trackpad_touch0_id __trackpad_touch0_id)) _trackpad_touch0_id)
  (:trackpad_touch0_active
   (&optional __trackpad_touch0_active)
   (if __trackpad_touch0_active (setq _trackpad_touch0_active __trackpad_touch0_active)) _trackpad_touch0_active)
  (:trackpad_touch0_x
   (&optional __trackpad_touch0_x)
   (if __trackpad_touch0_x (setq _trackpad_touch0_x __trackpad_touch0_x)) _trackpad_touch0_x)
  (:trackpad_touch0_y
   (&optional __trackpad_touch0_y)
   (if __trackpad_touch0_y (setq _trackpad_touch0_y __trackpad_touch0_y)) _trackpad_touch0_y)
  (:trackpad_touch1_id
   (&optional __trackpad_touch1_id)
   (if __trackpad_touch1_id (setq _trackpad_touch1_id __trackpad_touch1_id)) _trackpad_touch1_id)
  (:trackpad_touch1_active
   (&optional __trackpad_touch1_active)
   (if __trackpad_touch1_active (setq _trackpad_touch1_active __trackpad_touch1_active)) _trackpad_touch1_active)
  (:trackpad_touch1_x
   (&optional __trackpad_touch1_x)
   (if __trackpad_touch1_x (setq _trackpad_touch1_x __trackpad_touch1_x)) _trackpad_touch1_x)
  (:trackpad_touch1_y
   (&optional __trackpad_touch1_y)
   (if __trackpad_touch1_y (setq _trackpad_touch1_y __trackpad_touch1_y)) _trackpad_touch1_y)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:battery
   (&optional __battery)
   (if __battery (setq _battery __battery)) _battery)
  (:plug_usb
   (&optional (__plug_usb :null))
   (if (not (eq __plug_usb :null)) (setq _plug_usb __plug_usb)) _plug_usb)
  (:plug_audio
   (&optional (__plug_audio :null))
   (if (not (eq __plug_audio :null)) (setq _plug_audio __plug_audio)) _plug_audio)
  (:plug_mic
   (&optional (__plug_mic :null))
   (if (not (eq __plug_mic :null)) (setq _plug_mic __plug_mic)) _plug_mic)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _left_analog_x
    1
    ;; uint8 _left_analog_y
    1
    ;; uint8 _right_analog_x
    1
    ;; uint8 _right_analog_y
    1
    ;; uint8 _l2_analog
    1
    ;; uint8 _r2_analog
    1
    ;; bool _dpad_up
    1
    ;; bool _dpad_down
    1
    ;; bool _dpad_left
    1
    ;; bool _dpad_right
    1
    ;; bool _button_cross
    1
    ;; bool _button_circle
    1
    ;; bool _button_square
    1
    ;; bool _button_triangle
    1
    ;; bool _button_l1
    1
    ;; bool _button_l2
    1
    ;; bool _button_l3
    1
    ;; bool _button_r1
    1
    ;; bool _button_r2
    1
    ;; bool _button_r3
    1
    ;; bool _button_share
    1
    ;; bool _button_options
    1
    ;; bool _button_trackpad
    1
    ;; bool _button_ps
    1
    ;; int16 _lin_acc_x
    2
    ;; int16 _lin_acc_y
    2
    ;; int16 _lin_acc_z
    2
    ;; int16 _ang_vel_x
    2
    ;; int16 _ang_vel_y
    2
    ;; int16 _ang_vel_z
    2
    ;; uint16 _trackpad_touch0_id
    2
    ;; uint16 _trackpad_touch0_active
    2
    ;; uint16 _trackpad_touch0_x
    2
    ;; uint16 _trackpad_touch0_y
    2
    ;; uint16 _trackpad_touch1_id
    2
    ;; uint16 _trackpad_touch1_active
    2
    ;; uint16 _trackpad_touch1_x
    2
    ;; uint16 _trackpad_touch1_y
    2
    ;; uint8 _timestamp
    1
    ;; uint8 _battery
    1
    ;; bool _plug_usb
    1
    ;; bool _plug_audio
    1
    ;; bool _plug_mic
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _left_analog_x
       (write-byte _left_analog_x s)
     ;; uint8 _left_analog_y
       (write-byte _left_analog_y s)
     ;; uint8 _right_analog_x
       (write-byte _right_analog_x s)
     ;; uint8 _right_analog_y
       (write-byte _right_analog_y s)
     ;; uint8 _l2_analog
       (write-byte _l2_analog s)
     ;; uint8 _r2_analog
       (write-byte _r2_analog s)
     ;; bool _dpad_up
       (if _dpad_up (write-byte -1 s) (write-byte 0 s))
     ;; bool _dpad_down
       (if _dpad_down (write-byte -1 s) (write-byte 0 s))
     ;; bool _dpad_left
       (if _dpad_left (write-byte -1 s) (write-byte 0 s))
     ;; bool _dpad_right
       (if _dpad_right (write-byte -1 s) (write-byte 0 s))
     ;; bool _button_cross
       (if _button_cross (write-byte -1 s) (write-byte 0 s))
     ;; bool _button_circle
       (if _button_circle (write-byte -1 s) (write-byte 0 s))
     ;; bool _button_square
       (if _button_square (write-byte -1 s) (write-byte 0 s))
     ;; bool _button_triangle
       (if _button_triangle (write-byte -1 s) (write-byte 0 s))
     ;; bool _button_l1
       (if _button_l1 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button_l2
       (if _button_l2 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button_l3
       (if _button_l3 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button_r1
       (if _button_r1 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button_r2
       (if _button_r2 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button_r3
       (if _button_r3 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button_share
       (if _button_share (write-byte -1 s) (write-byte 0 s))
     ;; bool _button_options
       (if _button_options (write-byte -1 s) (write-byte 0 s))
     ;; bool _button_trackpad
       (if _button_trackpad (write-byte -1 s) (write-byte 0 s))
     ;; bool _button_ps
       (if _button_ps (write-byte -1 s) (write-byte 0 s))
     ;; int16 _lin_acc_x
       (write-word _lin_acc_x s)
     ;; int16 _lin_acc_y
       (write-word _lin_acc_y s)
     ;; int16 _lin_acc_z
       (write-word _lin_acc_z s)
     ;; int16 _ang_vel_x
       (write-word _ang_vel_x s)
     ;; int16 _ang_vel_y
       (write-word _ang_vel_y s)
     ;; int16 _ang_vel_z
       (write-word _ang_vel_z s)
     ;; uint16 _trackpad_touch0_id
       (write-word _trackpad_touch0_id s)
     ;; uint16 _trackpad_touch0_active
       (write-word _trackpad_touch0_active s)
     ;; uint16 _trackpad_touch0_x
       (write-word _trackpad_touch0_x s)
     ;; uint16 _trackpad_touch0_y
       (write-word _trackpad_touch0_y s)
     ;; uint16 _trackpad_touch1_id
       (write-word _trackpad_touch1_id s)
     ;; uint16 _trackpad_touch1_active
       (write-word _trackpad_touch1_active s)
     ;; uint16 _trackpad_touch1_x
       (write-word _trackpad_touch1_x s)
     ;; uint16 _trackpad_touch1_y
       (write-word _trackpad_touch1_y s)
     ;; uint8 _timestamp
       (write-byte _timestamp s)
     ;; uint8 _battery
       (write-byte _battery s)
     ;; bool _plug_usb
       (if _plug_usb (write-byte -1 s) (write-byte 0 s))
     ;; bool _plug_audio
       (if _plug_audio (write-byte -1 s) (write-byte 0 s))
     ;; bool _plug_mic
       (if _plug_mic (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _left_analog_x
     (setq _left_analog_x (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _left_analog_y
     (setq _left_analog_y (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _right_analog_x
     (setq _right_analog_x (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _right_analog_y
     (setq _right_analog_y (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _l2_analog
     (setq _l2_analog (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _r2_analog
     (setq _r2_analog (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _dpad_up
     (setq _dpad_up (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dpad_down
     (setq _dpad_down (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dpad_left
     (setq _dpad_left (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dpad_right
     (setq _dpad_right (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button_cross
     (setq _button_cross (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button_circle
     (setq _button_circle (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button_square
     (setq _button_square (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button_triangle
     (setq _button_triangle (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button_l1
     (setq _button_l1 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button_l2
     (setq _button_l2 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button_l3
     (setq _button_l3 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button_r1
     (setq _button_r1 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button_r2
     (setq _button_r2 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button_r3
     (setq _button_r3 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button_share
     (setq _button_share (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button_options
     (setq _button_options (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button_trackpad
     (setq _button_trackpad (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button_ps
     (setq _button_ps (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int16 _lin_acc_x
     (setq _lin_acc_x (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _lin_acc_y
     (setq _lin_acc_y (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _lin_acc_z
     (setq _lin_acc_z (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _ang_vel_x
     (setq _ang_vel_x (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _ang_vel_y
     (setq _ang_vel_y (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _ang_vel_z
     (setq _ang_vel_z (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _trackpad_touch0_id
     (setq _trackpad_touch0_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _trackpad_touch0_active
     (setq _trackpad_touch0_active (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _trackpad_touch0_x
     (setq _trackpad_touch0_x (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _trackpad_touch0_y
     (setq _trackpad_touch0_y (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _trackpad_touch1_id
     (setq _trackpad_touch1_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _trackpad_touch1_active
     (setq _trackpad_touch1_active (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _trackpad_touch1_x
     (setq _trackpad_touch1_x (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _trackpad_touch1_y
     (setq _trackpad_touch1_y (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _timestamp
     (setq _timestamp (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _battery
     (setq _battery (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _plug_usb
     (setq _plug_usb (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _plug_audio
     (setq _plug_audio (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _plug_mic
     (setq _plug_mic (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get ds4_driver::Report :md5sum-) "ec2c37165ced5aec5b7a50d72696b7dc")
(setf (get ds4_driver::Report :datatype-) "ds4_driver/Report")
(setf (get ds4_driver::Report :definition-)
      "# Raw report from DualShock 4
Header header

# Left: 0, Right: 255
uint8 left_analog_x
# Up: 0, Down: 255
uint8 left_analog_y
uint8 right_analog_x
uint8 right_analog_y

# Released: 0, Pressed: 255
uint8 l2_analog
# Released: 0, Pressed: 255
uint8 r2_analog

# Released: 0, Pressed: 1
bool dpad_up
bool dpad_down
bool dpad_left
bool dpad_right
bool button_cross
bool button_circle
bool button_square
bool button_triangle
bool button_l1
bool button_l2
bool button_l3
bool button_r1
bool button_r2
bool button_r3
bool button_share
bool button_options
bool button_trackpad
bool button_ps

# IMU
int16 lin_acc_x
int16 lin_acc_y
int16 lin_acc_z
int16 ang_vel_x
int16 ang_vel_y
int16 ang_vel_z

# Top-left: (0, 0), Bottom-right: (1919, 942)
uint16 trackpad_touch0_id
uint16 trackpad_touch0_active
uint16 trackpad_touch0_x
uint16 trackpad_touch0_y
uint16 trackpad_touch1_id
uint16 trackpad_touch1_active
uint16 trackpad_touch1_x
uint16 trackpad_touch1_y

uint8 timestamp
# Full: 8, Full (and charging): 11
uint8 battery

# Unused: 0, Plugged: 1
bool plug_usb
bool plug_audio
bool plug_mic

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

")



(provide :ds4_driver/Report "ec2c37165ced5aec5b7a50d72696b7dc")


