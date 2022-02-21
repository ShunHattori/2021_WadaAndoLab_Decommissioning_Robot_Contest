;; Auto-generated. Do not edit!


(when (boundp 'ds4_driver::Trackpad)
  (if (not (find-package "DS4_DRIVER"))
    (make-package "DS4_DRIVER"))
  (shadow 'Trackpad (find-package "DS4_DRIVER")))
(unless (find-package "DS4_DRIVER::TRACKPAD")
  (make-package "DS4_DRIVER::TRACKPAD"))

(in-package "ROS")
;;//! \htmlinclude Trackpad.msg.html


(defclass ds4_driver::Trackpad
  :super ros::object
  :slots (_id _active _x _y ))

(defmethod ds4_driver::Trackpad
  (:init
   (&key
    ((:id __id) 0)
    ((:active __active) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _active (round __active))
   (setq _x (float __x))
   (setq _y (float __y))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:active
   (&optional __active)
   (if __active (setq _active __active)) _active)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; uint16 _id
    2
    ;; int32 _active
    4
    ;; float32 _x
    4
    ;; float32 _y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _id
       (write-word _id s)
     ;; int32 _active
       (write-long _active s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _id
     (setq _id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int32 _active
     (setq _active (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ds4_driver::Trackpad :md5sum-) "7f8d46ab2334dfb3664bed321f9eaf05")
(setf (get ds4_driver::Trackpad :datatype-) "ds4_driver/Trackpad")
(setf (get ds4_driver::Trackpad :definition-)
      "# Trackpad message for DualShock 4
uint16 id       # Touch ID (increments every touch)
int32 active    # 0: inactive, 1: active
float32 x       # 0.0: left edge, 1.0: right edge
float32 y       # 0.0: left edge, 1.0: right edge

")



(provide :ds4_driver/Trackpad "7f8d46ab2334dfb3664bed321f9eaf05")


