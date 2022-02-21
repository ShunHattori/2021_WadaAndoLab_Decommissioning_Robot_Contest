;; Auto-generated. Do not edit!


(when (boundp 'ds4_driver::Feedback)
  (if (not (find-package "DS4_DRIVER"))
    (make-package "DS4_DRIVER"))
  (shadow 'Feedback (find-package "DS4_DRIVER")))
(unless (find-package "DS4_DRIVER::FEEDBACK")
  (make-package "DS4_DRIVER::FEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude Feedback.msg.html


(defclass ds4_driver::Feedback
  :super ros::object
  :slots (_set_led _led_r _led_g _led_b _set_led_flash _led_flash_on _led_flash_off _set_rumble _rumble_duration _rumble_small _rumble_big ))

(defmethod ds4_driver::Feedback
  (:init
   (&key
    ((:set_led __set_led) nil)
    ((:led_r __led_r) 0.0)
    ((:led_g __led_g) 0.0)
    ((:led_b __led_b) 0.0)
    ((:set_led_flash __set_led_flash) nil)
    ((:led_flash_on __led_flash_on) 0.0)
    ((:led_flash_off __led_flash_off) 0.0)
    ((:set_rumble __set_rumble) nil)
    ((:rumble_duration __rumble_duration) 0.0)
    ((:rumble_small __rumble_small) 0.0)
    ((:rumble_big __rumble_big) 0.0)
    )
   (send-super :init)
   (setq _set_led __set_led)
   (setq _led_r (float __led_r))
   (setq _led_g (float __led_g))
   (setq _led_b (float __led_b))
   (setq _set_led_flash __set_led_flash)
   (setq _led_flash_on (float __led_flash_on))
   (setq _led_flash_off (float __led_flash_off))
   (setq _set_rumble __set_rumble)
   (setq _rumble_duration (float __rumble_duration))
   (setq _rumble_small (float __rumble_small))
   (setq _rumble_big (float __rumble_big))
   self)
  (:set_led
   (&optional (__set_led :null))
   (if (not (eq __set_led :null)) (setq _set_led __set_led)) _set_led)
  (:led_r
   (&optional __led_r)
   (if __led_r (setq _led_r __led_r)) _led_r)
  (:led_g
   (&optional __led_g)
   (if __led_g (setq _led_g __led_g)) _led_g)
  (:led_b
   (&optional __led_b)
   (if __led_b (setq _led_b __led_b)) _led_b)
  (:set_led_flash
   (&optional (__set_led_flash :null))
   (if (not (eq __set_led_flash :null)) (setq _set_led_flash __set_led_flash)) _set_led_flash)
  (:led_flash_on
   (&optional __led_flash_on)
   (if __led_flash_on (setq _led_flash_on __led_flash_on)) _led_flash_on)
  (:led_flash_off
   (&optional __led_flash_off)
   (if __led_flash_off (setq _led_flash_off __led_flash_off)) _led_flash_off)
  (:set_rumble
   (&optional (__set_rumble :null))
   (if (not (eq __set_rumble :null)) (setq _set_rumble __set_rumble)) _set_rumble)
  (:rumble_duration
   (&optional __rumble_duration)
   (if __rumble_duration (setq _rumble_duration __rumble_duration)) _rumble_duration)
  (:rumble_small
   (&optional __rumble_small)
   (if __rumble_small (setq _rumble_small __rumble_small)) _rumble_small)
  (:rumble_big
   (&optional __rumble_big)
   (if __rumble_big (setq _rumble_big __rumble_big)) _rumble_big)
  (:serialization-length
   ()
   (+
    ;; bool _set_led
    1
    ;; float32 _led_r
    4
    ;; float32 _led_g
    4
    ;; float32 _led_b
    4
    ;; bool _set_led_flash
    1
    ;; float32 _led_flash_on
    4
    ;; float32 _led_flash_off
    4
    ;; bool _set_rumble
    1
    ;; float32 _rumble_duration
    4
    ;; float32 _rumble_small
    4
    ;; float32 _rumble_big
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _set_led
       (if _set_led (write-byte -1 s) (write-byte 0 s))
     ;; float32 _led_r
       (sys::poke _led_r (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _led_g
       (sys::poke _led_g (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _led_b
       (sys::poke _led_b (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _set_led_flash
       (if _set_led_flash (write-byte -1 s) (write-byte 0 s))
     ;; float32 _led_flash_on
       (sys::poke _led_flash_on (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _led_flash_off
       (sys::poke _led_flash_off (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _set_rumble
       (if _set_rumble (write-byte -1 s) (write-byte 0 s))
     ;; float32 _rumble_duration
       (sys::poke _rumble_duration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rumble_small
       (sys::poke _rumble_small (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rumble_big
       (sys::poke _rumble_big (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _set_led
     (setq _set_led (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _led_r
     (setq _led_r (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _led_g
     (setq _led_g (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _led_b
     (setq _led_b (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _set_led_flash
     (setq _set_led_flash (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _led_flash_on
     (setq _led_flash_on (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _led_flash_off
     (setq _led_flash_off (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _set_rumble
     (setq _set_rumble (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _rumble_duration
     (setq _rumble_duration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rumble_small
     (setq _rumble_small (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rumble_big
     (setq _rumble_big (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ds4_driver::Feedback :md5sum-) "e6ff655a0461aca2c8725df811fe5c33")
(setf (get ds4_driver::Feedback :datatype-) "ds4_driver/Feedback")
(setf (get ds4_driver::Feedback :definition-)
      "# Message file for controlling DualShock4
# LED intensities [0.0, 1.0]
bool set_led
float32 led_r
float32 led_g
float32 led_b

# LED flash durations in seconds (max is 2.5 sec)
bool set_led_flash
# Note: Set led_flash_off to 0 to stop flashing
float32 led_flash_on
float32 led_flash_off

# Rumble intensities [0.0, 1.0] (continues rumbling for 5 sec by default)
bool set_rumble
float32 rumble_duration
float32 rumble_small
float32 rumble_big

")



(provide :ds4_driver/Feedback "e6ff655a0461aca2c8725df811fe5c33")


