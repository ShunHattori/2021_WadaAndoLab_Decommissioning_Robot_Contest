;; Auto-generated. Do not edit!


(when (boundp 'commissioning_robot::ControlState)
  (if (not (find-package "COMMISSIONING_ROBOT"))
    (make-package "COMMISSIONING_ROBOT"))
  (shadow 'ControlState (find-package "COMMISSIONING_ROBOT")))
(unless (find-package "COMMISSIONING_ROBOT::CONTROLSTATE")
  (make-package "COMMISSIONING_ROBOT::CONTROLSTATE"))

(in-package "ROS")
;;//! \htmlinclude ControlState.msg.html


(defclass commissioning_robot::ControlState
  :super ros::object
  :slots (_mode _reference _manual _offset ))

(defmethod commissioning_robot::ControlState
  (:init
   (&key
    ((:mode __mode) (make-array 0 :initial-element 0 :element-type :char))
    ((:reference __reference) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:manual __manual) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:offset __offset) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _mode __mode)
   (setq _reference __reference)
   (setq _manual __manual)
   (setq _offset __offset)
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:reference
   (&optional __reference)
   (if __reference (setq _reference __reference)) _reference)
  (:manual
   (&optional __manual)
   (if __manual (setq _manual __manual)) _manual)
  (:offset
   (&optional __offset)
   (if __offset (setq _offset __offset)) _offset)
  (:serialization-length
   ()
   (+
    ;; uint8[] _mode
    (* 1    (length _mode)) 4
    ;; float64[] _reference
    (* 8    (length _reference)) 4
    ;; float64[] _manual
    (* 8    (length _manual)) 4
    ;; float64[] _offset
    (* 8    (length _offset)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[] _mode
     (write-long (length _mode) s)
     (princ _mode s)
     ;; float64[] _reference
     (write-long (length _reference) s)
     (dotimes (i (length _reference))
       (sys::poke (elt _reference i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _manual
     (write-long (length _manual) s)
     (dotimes (i (length _manual))
       (sys::poke (elt _manual i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _offset
     (write-long (length _offset) s)
     (dotimes (i (length _offset))
       (sys::poke (elt _offset i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[] _mode
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _mode (make-array n :element-type :char))
     (replace _mode buf :start2 ptr-) (incf ptr- n))
   ;; float64[] _reference
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _reference (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _reference i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _manual
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _manual (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _manual i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _offset
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _offset (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _offset i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get commissioning_robot::ControlState :md5sum-) "37618245187afeac8a81955f76bd2d07")
(setf (get commissioning_robot::ControlState :datatype-) "commissioning_robot/ControlState")
(setf (get commissioning_robot::ControlState :definition-)
      "uint8[] mode
float64[] reference
float64[] manual
float64[] offset


")



(provide :commissioning_robot/ControlState "37618245187afeac8a81955f76bd2d07")


