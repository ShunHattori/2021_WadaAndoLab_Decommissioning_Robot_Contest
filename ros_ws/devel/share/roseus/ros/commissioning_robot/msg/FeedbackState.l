;; Auto-generated. Do not edit!


(when (boundp 'commissioning_robot::FeedbackState)
  (if (not (find-package "COMMISSIONING_ROBOT"))
    (make-package "COMMISSIONING_ROBOT"))
  (shadow 'FeedbackState (find-package "COMMISSIONING_ROBOT")))
(unless (find-package "COMMISSIONING_ROBOT::FEEDBACKSTATE")
  (make-package "COMMISSIONING_ROBOT::FEEDBACKSTATE"))

(in-package "ROS")
;;//! \htmlinclude FeedbackState.msg.html


(defclass commissioning_robot::FeedbackState
  :super ros::object
  :slots (_is_ended _current _reference_feedbackside _mode_feedbackside ))

(defmethod commissioning_robot::FeedbackState
  (:init
   (&key
    ((:is_ended __is_ended) (make-array 0 :initial-element 0 :element-type :char))
    ((:current __current) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:reference_feedbackside __reference_feedbackside) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:mode_feedbackside __mode_feedbackside) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _is_ended __is_ended)
   (setq _current __current)
   (setq _reference_feedbackside __reference_feedbackside)
   (setq _mode_feedbackside __mode_feedbackside)
   self)
  (:is_ended
   (&optional __is_ended)
   (if __is_ended (setq _is_ended __is_ended)) _is_ended)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:reference_feedbackside
   (&optional __reference_feedbackside)
   (if __reference_feedbackside (setq _reference_feedbackside __reference_feedbackside)) _reference_feedbackside)
  (:mode_feedbackside
   (&optional __mode_feedbackside)
   (if __mode_feedbackside (setq _mode_feedbackside __mode_feedbackside)) _mode_feedbackside)
  (:serialization-length
   ()
   (+
    ;; uint8[] _is_ended
    (* 1    (length _is_ended)) 4
    ;; float64[] _current
    (* 8    (length _current)) 4
    ;; float64[] _reference_feedbackside
    (* 8    (length _reference_feedbackside)) 4
    ;; uint8[] _mode_feedbackside
    (* 1    (length _mode_feedbackside)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[] _is_ended
     (write-long (length _is_ended) s)
     (princ _is_ended s)
     ;; float64[] _current
     (write-long (length _current) s)
     (dotimes (i (length _current))
       (sys::poke (elt _current i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _reference_feedbackside
     (write-long (length _reference_feedbackside) s)
     (dotimes (i (length _reference_feedbackside))
       (sys::poke (elt _reference_feedbackside i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; uint8[] _mode_feedbackside
     (write-long (length _mode_feedbackside) s)
     (princ _mode_feedbackside s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[] _is_ended
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _is_ended (make-array n :element-type :char))
     (replace _is_ended buf :start2 ptr-) (incf ptr- n))
   ;; float64[] _current
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _current (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _current i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _reference_feedbackside
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _reference_feedbackside (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _reference_feedbackside i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; uint8[] _mode_feedbackside
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _mode_feedbackside (make-array n :element-type :char))
     (replace _mode_feedbackside buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get commissioning_robot::FeedbackState :md5sum-) "73c7091a2c6badbe1419f63c8f01c277")
(setf (get commissioning_robot::FeedbackState :datatype-) "commissioning_robot/FeedbackState")
(setf (get commissioning_robot::FeedbackState :definition-)
      "uint8[] is_ended
float64[] current
float64[] reference_feedbackside
uint8[] mode_feedbackside

")



(provide :commissioning_robot/FeedbackState "73c7091a2c6badbe1419f63c8f01c277")


