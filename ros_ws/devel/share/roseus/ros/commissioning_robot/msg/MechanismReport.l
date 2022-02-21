;; Auto-generated. Do not edit!


(when (boundp 'commissioning_robot::MechanismReport)
  (if (not (find-package "COMMISSIONING_ROBOT"))
    (make-package "COMMISSIONING_ROBOT"))
  (shadow 'MechanismReport (find-package "COMMISSIONING_ROBOT")))
(unless (find-package "COMMISSIONING_ROBOT::MECHANISMREPORT")
  (make-package "COMMISSIONING_ROBOT::MECHANISMREPORT"))

(in-package "ROS")
;;//! \htmlinclude MechanismReport.msg.html


(defclass commissioning_robot::MechanismReport
  :super ros::object
  :slots (_running_phase _running_mode _state_limit _state_pulse _reference _current ))

(defmethod commissioning_robot::MechanismReport
  (:init
   (&key
    ((:running_phase __running_phase) 0)
    ((:running_mode __running_mode) (make-array 0 :initial-element 0 :element-type :char))
    ((:state_limit __state_limit) (make-array 0 :initial-element 0 :element-type :char))
    ((:state_pulse __state_pulse) (make-array 0 :initial-element 0 :element-type :integer))
    ((:reference __reference) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:current __current) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _running_phase (round __running_phase))
   (setq _running_mode __running_mode)
   (setq _state_limit __state_limit)
   (setq _state_pulse __state_pulse)
   (setq _reference __reference)
   (setq _current __current)
   self)
  (:running_phase
   (&optional __running_phase)
   (if __running_phase (setq _running_phase __running_phase)) _running_phase)
  (:running_mode
   (&optional __running_mode)
   (if __running_mode (setq _running_mode __running_mode)) _running_mode)
  (:state_limit
   (&optional __state_limit)
   (if __state_limit (setq _state_limit __state_limit)) _state_limit)
  (:state_pulse
   (&optional __state_pulse)
   (if __state_pulse (setq _state_pulse __state_pulse)) _state_pulse)
  (:reference
   (&optional __reference)
   (if __reference (setq _reference __reference)) _reference)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:serialization-length
   ()
   (+
    ;; uint8 _running_phase
    1
    ;; uint8[] _running_mode
    (* 1    (length _running_mode)) 4
    ;; uint8[] _state_limit
    (* 1    (length _state_limit)) 4
    ;; int64[] _state_pulse
    (* 8    (length _state_pulse)) 4
    ;; float64[] _reference
    (* 8    (length _reference)) 4
    ;; float64[] _current
    (* 8    (length _current)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _running_phase
       (write-byte _running_phase s)
     ;; uint8[] _running_mode
     (write-long (length _running_mode) s)
     (princ _running_mode s)
     ;; uint8[] _state_limit
     (write-long (length _state_limit) s)
     (princ _state_limit s)
     ;; int64[] _state_pulse
     (write-long (length _state_pulse) s)
     (dotimes (i (length _state_pulse))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _state_pulse i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _state_pulse i)) (= (length ((elt _state_pulse i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _state_pulse i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _state_pulse i) . bv) 1) -1) s))
             ((and (class (elt _state_pulse i)) (= (length ((elt _state_pulse i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _state_pulse i) . bv) 0) s)
              (write-long (if (>= (elt _state_pulse i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _state_pulse i) s)(write-long (if (>= (elt _state_pulse i) 0) 0 #xffffffff) s)))
       )
     ;; float64[] _reference
     (write-long (length _reference) s)
     (dotimes (i (length _reference))
       (sys::poke (elt _reference i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _current
     (write-long (length _current) s)
     (dotimes (i (length _current))
       (sys::poke (elt _current i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _running_phase
     (setq _running_phase (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8[] _running_mode
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _running_mode (make-array n :element-type :char))
     (replace _running_mode buf :start2 ptr-) (incf ptr- n))
   ;; uint8[] _state_limit
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _state_limit (make-array n :element-type :char))
     (replace _state_limit buf :start2 ptr-) (incf ptr- n))
   ;; int64[] _state_pulse
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _state_pulse (instantiate integer-vector n))
     (dotimes (i n)
#+(or :alpha :irix6 :x86_64)
      (setf (elt _state_pulse i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _state_pulse i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;; float64[] _reference
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _reference (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _reference i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _current
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _current (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _current i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get commissioning_robot::MechanismReport :md5sum-) "4efe817f8f990d484fcf48fdff687678")
(setf (get commissioning_robot::MechanismReport :datatype-) "commissioning_robot/MechanismReport")
(setf (get commissioning_robot::MechanismReport :definition-)
      "uint8 running_phase
uint8[] running_mode
uint8[] state_limit
int64[] state_pulse
float64[] reference
float64[] current

")



(provide :commissioning_robot/MechanismReport "4efe817f8f990d484fcf48fdff687678")


