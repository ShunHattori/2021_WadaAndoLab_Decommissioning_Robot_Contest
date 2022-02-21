;; Auto-generated. Do not edit!


(when (boundp 'commissioning_robot::PhaseReport)
  (if (not (find-package "COMMISSIONING_ROBOT"))
    (make-package "COMMISSIONING_ROBOT"))
  (shadow 'PhaseReport (find-package "COMMISSIONING_ROBOT")))
(unless (find-package "COMMISSIONING_ROBOT::PHASEREPORT")
  (make-package "COMMISSIONING_ROBOT::PHASEREPORT"))

(in-package "ROS")
;;//! \htmlinclude PhaseReport.msg.html


(defclass commissioning_robot::PhaseReport
  :super ros::object
  :slots (_phase _description ))

(defmethod commissioning_robot::PhaseReport
  (:init
   (&key
    ((:phase __phase) 0)
    ((:description __description) "")
    )
   (send-super :init)
   (setq _phase (round __phase))
   (setq _description (string __description))
   self)
  (:phase
   (&optional __phase)
   (if __phase (setq _phase __phase)) _phase)
  (:description
   (&optional __description)
   (if __description (setq _description __description)) _description)
  (:serialization-length
   ()
   (+
    ;; uint8 _phase
    1
    ;; string _description
    4 (length _description)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _phase
       (write-byte _phase s)
     ;; string _description
       (write-long (length _description) s) (princ _description s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _phase
     (setq _phase (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get commissioning_robot::PhaseReport :md5sum-) "73ce18b297c87cc885b69b7c94ec8474")
(setf (get commissioning_robot::PhaseReport :datatype-) "commissioning_robot/PhaseReport")
(setf (get commissioning_robot::PhaseReport :definition-)
      "uint8 phase
string description
")



(provide :commissioning_robot/PhaseReport "73ce18b297c87cc885b69b7c94ec8474")


