
(cl:in-package :asdf)

(defsystem "commissioning_robot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ControlState" :depends-on ("_package_ControlState"))
    (:file "_package_ControlState" :depends-on ("_package"))
    (:file "FeedbackState" :depends-on ("_package_FeedbackState"))
    (:file "_package_FeedbackState" :depends-on ("_package"))
    (:file "MechanismReport" :depends-on ("_package_MechanismReport"))
    (:file "_package_MechanismReport" :depends-on ("_package"))
    (:file "PhaseReport" :depends-on ("_package_PhaseReport"))
    (:file "_package_PhaseReport" :depends-on ("_package"))
  ))