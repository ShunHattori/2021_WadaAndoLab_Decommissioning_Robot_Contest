
(cl:in-package :asdf)

(defsystem "ds4_driver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Feedback" :depends-on ("_package_Feedback"))
    (:file "_package_Feedback" :depends-on ("_package"))
    (:file "Report" :depends-on ("_package_Report"))
    (:file "_package_Report" :depends-on ("_package"))
    (:file "Status" :depends-on ("_package_Status"))
    (:file "_package_Status" :depends-on ("_package"))
    (:file "Trackpad" :depends-on ("_package_Trackpad"))
    (:file "_package_Trackpad" :depends-on ("_package"))
  ))