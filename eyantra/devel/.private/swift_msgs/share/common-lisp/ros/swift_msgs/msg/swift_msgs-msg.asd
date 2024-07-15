
(cl:in-package :asdf)

(defsystem "swift_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "swift_msgs" :depends-on ("_package_swift_msgs"))
    (:file "_package_swift_msgs" :depends-on ("_package"))
  ))