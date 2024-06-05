
(cl:in-package :asdf)

(defsystem "luminosity_drone-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Biolocation" :depends-on ("_package_Biolocation"))
    (:file "_package_Biolocation" :depends-on ("_package"))
    (:file "Info" :depends-on ("_package_Info"))
    (:file "_package_Info" :depends-on ("_package"))
  ))