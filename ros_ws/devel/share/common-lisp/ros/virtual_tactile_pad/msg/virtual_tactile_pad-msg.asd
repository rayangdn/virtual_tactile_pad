
(cl:in-package :asdf)

(defsystem "virtual_tactile_pad-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ContactForce" :depends-on ("_package_ContactForce"))
    (:file "_package_ContactForce" :depends-on ("_package"))
  ))