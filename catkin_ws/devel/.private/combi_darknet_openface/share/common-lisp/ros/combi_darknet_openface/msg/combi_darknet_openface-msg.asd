
(cl:in-package :asdf)

(defsystem "combi_darknet_openface-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ActionUnit" :depends-on ("_package_ActionUnit"))
    (:file "_package_ActionUnit" :depends-on ("_package"))
    (:file "Face" :depends-on ("_package_Face"))
    (:file "_package_Face" :depends-on ("_package"))
    (:file "Faces" :depends-on ("_package_Faces"))
    (:file "_package_Faces" :depends-on ("_package"))
  ))