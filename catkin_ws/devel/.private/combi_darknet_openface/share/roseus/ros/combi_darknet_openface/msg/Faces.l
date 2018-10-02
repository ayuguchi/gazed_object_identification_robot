;; Auto-generated. Do not edit!


(when (boundp 'combi_darknet_openface::Faces)
  (if (not (find-package "COMBI_DARKNET_OPENFACE"))
    (make-package "COMBI_DARKNET_OPENFACE"))
  (shadow 'Faces (find-package "COMBI_DARKNET_OPENFACE")))
(unless (find-package "COMBI_DARKNET_OPENFACE::FACES")
  (make-package "COMBI_DARKNET_OPENFACE::FACES"))

(in-package "ROS")
;;//! \htmlinclude Faces.msg.html


(defclass combi_darknet_openface::Faces
  :super ros::object
  :slots (_faces ))

(defmethod combi_darknet_openface::Faces
  (:init
   (&key
    ((:faces __faces) (let (r) (dotimes (i 0) (push (instance combi_darknet_openface::Face :init) r)) r))
    )
   (send-super :init)
   (setq _faces __faces)
   self)
  (:faces
   (&rest __faces)
   (if (keywordp (car __faces))
       (send* _faces __faces)
     (progn
       (if __faces (setq _faces (car __faces)))
       _faces)))
  (:serialization-length
   ()
   (+
    ;; combi_darknet_openface/Face[] _faces
    (apply #'+ (send-all _faces :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; combi_darknet_openface/Face[] _faces
     (write-long (length _faces) s)
     (dolist (elem _faces)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; combi_darknet_openface/Face[] _faces
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _faces (let (r) (dotimes (i n) (push (instance combi_darknet_openface::Face :init) r)) r))
     (dolist (elem- _faces)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get combi_darknet_openface::Faces :md5sum-) "9efd106cd1ef1598e44e5a94142562d9")
(setf (get combi_darknet_openface::Faces :datatype-) "combi_darknet_openface/Faces")
(setf (get combi_darknet_openface::Faces :definition-)
      "Face[] faces
================================================================================
MSG: combi_darknet_openface/Face
std_msgs/Header header

geometry_msgs/Vector3 left_gaze
geometry_msgs/Vector3 right_gaze

geometry_msgs/Pose head_pose

geometry_msgs/Point[] landmarks_3d
geometry_msgs/Point[] landmarks_2d

#openface_ros/ActionUnit[] action_units

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :combi_darknet_openface/Faces "9efd106cd1ef1598e44e5a94142562d9")


