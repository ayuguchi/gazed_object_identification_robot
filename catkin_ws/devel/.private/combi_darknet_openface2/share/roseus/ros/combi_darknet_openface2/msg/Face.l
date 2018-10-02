;; Auto-generated. Do not edit!


(when (boundp 'combi_darknet_openface2::Face)
  (if (not (find-package "COMBI_DARKNET_OPENFACE2"))
    (make-package "COMBI_DARKNET_OPENFACE2"))
  (shadow 'Face (find-package "COMBI_DARKNET_OPENFACE2")))
(unless (find-package "COMBI_DARKNET_OPENFACE2::FACE")
  (make-package "COMBI_DARKNET_OPENFACE2::FACE"))

(in-package "ROS")
;;//! \htmlinclude Face.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass combi_darknet_openface2::Face
  :super ros::object
  :slots (_header _left_gaze _right_gaze _head_pose _landmarks_3d _landmarks_2d ))

(defmethod combi_darknet_openface2::Face
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:left_gaze __left_gaze) (instance geometry_msgs::Vector3 :init))
    ((:right_gaze __right_gaze) (instance geometry_msgs::Vector3 :init))
    ((:head_pose __head_pose) (instance geometry_msgs::Pose :init))
    ((:landmarks_3d __landmarks_3d) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:landmarks_2d __landmarks_2d) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _left_gaze __left_gaze)
   (setq _right_gaze __right_gaze)
   (setq _head_pose __head_pose)
   (setq _landmarks_3d __landmarks_3d)
   (setq _landmarks_2d __landmarks_2d)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:left_gaze
   (&rest __left_gaze)
   (if (keywordp (car __left_gaze))
       (send* _left_gaze __left_gaze)
     (progn
       (if __left_gaze (setq _left_gaze (car __left_gaze)))
       _left_gaze)))
  (:right_gaze
   (&rest __right_gaze)
   (if (keywordp (car __right_gaze))
       (send* _right_gaze __right_gaze)
     (progn
       (if __right_gaze (setq _right_gaze (car __right_gaze)))
       _right_gaze)))
  (:head_pose
   (&rest __head_pose)
   (if (keywordp (car __head_pose))
       (send* _head_pose __head_pose)
     (progn
       (if __head_pose (setq _head_pose (car __head_pose)))
       _head_pose)))
  (:landmarks_3d
   (&rest __landmarks_3d)
   (if (keywordp (car __landmarks_3d))
       (send* _landmarks_3d __landmarks_3d)
     (progn
       (if __landmarks_3d (setq _landmarks_3d (car __landmarks_3d)))
       _landmarks_3d)))
  (:landmarks_2d
   (&rest __landmarks_2d)
   (if (keywordp (car __landmarks_2d))
       (send* _landmarks_2d __landmarks_2d)
     (progn
       (if __landmarks_2d (setq _landmarks_2d (car __landmarks_2d)))
       _landmarks_2d)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Vector3 _left_gaze
    (send _left_gaze :serialization-length)
    ;; geometry_msgs/Vector3 _right_gaze
    (send _right_gaze :serialization-length)
    ;; geometry_msgs/Pose _head_pose
    (send _head_pose :serialization-length)
    ;; geometry_msgs/Point[] _landmarks_3d
    (apply #'+ (send-all _landmarks_3d :serialization-length)) 4
    ;; geometry_msgs/Point[] _landmarks_2d
    (apply #'+ (send-all _landmarks_2d :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Vector3 _left_gaze
       (send _left_gaze :serialize s)
     ;; geometry_msgs/Vector3 _right_gaze
       (send _right_gaze :serialize s)
     ;; geometry_msgs/Pose _head_pose
       (send _head_pose :serialize s)
     ;; geometry_msgs/Point[] _landmarks_3d
     (write-long (length _landmarks_3d) s)
     (dolist (elem _landmarks_3d)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _landmarks_2d
     (write-long (length _landmarks_2d) s)
     (dolist (elem _landmarks_2d)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Vector3 _left_gaze
     (send _left_gaze :deserialize buf ptr-) (incf ptr- (send _left_gaze :serialization-length))
   ;; geometry_msgs/Vector3 _right_gaze
     (send _right_gaze :deserialize buf ptr-) (incf ptr- (send _right_gaze :serialization-length))
   ;; geometry_msgs/Pose _head_pose
     (send _head_pose :deserialize buf ptr-) (incf ptr- (send _head_pose :serialization-length))
   ;; geometry_msgs/Point[] _landmarks_3d
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _landmarks_3d (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _landmarks_3d)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _landmarks_2d
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _landmarks_2d (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _landmarks_2d)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get combi_darknet_openface2::Face :md5sum-) "1621eee7fe600ba4c208a6533f5a249f")
(setf (get combi_darknet_openface2::Face :datatype-) "combi_darknet_openface2/Face")
(setf (get combi_darknet_openface2::Face :definition-)
      "std_msgs/Header header

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



(provide :combi_darknet_openface2/Face "1621eee7fe600ba4c208a6533f5a249f")


