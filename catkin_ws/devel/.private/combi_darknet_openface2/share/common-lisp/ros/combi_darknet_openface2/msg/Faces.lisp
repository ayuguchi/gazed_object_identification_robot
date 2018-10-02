; Auto-generated. Do not edit!


(cl:in-package combi_darknet_openface2-msg)


;//! \htmlinclude Faces.msg.html

(cl:defclass <Faces> (roslisp-msg-protocol:ros-message)
  ((faces
    :reader faces
    :initarg :faces
    :type (cl:vector combi_darknet_openface2-msg:Face)
   :initform (cl:make-array 0 :element-type 'combi_darknet_openface2-msg:Face :initial-element (cl:make-instance 'combi_darknet_openface2-msg:Face))))
)

(cl:defclass Faces (<Faces>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Faces>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Faces)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name combi_darknet_openface2-msg:<Faces> is deprecated: use combi_darknet_openface2-msg:Faces instead.")))

(cl:ensure-generic-function 'faces-val :lambda-list '(m))
(cl:defmethod faces-val ((m <Faces>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader combi_darknet_openface2-msg:faces-val is deprecated.  Use combi_darknet_openface2-msg:faces instead.")
  (faces m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Faces>) ostream)
  "Serializes a message object of type '<Faces>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'faces))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'faces))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Faces>) istream)
  "Deserializes a message object of type '<Faces>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'faces) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'faces)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'combi_darknet_openface2-msg:Face))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Faces>)))
  "Returns string type for a message object of type '<Faces>"
  "combi_darknet_openface2/Faces")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Faces)))
  "Returns string type for a message object of type 'Faces"
  "combi_darknet_openface2/Faces")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Faces>)))
  "Returns md5sum for a message object of type '<Faces>"
  "9efd106cd1ef1598e44e5a94142562d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Faces)))
  "Returns md5sum for a message object of type 'Faces"
  "9efd106cd1ef1598e44e5a94142562d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Faces>)))
  "Returns full string definition for message of type '<Faces>"
  (cl:format cl:nil "Face[] faces~%================================================================================~%MSG: combi_darknet_openface2/Face~%std_msgs/Header header~%~%geometry_msgs/Vector3 left_gaze~%geometry_msgs/Vector3 right_gaze~%~%geometry_msgs/Pose head_pose~%~%geometry_msgs/Point[] landmarks_3d~%geometry_msgs/Point[] landmarks_2d~%~%#openface_ros/ActionUnit[] action_units~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Faces)))
  "Returns full string definition for message of type 'Faces"
  (cl:format cl:nil "Face[] faces~%================================================================================~%MSG: combi_darknet_openface2/Face~%std_msgs/Header header~%~%geometry_msgs/Vector3 left_gaze~%geometry_msgs/Vector3 right_gaze~%~%geometry_msgs/Pose head_pose~%~%geometry_msgs/Point[] landmarks_3d~%geometry_msgs/Point[] landmarks_2d~%~%#openface_ros/ActionUnit[] action_units~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Faces>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'faces) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Faces>))
  "Converts a ROS message object to a list"
  (cl:list 'Faces
    (cl:cons ':faces (faces msg))
))
