; Auto-generated. Do not edit!


(cl:in-package test_headpose_estimate-msg)


;//! \htmlinclude ActionUnit.msg.html

(cl:defclass <ActionUnit> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (presence
    :reader presence
    :initarg :presence
    :type cl:float
    :initform 0.0)
   (intensity
    :reader intensity
    :initarg :intensity
    :type cl:float
    :initform 0.0))
)

(cl:defclass ActionUnit (<ActionUnit>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActionUnit>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActionUnit)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name test_headpose_estimate-msg:<ActionUnit> is deprecated: use test_headpose_estimate-msg:ActionUnit instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ActionUnit>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader test_headpose_estimate-msg:name-val is deprecated.  Use test_headpose_estimate-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'presence-val :lambda-list '(m))
(cl:defmethod presence-val ((m <ActionUnit>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader test_headpose_estimate-msg:presence-val is deprecated.  Use test_headpose_estimate-msg:presence instead.")
  (presence m))

(cl:ensure-generic-function 'intensity-val :lambda-list '(m))
(cl:defmethod intensity-val ((m <ActionUnit>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader test_headpose_estimate-msg:intensity-val is deprecated.  Use test_headpose_estimate-msg:intensity instead.")
  (intensity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActionUnit>) ostream)
  "Serializes a message object of type '<ActionUnit>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'presence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'intensity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActionUnit>) istream)
  "Deserializes a message object of type '<ActionUnit>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'presence) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'intensity) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActionUnit>)))
  "Returns string type for a message object of type '<ActionUnit>"
  "test_headpose_estimate/ActionUnit")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActionUnit)))
  "Returns string type for a message object of type 'ActionUnit"
  "test_headpose_estimate/ActionUnit")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActionUnit>)))
  "Returns md5sum for a message object of type '<ActionUnit>"
  "aa8dca63187d7a12cf7a6c27efaaaa8d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActionUnit)))
  "Returns md5sum for a message object of type 'ActionUnit"
  "aa8dca63187d7a12cf7a6c27efaaaa8d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActionUnit>)))
  "Returns full string definition for message of type '<ActionUnit>"
  (cl:format cl:nil "string name~%float64 presence~%float64 intensity~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActionUnit)))
  "Returns full string definition for message of type 'ActionUnit"
  (cl:format cl:nil "string name~%float64 presence~%float64 intensity~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActionUnit>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActionUnit>))
  "Converts a ROS message object to a list"
  (cl:list 'ActionUnit
    (cl:cons ':name (name msg))
    (cl:cons ':presence (presence msg))
    (cl:cons ':intensity (intensity msg))
))
