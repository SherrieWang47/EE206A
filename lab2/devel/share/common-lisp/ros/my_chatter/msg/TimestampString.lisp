; Auto-generated. Do not edit!


(cl:in-package my_chatter-msg)


;//! \htmlinclude TimestampString.msg.html

(cl:defclass <TimestampString> (roslisp-msg-protocol:ros-message)
  ((user_input
    :reader user_input
    :initarg :user_input
    :type cl:string
    :initform "")
   (time
    :reader time
    :initarg :time
    :type cl:float
    :initform 0.0))
)

(cl:defclass TimestampString (<TimestampString>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TimestampString>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TimestampString)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_chatter-msg:<TimestampString> is deprecated: use my_chatter-msg:TimestampString instead.")))

(cl:ensure-generic-function 'user_input-val :lambda-list '(m))
(cl:defmethod user_input-val ((m <TimestampString>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_chatter-msg:user_input-val is deprecated.  Use my_chatter-msg:user_input instead.")
  (user_input m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <TimestampString>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_chatter-msg:time-val is deprecated.  Use my_chatter-msg:time instead.")
  (time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TimestampString>) ostream)
  "Serializes a message object of type '<TimestampString>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'user_input))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'user_input))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TimestampString>) istream)
  "Deserializes a message object of type '<TimestampString>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'user_input) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'user_input) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TimestampString>)))
  "Returns string type for a message object of type '<TimestampString>"
  "my_chatter/TimestampString")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TimestampString)))
  "Returns string type for a message object of type 'TimestampString"
  "my_chatter/TimestampString")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TimestampString>)))
  "Returns md5sum for a message object of type '<TimestampString>"
  "ea60f95b56d48cb2880c573ef9019cdc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TimestampString)))
  "Returns md5sum for a message object of type 'TimestampString"
  "ea60f95b56d48cb2880c573ef9019cdc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TimestampString>)))
  "Returns full string definition for message of type '<TimestampString>"
  (cl:format cl:nil "string user_input~%float32 time~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TimestampString)))
  "Returns full string definition for message of type 'TimestampString"
  (cl:format cl:nil "string user_input~%float32 time~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TimestampString>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'user_input))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TimestampString>))
  "Converts a ROS message object to a list"
  (cl:list 'TimestampString
    (cl:cons ':user_input (user_input msg))
    (cl:cons ':time (time msg))
))
