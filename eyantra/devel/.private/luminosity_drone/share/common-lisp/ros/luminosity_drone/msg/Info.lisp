; Auto-generated. Do not edit!


(cl:in-package luminosity_drone-msg)


;//! \htmlinclude Info.msg.html

(cl:defclass <Info> (roslisp-msg-protocol:ros-message)
  ((sub_ms
    :reader sub_ms
    :initarg :sub_ms
    :type cl:integer
    :initform 0)
   (sub_mst
    :reader sub_mst
    :initarg :sub_mst
    :type cl:integer
    :initform 0)
   (sub_ls
    :reader sub_ls
    :initarg :sub_ls
    :type cl:integer
    :initform 0))
)

(cl:defclass Info (<Info>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Info>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Info)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name luminosity_drone-msg:<Info> is deprecated: use luminosity_drone-msg:Info instead.")))

(cl:ensure-generic-function 'sub_ms-val :lambda-list '(m))
(cl:defmethod sub_ms-val ((m <Info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader luminosity_drone-msg:sub_ms-val is deprecated.  Use luminosity_drone-msg:sub_ms instead.")
  (sub_ms m))

(cl:ensure-generic-function 'sub_mst-val :lambda-list '(m))
(cl:defmethod sub_mst-val ((m <Info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader luminosity_drone-msg:sub_mst-val is deprecated.  Use luminosity_drone-msg:sub_mst instead.")
  (sub_mst m))

(cl:ensure-generic-function 'sub_ls-val :lambda-list '(m))
(cl:defmethod sub_ls-val ((m <Info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader luminosity_drone-msg:sub_ls-val is deprecated.  Use luminosity_drone-msg:sub_ls instead.")
  (sub_ls m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Info>) ostream)
  "Serializes a message object of type '<Info>"
  (cl:let* ((signed (cl:slot-value msg 'sub_ms)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'sub_mst)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'sub_ls)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Info>) istream)
  "Deserializes a message object of type '<Info>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sub_ms) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sub_mst) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sub_ls) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Info>)))
  "Returns string type for a message object of type '<Info>"
  "luminosity_drone/Info")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Info)))
  "Returns string type for a message object of type 'Info"
  "luminosity_drone/Info")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Info>)))
  "Returns md5sum for a message object of type '<Info>"
  "fe8702018e11ba267a7b52a7fb524fc7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Info)))
  "Returns md5sum for a message object of type 'Info"
  "fe8702018e11ba267a7b52a7fb524fc7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Info>)))
  "Returns full string definition for message of type '<Info>"
  (cl:format cl:nil "int32 sub_ms~%int32 sub_mst~%int32 sub_ls~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Info)))
  "Returns full string definition for message of type 'Info"
  (cl:format cl:nil "int32 sub_ms~%int32 sub_mst~%int32 sub_ls~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Info>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Info>))
  "Converts a ROS message object to a list"
  (cl:list 'Info
    (cl:cons ':sub_ms (sub_ms msg))
    (cl:cons ':sub_mst (sub_mst msg))
    (cl:cons ':sub_ls (sub_ls msg))
))
