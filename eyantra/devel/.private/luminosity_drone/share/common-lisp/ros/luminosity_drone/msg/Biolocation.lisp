; Auto-generated. Do not edit!


(cl:in-package luminosity_drone-msg)


;//! \htmlinclude Biolocation.msg.html

(cl:defclass <Biolocation> (roslisp-msg-protocol:ros-message)
  ((organism_type
    :reader organism_type
    :initarg :organism_type
    :type cl:string
    :initform "")
   (whycon_x
    :reader whycon_x
    :initarg :whycon_x
    :type cl:float
    :initform 0.0)
   (whycon_y
    :reader whycon_y
    :initarg :whycon_y
    :type cl:float
    :initform 0.0)
   (whycon_z
    :reader whycon_z
    :initarg :whycon_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass Biolocation (<Biolocation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Biolocation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Biolocation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name luminosity_drone-msg:<Biolocation> is deprecated: use luminosity_drone-msg:Biolocation instead.")))

(cl:ensure-generic-function 'organism_type-val :lambda-list '(m))
(cl:defmethod organism_type-val ((m <Biolocation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader luminosity_drone-msg:organism_type-val is deprecated.  Use luminosity_drone-msg:organism_type instead.")
  (organism_type m))

(cl:ensure-generic-function 'whycon_x-val :lambda-list '(m))
(cl:defmethod whycon_x-val ((m <Biolocation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader luminosity_drone-msg:whycon_x-val is deprecated.  Use luminosity_drone-msg:whycon_x instead.")
  (whycon_x m))

(cl:ensure-generic-function 'whycon_y-val :lambda-list '(m))
(cl:defmethod whycon_y-val ((m <Biolocation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader luminosity_drone-msg:whycon_y-val is deprecated.  Use luminosity_drone-msg:whycon_y instead.")
  (whycon_y m))

(cl:ensure-generic-function 'whycon_z-val :lambda-list '(m))
(cl:defmethod whycon_z-val ((m <Biolocation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader luminosity_drone-msg:whycon_z-val is deprecated.  Use luminosity_drone-msg:whycon_z instead.")
  (whycon_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Biolocation>) ostream)
  "Serializes a message object of type '<Biolocation>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'organism_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'organism_type))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'whycon_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'whycon_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'whycon_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Biolocation>) istream)
  "Deserializes a message object of type '<Biolocation>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'organism_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'organism_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'whycon_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'whycon_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'whycon_z) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Biolocation>)))
  "Returns string type for a message object of type '<Biolocation>"
  "luminosity_drone/Biolocation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Biolocation)))
  "Returns string type for a message object of type 'Biolocation"
  "luminosity_drone/Biolocation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Biolocation>)))
  "Returns md5sum for a message object of type '<Biolocation>"
  "101e3ee1346387705dd3316e6b7bd9bd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Biolocation)))
  "Returns md5sum for a message object of type 'Biolocation"
  "101e3ee1346387705dd3316e6b7bd9bd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Biolocation>)))
  "Returns full string definition for message of type '<Biolocation>"
  (cl:format cl:nil "string organism_type~%float64 whycon_x~%float64 whycon_y~%float64 whycon_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Biolocation)))
  "Returns full string definition for message of type 'Biolocation"
  (cl:format cl:nil "string organism_type~%float64 whycon_x~%float64 whycon_y~%float64 whycon_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Biolocation>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'organism_type))
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Biolocation>))
  "Converts a ROS message object to a list"
  (cl:list 'Biolocation
    (cl:cons ':organism_type (organism_type msg))
    (cl:cons ':whycon_x (whycon_x msg))
    (cl:cons ':whycon_y (whycon_y msg))
    (cl:cons ':whycon_z (whycon_z msg))
))
