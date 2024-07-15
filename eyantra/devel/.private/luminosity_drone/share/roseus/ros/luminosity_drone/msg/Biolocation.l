;; Auto-generated. Do not edit!


(when (boundp 'luminosity_drone::Biolocation)
  (if (not (find-package "LUMINOSITY_DRONE"))
    (make-package "LUMINOSITY_DRONE"))
  (shadow 'Biolocation (find-package "LUMINOSITY_DRONE")))
(unless (find-package "LUMINOSITY_DRONE::BIOLOCATION")
  (make-package "LUMINOSITY_DRONE::BIOLOCATION"))

(in-package "ROS")
;;//! \htmlinclude Biolocation.msg.html


(defclass luminosity_drone::Biolocation
  :super ros::object
  :slots (_organism_type _whycon_x _whycon_y _whycon_z ))

(defmethod luminosity_drone::Biolocation
  (:init
   (&key
    ((:organism_type __organism_type) "")
    ((:whycon_x __whycon_x) 0.0)
    ((:whycon_y __whycon_y) 0.0)
    ((:whycon_z __whycon_z) 0.0)
    )
   (send-super :init)
   (setq _organism_type (string __organism_type))
   (setq _whycon_x (float __whycon_x))
   (setq _whycon_y (float __whycon_y))
   (setq _whycon_z (float __whycon_z))
   self)
  (:organism_type
   (&optional __organism_type)
   (if __organism_type (setq _organism_type __organism_type)) _organism_type)
  (:whycon_x
   (&optional __whycon_x)
   (if __whycon_x (setq _whycon_x __whycon_x)) _whycon_x)
  (:whycon_y
   (&optional __whycon_y)
   (if __whycon_y (setq _whycon_y __whycon_y)) _whycon_y)
  (:whycon_z
   (&optional __whycon_z)
   (if __whycon_z (setq _whycon_z __whycon_z)) _whycon_z)
  (:serialization-length
   ()
   (+
    ;; string _organism_type
    4 (length _organism_type)
    ;; float64 _whycon_x
    8
    ;; float64 _whycon_y
    8
    ;; float64 _whycon_z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _organism_type
       (write-long (length _organism_type) s) (princ _organism_type s)
     ;; float64 _whycon_x
       (sys::poke _whycon_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _whycon_y
       (sys::poke _whycon_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _whycon_z
       (sys::poke _whycon_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _organism_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _organism_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _whycon_x
     (setq _whycon_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _whycon_y
     (setq _whycon_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _whycon_z
     (setq _whycon_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get luminosity_drone::Biolocation :md5sum-) "101e3ee1346387705dd3316e6b7bd9bd")
(setf (get luminosity_drone::Biolocation :datatype-) "luminosity_drone/Biolocation")
(setf (get luminosity_drone::Biolocation :definition-)
      "string organism_type
float64 whycon_x
float64 whycon_y
float64 whycon_z

")



(provide :luminosity_drone/Biolocation "101e3ee1346387705dd3316e6b7bd9bd")


