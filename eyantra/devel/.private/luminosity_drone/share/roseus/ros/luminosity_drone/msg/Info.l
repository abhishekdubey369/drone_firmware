;; Auto-generated. Do not edit!


(when (boundp 'luminosity_drone::Info)
  (if (not (find-package "LUMINOSITY_DRONE"))
    (make-package "LUMINOSITY_DRONE"))
  (shadow 'Info (find-package "LUMINOSITY_DRONE")))
(unless (find-package "LUMINOSITY_DRONE::INFO")
  (make-package "LUMINOSITY_DRONE::INFO"))

(in-package "ROS")
;;//! \htmlinclude Info.msg.html


(defclass luminosity_drone::Info
  :super ros::object
  :slots (_sub_ms _sub_mst _sub_ls ))

(defmethod luminosity_drone::Info
  (:init
   (&key
    ((:sub_ms __sub_ms) 0)
    ((:sub_mst __sub_mst) 0)
    ((:sub_ls __sub_ls) 0)
    )
   (send-super :init)
   (setq _sub_ms (round __sub_ms))
   (setq _sub_mst (round __sub_mst))
   (setq _sub_ls (round __sub_ls))
   self)
  (:sub_ms
   (&optional __sub_ms)
   (if __sub_ms (setq _sub_ms __sub_ms)) _sub_ms)
  (:sub_mst
   (&optional __sub_mst)
   (if __sub_mst (setq _sub_mst __sub_mst)) _sub_mst)
  (:sub_ls
   (&optional __sub_ls)
   (if __sub_ls (setq _sub_ls __sub_ls)) _sub_ls)
  (:serialization-length
   ()
   (+
    ;; int32 _sub_ms
    4
    ;; int32 _sub_mst
    4
    ;; int32 _sub_ls
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _sub_ms
       (write-long _sub_ms s)
     ;; int32 _sub_mst
       (write-long _sub_mst s)
     ;; int32 _sub_ls
       (write-long _sub_ls s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _sub_ms
     (setq _sub_ms (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _sub_mst
     (setq _sub_mst (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _sub_ls
     (setq _sub_ls (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get luminosity_drone::Info :md5sum-) "fe8702018e11ba267a7b52a7fb524fc7")
(setf (get luminosity_drone::Info :datatype-) "luminosity_drone/Info")
(setf (get luminosity_drone::Info :definition-)
      "int32 sub_ms
int32 sub_mst
int32 sub_ls
")



(provide :luminosity_drone/Info "fe8702018e11ba267a7b52a7fb524fc7")


