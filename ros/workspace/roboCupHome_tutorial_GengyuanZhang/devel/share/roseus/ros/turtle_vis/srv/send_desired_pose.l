;; Auto-generated. Do not edit!


(when (boundp 'turtle_vis::send_desired_pose)
  (if (not (find-package "TURTLE_VIS"))
    (make-package "TURTLE_VIS"))
  (shadow 'send_desired_pose (find-package "TURTLE_VIS")))
(unless (find-package "TURTLE_VIS::SEND_DESIRED_POSE")
  (make-package "TURTLE_VIS::SEND_DESIRED_POSE"))
(unless (find-package "TURTLE_VIS::SEND_DESIRED_POSEREQUEST")
  (make-package "TURTLE_VIS::SEND_DESIRED_POSEREQUEST"))
(unless (find-package "TURTLE_VIS::SEND_DESIRED_POSERESPONSE")
  (make-package "TURTLE_VIS::SEND_DESIRED_POSERESPONSE"))

(in-package "ROS")





(defclass turtle_vis::send_desired_poseRequest
  :super ros::object
  :slots (_x _y _theta ))

(defmethod turtle_vis::send_desired_poseRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:theta __theta) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _theta (float __theta))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _theta
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta
     (setq _theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass turtle_vis::send_desired_poseResponse
  :super ros::object
  :slots (_reply ))

(defmethod turtle_vis::send_desired_poseResponse
  (:init
   (&key
    ((:reply __reply) 0)
    )
   (send-super :init)
   (setq _reply (round __reply))
   self)
  (:reply
   (&optional __reply)
   (if __reply (setq _reply __reply)) _reply)
  (:serialization-length
   ()
   (+
    ;; int64 _reply
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _reply
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _reply (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _reply) (= (length (_reply . bv)) 2)) ;; bignum
              (write-long (ash (elt (_reply . bv) 0) 0) s)
              (write-long (ash (elt (_reply . bv) 1) -1) s))
             ((and (class _reply) (= (length (_reply . bv)) 1)) ;; big1
              (write-long (elt (_reply . bv) 0) s)
              (write-long (if (>= _reply 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _reply s)(write-long (if (>= _reply 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _reply
#+(or :alpha :irix6 :x86_64)
      (setf _reply (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _reply (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass turtle_vis::send_desired_pose
  :super ros::object
  :slots ())

(setf (get turtle_vis::send_desired_pose :md5sum-) "e6eadbe1c0e5fb008aecf37722187400")
(setf (get turtle_vis::send_desired_pose :datatype-) "turtle_vis/send_desired_pose")
(setf (get turtle_vis::send_desired_pose :request) turtle_vis::send_desired_poseRequest)
(setf (get turtle_vis::send_desired_pose :response) turtle_vis::send_desired_poseResponse)

(defmethod turtle_vis::send_desired_poseRequest
  (:response () (instance turtle_vis::send_desired_poseResponse :init)))

(setf (get turtle_vis::send_desired_poseRequest :md5sum-) "e6eadbe1c0e5fb008aecf37722187400")
(setf (get turtle_vis::send_desired_poseRequest :datatype-) "turtle_vis/send_desired_poseRequest")
(setf (get turtle_vis::send_desired_poseRequest :definition-)
      "
float64 x
float64 y
float64 theta
---

int64 reply
")

(setf (get turtle_vis::send_desired_poseResponse :md5sum-) "e6eadbe1c0e5fb008aecf37722187400")
(setf (get turtle_vis::send_desired_poseResponse :datatype-) "turtle_vis/send_desired_poseResponse")
(setf (get turtle_vis::send_desired_poseResponse :definition-)
      "
float64 x
float64 y
float64 theta
---

int64 reply
")



(provide :turtle_vis/send_desired_pose "e6eadbe1c0e5fb008aecf37722187400")


