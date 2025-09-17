;; Auto-generated. Do not edit!


(when (boundp 'object_information_msgs::Object)
  (if (not (find-package "OBJECT_INFORMATION_MSGS"))
    (make-package "OBJECT_INFORMATION_MSGS"))
  (shadow 'Object (find-package "OBJECT_INFORMATION_MSGS")))
(unless (find-package "OBJECT_INFORMATION_MSGS::OBJECT")
  (make-package "OBJECT_INFORMATION_MSGS::OBJECT"))

(in-package "ROS")
;;//! \htmlinclude Object.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass object_information_msgs::Object
  :super ros::object
  :slots (_header _detect_sequence _object_total _object_sequence _label _probability _position _size ))

(defmethod object_information_msgs::Object
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:detect_sequence __detect_sequence) 0)
    ((:object_total __object_total) 0)
    ((:object_sequence __object_sequence) 0)
    ((:label __label) "")
    ((:probability __probability) 0.0)
    ((:position __position) (instance geometry_msgs::Pose :init))
    ((:size __size) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _detect_sequence (round __detect_sequence))
   (setq _object_total (round __object_total))
   (setq _object_sequence (round __object_sequence))
   (setq _label (string __label))
   (setq _probability (float __probability))
   (setq _position __position)
   (setq _size __size)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:detect_sequence
   (&optional __detect_sequence)
   (if __detect_sequence (setq _detect_sequence __detect_sequence)) _detect_sequence)
  (:object_total
   (&optional __object_total)
   (if __object_total (setq _object_total __object_total)) _object_total)
  (:object_sequence
   (&optional __object_sequence)
   (if __object_sequence (setq _object_sequence __object_sequence)) _object_sequence)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:probability
   (&optional __probability)
   (if __probability (setq _probability __probability)) _probability)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:size
   (&rest __size)
   (if (keywordp (car __size))
       (send* _size __size)
     (progn
       (if __size (setq _size (car __size)))
       _size)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint64 _detect_sequence
    8
    ;; uint16 _object_total
    2
    ;; uint16 _object_sequence
    2
    ;; string _label
    4 (length _label)
    ;; float32 _probability
    4
    ;; geometry_msgs/Pose _position
    (send _position :serialization-length)
    ;; geometry_msgs/Vector3 _size
    (send _size :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint64 _detect_sequence
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _detect_sequence (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _detect_sequence) (= (length (_detect_sequence . bv)) 2)) ;; bignum
              (write-long (ash (elt (_detect_sequence . bv) 0) 0) s)
              (write-long (ash (elt (_detect_sequence . bv) 1) -1) s))
             ((and (class _detect_sequence) (= (length (_detect_sequence . bv)) 1)) ;; big1
              (write-long (elt (_detect_sequence . bv) 0) s)
              (write-long (if (>= _detect_sequence 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _detect_sequence s)(write-long (if (>= _detect_sequence 0) 0 #xffffffff) s)))
     ;; uint16 _object_total
       (write-word _object_total s)
     ;; uint16 _object_sequence
       (write-word _object_sequence s)
     ;; string _label
       (write-long (length _label) s) (princ _label s)
     ;; float32 _probability
       (sys::poke _probability (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Pose _position
       (send _position :serialize s)
     ;; geometry_msgs/Vector3 _size
       (send _size :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint64 _detect_sequence
     
#+(or :alpha :irix6 :x86_64)
      (setf _detect_sequence (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _detect_sequence (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint16 _object_total
     (setq _object_total (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _object_sequence
     (setq _object_sequence (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; string _label
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _label (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _probability
     (setq _probability (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Pose _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geometry_msgs/Vector3 _size
     (send _size :deserialize buf ptr-) (incf ptr- (send _size :serialization-length))
   ;;
   self)
  )

(setf (get object_information_msgs::Object :md5sum-) "1d8f12d51af0cd884816fbc62b3e1cd4")
(setf (get object_information_msgs::Object :datatype-) "object_information_msgs/Object")
(setf (get object_information_msgs::Object :definition-)
      "# Face bounding box with marker positions
Header header
uint64 detect_sequence
uint16 object_total
uint16 object_sequence
string label
float32 probability
geometry_msgs/Pose position
geometry_msgs/Vector3 size

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
string frame_id

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
")



(provide :object_information_msgs/Object "1d8f12d51af0cd884816fbc62b3e1cd4")


