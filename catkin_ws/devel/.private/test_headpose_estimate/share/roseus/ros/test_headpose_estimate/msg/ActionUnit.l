;; Auto-generated. Do not edit!


(when (boundp 'test_headpose_estimate::ActionUnit)
  (if (not (find-package "TEST_HEADPOSE_ESTIMATE"))
    (make-package "TEST_HEADPOSE_ESTIMATE"))
  (shadow 'ActionUnit (find-package "TEST_HEADPOSE_ESTIMATE")))
(unless (find-package "TEST_HEADPOSE_ESTIMATE::ACTIONUNIT")
  (make-package "TEST_HEADPOSE_ESTIMATE::ACTIONUNIT"))

(in-package "ROS")
;;//! \htmlinclude ActionUnit.msg.html


(defclass test_headpose_estimate::ActionUnit
  :super ros::object
  :slots (_name _presence _intensity ))

(defmethod test_headpose_estimate::ActionUnit
  (:init
   (&key
    ((:name __name) "")
    ((:presence __presence) 0.0)
    ((:intensity __intensity) 0.0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _presence (float __presence))
   (setq _intensity (float __intensity))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:presence
   (&optional __presence)
   (if __presence (setq _presence __presence)) _presence)
  (:intensity
   (&optional __intensity)
   (if __intensity (setq _intensity __intensity)) _intensity)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; float64 _presence
    8
    ;; float64 _intensity
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float64 _presence
       (sys::poke _presence (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _intensity
       (sys::poke _intensity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _presence
     (setq _presence (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _intensity
     (setq _intensity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get test_headpose_estimate::ActionUnit :md5sum-) "aa8dca63187d7a12cf7a6c27efaaaa8d")
(setf (get test_headpose_estimate::ActionUnit :datatype-) "test_headpose_estimate/ActionUnit")
(setf (get test_headpose_estimate::ActionUnit :definition-)
      "string name
float64 presence
float64 intensity
")



(provide :test_headpose_estimate/ActionUnit "aa8dca63187d7a12cf7a6c27efaaaa8d")


