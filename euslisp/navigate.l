(ros::load-ros-manifest "roseus")
(ros::roseus "demo_main" :anonymous t)
(load "package://pr2eus/pr2-interface.l")

(setq vec #f(179.752 48.6322 39.0744 63.0444 -101.161 124.202 -104.661 -16.8737 -49.602 -9.86737 -149.803 -79.6115 124.749 -49.2461 -179.442 -30.7553 39.57))

(setq *msg* nil)
(setq *pos* nil)
(defun rospos->eusvec (rospos)
  (apply #'float-vector
         (mapcar #'(lambda (key) (* (send rospos key) 1000))
                 '(:x :y :z))))

(ros::subscribe "/core/cluster_decomposer/centroid_pose_array" geometry_msgs::PoseArray
                #'(lambda (msg) 
                    (let ((pose (first (send msg :poses))))
                      (unless (null pose)
                        (setq *pos* (rospos->eusvec (send pose :position)))))))

(pr2-init)

(send *pr2* :rarm :move-end-pos #f(0 0 -100) :world)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)

(send *pr2* :angle-vector vec)
(send *ri* :angle-vector (send *pr2* :angle-vector))
(send *ri* :wait-interpolation)
(send *ri* :move-gripper :rarm 0.08 :wait t)
(send *ri* :wait-interpolation)
(loop 
  (unix:usleep 10000)
  (ros::spin-once)
  (unless (null *pos*) (return)))

(defun get-end-effector-state (robot which-arm)
  (let* ((coords-endeffector (send robot which-arm :end-coords))
         (rot-endeffector (send coords-endeffector :worldrot))
         (rpy-endeffector-tmp (car (rpy-angle rot-endeffector)))
         (rpy-endeffector (float-vector 
                            (elt rpy-endeffector-tmp 0)
                            (elt rpy-endeffector-tmp 1)
                            (elt rpy-endeffector-tmp 2)))
         (pos-endeffector (send coords-endeffector :worldpos)))
    (list pos-endeffector rpy-endeffector)))
  
(defun guide-arm-sim (obj-pos)
  (let* ((arm-pos (first (get-end-effector-state *pr2* :rarm)))
         (y-diff (- (aref obj-pos 1) (aref arm-pos 1))))
    (send *pr2* :rarm :move-end-pos (float-vector 0 y-diff 0) :world)))

(defun get-error (obj-pos)
  (let* ((arm-pos (first (get-end-effector-state *pr2* :rarm)))
         (e (v- obj-pos arm-pos)))
    e))


(send *ri* :move-gripper :rarm 0.08 :wait t)
(unix:sleep 2)
(loop
  (ros::spin-once)
  (let* ((e (get-error *pos*))
         (y-diff (aref e 1))
         (x-diff (aref e 0)))
    (send *pr2* :rarm :move-end-pos 
          (float-vector 0 (* y-diff 1.5) 0) :world)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (unix:usleep 50000)

    (when (< x-diff 180)
      (send *ri* :move-gripper :rarm 0.04 :wait t)
      (return))))

(send *pr2* :rarm :move-end-pos #f(0 0 100) :world)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)

