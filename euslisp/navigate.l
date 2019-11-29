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

(defun compute-jacobian (robot arm)
  (let* ((link (cdr (send robot :link-list (send (send robot arm :end-coords) :parent))))
         (j (send robot :calc-jacobian-from-link-list link
                  :move-target (send robot arm :end-coords)
                  :rotation-axis nil
                  :translation-axis t))
         (j# (sr-inverse j)))
    j#))

(defun compute-diff (robot arm vec)
  (let* ((j (compute-jacobian robot arm))
         (av #f(0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0))
         (theta-diff (transform j vec)))
    (dotimes (i 7)
      (setf (aref av (+ 8 i)) (aref theta-diff i)))
    av))


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
  
(defun get-error (obj-pos)
  (let* ((arm-pos (first (get-end-effector-state *pr2* :rarm)))
         (e (v- (float-vector 636.9 (aref obj-pos 1) 765.8) arm-pos)))
    e))


(send *ri* :move-gripper :rarm 0.08 :wait t)
(speak-jp "三秒後に発進します")
(send *ri* :wait-interpolation)
(unix:sleep 2)

(defun feedback-inner (robot error-getter &optional (rate 100))
    ;(send *ri* :go-pos-unsafe-no-wait 1.5 0 0)
    (ros::rate rate)
    (let ((err-pre #f(0 0 0)))
      (loop
        (ros::spin-once)
        (send robot :angle-vector (send *ri* :state :potentio-vector))
        (let* ((err (funcall error-getter))
               (errd (scale rate (v- err err-pre)))
               (err-x (aref err 0))
               (err-y (aref err 1))
               (err-z (aref err 2))
               (errd-y (aref errd 1))
               (command (+ (* 0.6 err-y) (* 0.0 errd-y))))


          (when (> command 150) (setq command 150))
          (let ((av-new (send robot :angle-vector)))
            (setq av-new (v- av-new (scale command (compute-diff robot :rarm (float-vector 0 1 0)))))
            (setq av-new (v+ av-new (scale err-x (compute-diff robot :rarm (float-vector 0.5 0 0)))))
            (setq av-new (v- av-new (scale err-z (compute-diff robot :rarm (float-vector 0 0 0.5)))))
            (setf [av-new 15] -30.7553)
            (setf [av-new 16] 39.57)
            (send *ri* :angle-vector av-new 1000))
          (unix:usleep 20000)
          (setq err-pre err)))))

(feedback-inner *pr2* #'(lambda () (get-error *pos*)))

(send *pr2* :rarm :move-end-pos #f(0 0 100) :world)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)

