(ros::load-ros-manifest "roseus")
(ros::roseus "demo_main" :anonymous t)

(setq *pos* nil)

(defun rospos->eusvec (rospos)
  (apply #'float-vector
         (mapcar #'(lambda (key) (* (send rospos key) 1000))
                 '(:x :y :z))))

(ros::subscribe "/core/cluster_decomposer/centroid_pose_array" geometry_msgs::PoseArray
                #'(lambda (msg) 
                    (let ((pose (first (send msg :poses))))
                      (setq *pos* (rospos->eusvec (send pose :position))))))

(unix::sleep 2)
(ros::spin-once)

