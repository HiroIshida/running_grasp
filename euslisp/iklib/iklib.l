(load "package://pr2eus/pr2-interface.l")
(load "package://roseus_mongo/euslisp/json/json-encode.l")
(load "utils.l")

(pr2)

(defun generate-iklib (robot arm y-step-seq x z) 
  (let ((lst nil))
    (dolist (y y-step-seq)
      (send robot arm :inverse-kinematics
          (make-coords :pos (float-vector x y z) :rpy #f(0 0 0))
          :rotation-axis t :use-torso nil)
      (let* ((av (send robot :angle-vector)))
        (setq lst (cons av lst))))
    lst))


;; initial solution
(setq vec #f(179.752 48.6322 39.0744 63.0444 -101.161 124.202 -104.661 -16.8737 -49.602 -9.86737 -149.803 -79.6115 124.749 -49.2461 -179.442 -30.7553 39.57))
(send *pr2* :angle-vector vec)
(setq *pos-init* (send (send *pr2* :rarm :end-coords) :worldpos))

(let* ((ymin -800)
       (stepsize 20)
       (n 35)
       (ylist (make-seq ymin stepsize n))
       (avs (mapcar #'vec-to-list (generate-iklib *pr2* :rarm ylist [*pos-init* 0] [*pos-init* 2]))))
  (let ((data 
          `((:avs . ,avs)
            (:ymin . ,ymin)
            (:stepsize . ,stepsize)
            (:ylist . ,ylist)
            (:x . ,(aref *pos-init* 0))
            (:z . ,(aref *pos-init* 2))
            (:n . ,n))))
    (let ((out (open "tmp.json" :direction :output)))
      (json::encode-element data out)
      (close out))))

