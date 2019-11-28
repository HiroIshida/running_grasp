(load "package://pr2eus/pr2-interface.l")
(load "package://roseus_mongo/euslisp/json/json-decode.l")
(load "utils.l")

(defun load-jsonfile (filename)
  (let* ((in (open filename :direction :input))
         (alist (json::parse-object in)))
    (close in)
    alist))



(setq *alist-load* (load-jsonfile "tmp.json"))
(setq *avs*
      (apply #'vector 
             (mapcar #'lst->vec (cdr (assoc :avs *alist-load*)))))

(setq *ymin* (cdr (assoc :ymin *alist-load*)))
(setq *stepsize* (cdr (assoc :stepsize *alist-load*)))
(setq *n* (cdr (assoc :n *alist-load*)))
(setq *ylist* (cdr (assoc :ylist *alist-load*)))
(setq *x* (cdr (assoc :x *alist-load*)))
(setq *z* (cdr (assoc :z *alist-load*)))


(defun get-nearest-index (y)
  (setq y (max y *ymin*))
  (let ((ymax (+ *ymin* (* (- *n* 1) *stepsize*))))
    (setq y (min y ymax)))
  (div (- y *ymin*) *stepsize*))

(defun solve-from-nearest-av (robot arm y)
  (let* ((idx (get-nearest-index y))
         (av-closest (aref *avs* idx)))
    (send robot :angle-vector av-closest)
    (send robot arm :inverse-kinematics 
          (make-coords :pos (float-vector *x* y *z*) :rpy #f(0 0 0))
          :rotation-axis t
          :use-torso nil)))

(pr2)
(dolist (y (make-seq (- *ymin* 50) *stepsize* (+ *n* 10)))
  (bench (solve-from-nearest-av *pr2* :rarm (+ y 0.1))))

