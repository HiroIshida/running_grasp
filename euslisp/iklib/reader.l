(load "package://pr2eus/pr2-interface.l")
(load "package://roseus_mongo/euslisp/json/json-decode.l")
(load "utils.l")

(defun load-jsonfile (filename)
  (let* ((in (open filename :direction :input))
         (alist (json::parse-object in)))
    (close in)
    alist))

(defclass IKLib
  :slots (robot avs n x z stepsize ymin ymax))

(defmethod IKLib
  (:init (filename)
   (setq robot (pr2))
   (let* ((alist-load (load-jsonfile filename)))
     (setq avs 
           (apply #'vector 
                  (mapcar #'lst->vec (cdr (assoc :avs alist-load)))))
     (setq x (cdr (assoc :x alist-load)))
     (setq z (cdr (assoc :z alist-load)))
     (setq n (cdr (assoc :n alist-load)))
     (setq stepsize (cdr (assoc :stepsize alist-load)))
     (setq ymin (cdr (assoc :ymin alist-load)))
     (setq ymax (+ ymin (* (- n 1) stepsize)))))

  (:_get_neareset_index (y)
   (setq y (max y ymin))
   (setq y (min y ymax))
   (div (- y ymin) stepsize))

  (:solve (y)
   (let* ((idx (send self :_get_neareset_index y))
          (av-closest (aref avs idx)))
     (send robot :angle-vector av-closest)
     (send robot :rarm :inverse-kinematics 
           (make-coords :pos (float-vector x y z) :rpy #f(0 0 0))
           :rotation-axis t
           :use-torso nil))
   (send robot :angle-vector))
  )

(defun test ()
  (setq lib (instance IKLib :init))
  (pr2)
  (bench (send *pr2* :angle-vector (send lib :solve -700)))
  (objects *pr2*)
  )

