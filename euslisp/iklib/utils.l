;; python like functions

(defun zip (&rest lists)
  (apply #'mapcar #'list lists))

(defun range (n &optional (ret nil))
  (if (< n 1) 
      ret
      (range (- n 1) (cons (- n 1) ret ))))

(defun enumerate (lst)
  (let ((n (length lst)))
    (zip (list (range n) lst))))

(defun test ((i e))
  (print i)
  (print e))


(defun make-seq (start stepsize n)
  (let ((lst nil))
    (dotimes (i n)
      (setq lst (cons (+ start (* stepsize i)) lst)))
    (reverse lst)))

(defun vec-to-list (vec)
  (let ((lst-ret nil))
    (dotimes (i (length vec))
      (push [vec i] lst-ret))
    (reverse lst-ret)))

(defun lst->vec (lst)
  (apply #'float-vector lst))

(defun div (x y) 
  (round (/ (- x (mod x y)) y)))


