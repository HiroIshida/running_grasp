(load "package://pr2eus/pr2-interface.l")
(pr2-init)

(setq vec #f(179.752 48.6322 39.0744 63.0444 -101.161 124.202 -104.661 -16.8737 -49.602 -9.86737 -149.803 -79.6115 124.749 -49.2461 -179.442 -30.7553 39.57))

(send *pr2* :angle-vector vec)
(send *ri* :angle-vector (send *pr2* :angle-vector))


