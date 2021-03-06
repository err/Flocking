(ns flocking.boid
  (:use [flocking.protocols]
	[rosado.processing]))

;; (def *boid-counter* (ref 0))
(def *boid-radius*       10)

(defrecord Boid [id pos vel]
  Identifiable
  (id   [x] id)

  Particle
  (mass [x]   1)
  (pos  [x] pos)
  (vel  [x] vel)
  ;; (acc  [x] acc)
  (dir  [x]
	(let [[x y] vel]
	  (Math/atan2 y x)))
  Radial
  (radius [x] *boid-radius*)

  Drawable
  ;; (draw-obj [x]
  ;; 	    (let [[xp yp] pos
  ;; 		  r (radius x)]
  ;; 	      (fill-float 150 88 220)
  ;; 	      (ellipse-mode RADIUS)
  ;; 	      (with-translation [xp yp]
  ;; 		(with-rotation  [(dir x)]
  ;; 		  (stroke-weight 2)
  ;; 		  (line 0 0 0 (* 2 r))
  ;; 		  (ellipse 0 0 r r)
  ;; 		  (with-translation [0 (* 2 r)]
  ;; 		    (with-rotation [QUARTER_PI]
  ;; 		      (line 0 0 0 (/ r 3)))
  ;; 		    (with-rotation [(- QUARTER_PI)]
  ;; 		      (line 0 0 0 (/ r 3))))))))
  )

(defn make-boid [id pos vel]
  (Boid. id pos vel))
