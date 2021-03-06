(ns flocking.utils
  (:use [flocking.socks]))



;; common-speak
(defn lesser-of [x y]
  (or (and (< x y) x) y))

(defn greater-of [x y]
  (or (and (> x y) x) y))

(defn vec-add [v1 v2]
  (let [[x y] v1
	[p q] v2]
    [(+ x p) (+ y q)]))

(defn vec-sub [v1 v2]
  (let [[x y] v1
	[p q] v2]
    [(- x p) (- y q)]))


;;;; these params produced really interesting schooling behavior
;; (def *min-speed*  0.15)
;; (def *max-speed*     6)
;; (def *max-accel*   1.1)

;; ;;;; RADII
;; ;;;
;; (def *fc-radius*   280) ;flock-centering radius
;; (def *ca-radius*   100) ;collision-avoidance radius
;; (def *vm-radius*   260) ;velocity-matching radius

;; ;;;; WEIGHTS
;; ;;; TODO: add knobs/sliders for
;; ;;; adjusting these parameters
;; (def *ca-weight*   30)
;; (def *vm-weight*   50)
;; (def *fc-weight*   50)
;; (def *wa-weight*   80)
;; (def *usr-weight*   4)