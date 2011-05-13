(ns flocking.protocols)

(defprotocol Particle
  (pos  [x] "[x y] coordinates")
  (vel  [x] "component vectors for velocity")
  ;; (acc  [x] "component vectors for acceleration")
  (dir  [x] "orientation of the object (in radians, where 0 is straight-up)")
  (mass [x] "mass of the physical object"))

(defprotocol Drawable
  (draw-obj [x] "draws object"))
(defprotocol Radial
  (radius [x] "radius of the object"))

(defprotocol Identifiable
  (id   [x] "id of the object"))
