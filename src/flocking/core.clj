(ns flocking.core
  (:use [flocking.boid]
	[flocking.utils]
	[flocking.protocols]
	[rosado.processing]
	[rosado.processing.applet]
	[flocking.socks])
  (:gen-class))


(def *screen-width*     800)
(def *screen-height*    800)
(def *panel-width*      800)
(def *panel-height*     860)
(def *bg-color*         255)
(def *time*        (atom 0))

(def *mouse-dn?*     (atom nil))
(def *mouse-position* (atom [0 0]))


(def *flock* (ref []))
(def *boid-count* (atom 0))

;; these flags are set in the toggle function
(def *fc-force?*   (atom true))
(def *vm-force?*   (atom true))
(def *ca-force?*   (atom true))
(def *wa-force?*   (atom true))
(def *attract?*    (atom :attract))
(def *show-path?*  (atom false))
(def *clear-once*  (atom false))
(def *running?*    (atom true))
(def *draw-fields* (atom false))


(defn forces []
  (map first
       (filter #(second %)
	       [[:cavoid @*ca-force?*]
		[:vmatch @*vm-force?*]
		[:center @*fc-force?*]
		[:wander @*wa-force?*]
		[:mouse  @*mouse-dn?*]])))



(def *default-flock-size*  10)
(def *maximum-flock-size* 100)

;; (def *min-speed*  0.15)
;; (def *max-speed*   9.4)
;; (def *max-accel*   3.1)
;; (def *max-attract*   3)

;; ;;;; RADII
;; ;;;
;; (def *fc-radius*     80) ;flock-centering radius
;; (def *ca-radius*     40) ;collision-avoidance radius
;; (def *vm-radius*     80) ;velocity-matching radius

;; ;;;; WEIGHTS
;; ;;; TODO: add knobs/sliders for
;; ;;; adjusting these parameters
;; (def *ca-weight*   1.40)
;; (def *vm-weight*   0.78)
;; (def *fc-weight*   0.60)
;; (def *wa-weight*   0.17)
;; (def *usr-weight*  3200)



(def *SLIDERS*                  (ref {}))
(def *GUI*     (make-frame "PARAMETERS"))

;; ui-tools
(defmacro defslider [ref val label]
  `(let [s# (slider #(dosync (ref-set ~ref %))
		   (range 0 (* 2 ~val) (if (integer? ~val) 1 0.01))
		   ~label
		   ~val)]
     (dosync (alter *SLIDERS* assoc (count @*SLIDERS*) s#))))

(defmacro defparam [name val]
  `(do
     (def ~name (ref ~val))
     (defslider ~name ~val (name '~name))))

(defmacro defparams [binds]
  `(do
     ~@(map (fn [[k v]]
	      `(defparam ~k ~v))
	    binds)))


(defparams [[*fc-radius*        80]
	    [*ca-radius*        40]
	    [*vm-radius*        80]
	    [*ca-weight*      1.00]
	    [*vm-weight*      1.00]
	    [*fc-weight*      1.00]
	    [*wa-weight*      1.00]
	    [*usr-weight*     1.00]
	    [*min-speed*      0.15]
	    [*max-speed*       9.5]
	    [*max-accel*       3.0]
	    [*max-attract*     3.0]])


(defn display-parameter-window []
  (.display *GUI* (apply stack (concat 
				(vals @*SLIDERS*)
				[(label "Press SPACE to (un)PAUSE")]))))


;;;; Status Messages
(def *font* (atom nil))

(defn status-msg []
  (apply str
	 (concat "Boids: "     (str (count @*flock*))
		 "  Centering: " (if @*fc-force?* "on" "off")
		 "  Velocity Matching: " (if @*vm-force?* "on" "off")
		 "  Collsions: " (if @*ca-force?* "on" "off")
		 "  Wandering: " (if @*wa-force?* "on" "off"))))

(defn draw-msg [x y]
  (with-translation [x y]
    (text-font @*font* 14)
    (text-align LEFT)
    (fill 250)
    (string->text (status-msg) 0 0)))

(defn make-random-boid [id]
  (let [pos  [(* *screen-width*  (rand 1))
	      (* *screen-height* (rand 1))]
	vel  [(+ (* (dec @*max-speed*) (dec (rand 2))) @*min-speed*)
	      (+ (* (dec @*max-speed*) (dec (rand 2))) @*min-speed*)]]
    (make-boid id pos vel)))

(defn init-flock
  ([]
     (init-flock *default-flock-size*))
  ([flock-size]
     (let [count @*boid-count*
	   size  (or (and (<= flock-size *maximum-flock-size*) flock-size)
		     *default-flock-size*)
	   flock (for [id (range count (+ count size))]
		   {id (make-random-boid id)})]
       (dosync (ref-set *flock* (reduce into flock)))
       (reset! *boid-count* size))))

(defn add-boid
  ([]
     (let [id @*boid-count*]
       (dosync (alter *flock* assoc id (make-random-boid id))
	       (swap! *boid-count* inc)))))

(defn remove-boid
  ([]
     (let [id (rand-nth (keys @*flock*))]
       (remove-boid id)))
  ([id]
     (dosync (alter *flock* dissoc id))))

(defn scatter-boid [b]
  (make-random-boid (:id b))) 

(defn scatter-boids []
  (dosync
   (ref-set *flock*
	    (into {} (for [[k v] @*flock*] [k (scatter-boid v)])))))


(defn toggle [feature]
  (let [flag (case feature
		   :flock-centering     *fc-force?*
		   :velocity-matching   *vm-force?*
		   :collision-avoidance *ca-force?*
		   :draw-fields         *draw-fields*
		   :clear-once          *clear-once*
		   :wandering           *wa-force?*
		   :pause               *running?*
		   :path                *show-path?*

		   (:attract :repulse)  [*attract?*  feature])]
    (if (vector? flag)
      (let [[flag value] flag]
	(reset! flag value))
      (swap! flag not))))



(let [width *screen-width*
      height *screen-height*
      diagonal (sqrt (+ (sq width) (sq height)))]

  (defn- dist*
    "returns the displacement vector between two points on a toroidal surface"
    [p q w h]
    (let [[px py] p
	  [qx qy] q
	  dx (- qx px) adx (abs dx)
	  dy (- qy py) ady (abs dy)
	  dx (if (< adx (dec (- w adx))) dx (- w adx))
	  dy (if (< ady (dec (- h ady))) dy (- h ady))]
      [dx dy]))

  (defn distance-squared [p q]
    (let [[x y] (dist* p q width height)]
      (+ (sq x) (sq y))))

  (defn distance
    "returns the scalar distance between p and q"
    [p q]
    (let [[x y] (dist* p q width height)]
      (sqrt (+ (sq x) (sq y)))))

  (defn displacement
    "returns the displacement vector from p to q"
    [p q]
    (dist* p q width height))

  (defn- in-range? [pos1 pos2 radius]
    (< (distance-squared pos1 pos2) (sq radius)))
  
  (defn- neighbors*
    "Assumes the world is toroidal"
    [boid flock radius] 
    (filter #(and (not= (:id boid) (:id %))
		  (in-range? (:pos boid) (:pos %) radius))
	    flock))

  (defn neighbors
    "Returns a seq of boids within radius from pos."
    [boid radius]
    (neighbors* boid (vals @*flock*) radius)))


;;;; calculating forces

;;; Collision Avoidance
(defn f-ca [b]
  (let [pos (:pos b)
	epsilon 0.03
	neighbs (neighbors b @*ca-radius*)]
    (if (zero? (count neighbs))
      [0 0]
      (let [weights (map #(/ 1 (+ epsilon (distance pos (:pos %)))) neighbs)
	    sum-wts (reduce + weights)
	    net-frc (reduce (fn [[sum-x sum-y]
				 [weight [dx dy]]]
			      [(+ sum-x (* weight dx)) (+ sum-y (* weight dy))])
			    [0 0]
			    (map (fn [wt nb]
				   [wt (vec-sub (:pos b) (:pos nb))])
				 weights
				 neighbs))]
	[(/ (first net-frc) sum-wts) (/ (second net-frc) sum-wts)]))))

;;; Velocity Matching

;;; TODO
(defn f-vm [b]
  (let [pos (:pos b)
	epsilon 0.03
	neighbs (neighbors b @*vm-radius*)]
    (if (zero? (count neighbs))
      [0 0]
      (let [weights (map #(/ 1 (+ epsilon (distance pos (:pos %)))) neighbs)
	    sum-wts (reduce + weights)
	    net-frc (reduce (fn [[sum-x sum-y]
				 [weight [dx dy]]]
			      [(+ sum-x (* weight dx)) (+ sum-y (* weight dy))])
			    [0 0]
			    (map (fn [wt nb]
				   [wt (vec-sub (:vel b) (:vel nb))])
				 weights
				 neighbs))]
	[(/ (first net-frc) sum-wts) (/ (second net-frc) sum-wts)]))))


;;; Flock Centering
(defn f-fc [b]
  (let [pos (:pos b)
	epsilon 0.03
	neighbs (neighbors b @*fc-radius*)]
    (if (zero? (count neighbs))
      [0 0]
      (let [weights (map #(/ 1 (+ epsilon (distance pos (:pos %)))) neighbs)
	    sum-wts (reduce + weights)
	    net-frc (reduce (fn [[sum-x sum-y]
				 [weight [dx dy]]]
			      [(+ sum-x (* weight dx)) (+ sum-y (* weight dy))])
			    [0 0]
			    (map (fn [wt nb]
				   [wt (vec-sub (:pos nb) (:pos b))])
				 weights
				 neighbs))]
	[(/ (first net-frc) sum-wts) (/ (second net-frc) sum-wts)]))))

;;;; TODO: weighted-sum rewrite
;; (weighted-sum :obj       boid
;; 	      :slot      :pos
;; 	      :neighbors (neighbors b @*ca-radius*)
;; 	      :weight-fn #(/ 1 (+ epsilon (distance (:pos %1) (:pos %2))))
;; 	      :attract?  :attract)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; Wandering
(defn f-wa [b]
  (first (drop-while #(< 1 (+ (sq (first %))
			      (sq (second %))))
		     (repeatedly #(vector (dec (rand 2))
					  (dec (rand 2)))))))

(defn f-usr [b]
  (let [[dx dy] (displacement (:pos b) @*mouse-position*)
	dx (if (< @*max-attract* (abs dx))
		 (or (and (neg? dx) (- @*max-attract*))
		     @*max-attract*)
		 dx)
	dy (if (< @*max-attract* (abs dy))
	     (or (and (neg? dy) (- @*max-attract*))
		 @*max-attract*)
	     dy)
	displ [dx dy]]
    (case @*attract?*
	  :attract displ
	  :repulse (vec (map - displ))
	  	  )))

(defn f-next [b]
  (let [fca   [*ca-force?* @*ca-weight*  (f-ca b)] ;collision-avoidance
	fvm   [*vm-force?* @*vm-weight*  (f-vm b)] ;velocity-matching
	ffc   [*fc-force?* @*fc-weight*  (f-fc b)] ;flock-centering
	fwa   [*wa-force?* @*wa-weight*  (f-wa b)] ;wandering
	usr   [*mouse-dn?* @*usr-weight* (f-usr b)] ;mouse-click 
	sum-f (reduce (fn [[sum-x sum-y]
			   [toggle weight [x y]]]
			(if @toggle
			  [(+ sum-x (* weight x)) (+ sum-y (* weight y))]
			  [(+ sum-x 0) (+ sum-y 0)]))
		      [0 0]
		      [fca fvm ffc fwa usr])
	sum-wts (reduce (fn [sum [toggle weight]]
			  (+ sum (if @toggle weight 0)))
			0
			[[*ca-force?* @*ca-weight*]
			 [*vm-force?* @*vm-weight*]
			 [*fc-force?* @*fc-weight*]
			 [*wa-force?* @*wa-weight*]
			 [*mouse-dn?* @*usr-weight*]])
	net-frc (vector (/ (first  sum-f) (if (zero? sum-wts) 1  sum-wts))
			(/ (second sum-f) (if (zero? sum-wts) 1  sum-wts)))]

    ;; (when (zero? (mod @*time* 10))
    ;;   (println [:fca fca
    ;; 		:fvm fvm
    ;; 		:ffc ffc
    ;; 		:fwa fwa
    ;; 		:sum-f sum-f
    ;; 		:sum-wts sum-wts
    ;; 		:net-frc net-frc]))
    net-frc))

(defn v-next [boid accel dt]
  (let [[ax ay] accel
	[vx vy] (:vel boid)]
    (vector (+ vx (* ax dt)) (+ vy (* ay dt)))))

(defn p-next [boid veloc dt]
  (let [[vx vy] veloc
	[px py] (:pos boid)]
    (vector (mod (+ px (* vx dt)) *screen-width*)
	    (mod (+ py (* vy dt)) *screen-height*))))


(defn update-boid [b]
  (let [dt 1.0
	f-new   (f-next b)
	[vx vy] (v-next b f-new dt)
	v-new   [(constrain vx (- @*max-speed*) @*max-speed*)
		 (constrain vy (- @*max-speed*) @*max-speed*)]
	p-new   (p-next b v-new dt)]
    
    ;; (when (zero? (mod @*time* 10))
    ;;   (println [:Fca  (f-ca b)
    ;; 		:Fvm  (f-vm b)
    ;; 		:Ffc  (f-fc b)
    ;; 		:Fwa  (f-wa b)
    ;; 		:Fnet (f-next b)]))
    (assoc b :pos p-new :vel v-new)))

(defn- update-flock* [flock]
  (into {} (zipmap (keys flock) (map update-boid (vals flock)))))

(defn update-flock []
  (dosync (alter *flock* update-flock*)))

;;;; Updating
(defn update []
  (update-flock)
  (swap! *time* inc)
  ;; (increment-msg-timer)
  )

;;;; Rendering
;; (defn maybe-draw-background []
;;   (if @*clear-once*
;;     (do (reset! *clear-once* false)
;; 	(background-float 200 200 255))
;;     (if (not @*show-path?*)
;;       (background-float 200 200 255))))

(defn draw-background []
  ;; (background-float 200 200 255)
  (background 0)
  (color-mode RGB)
  (fill-float 200 200 255)
  (rect 0 0 *screen-width* *screen-height*))

(defn draw-msg-background []
  (with-translation [0 *screen-height*]
    (rect-mode CORNER)
    (fill 0)
    (rect 0 0 *panel-width* (- *panel-height* *screen-height*))))



;;ringseqs [feed to rads*]

(def erste-ringseq (concat (range 0 60 16) (range 60 120 8) (range 120 160 4)))

;; (take 40 (iterate #(cond (< % 70) (+ % 0.3)
;; 			 (< % 80) (+ % 3.4)
;; 			 (< % 90) (+ % 5.5)
;; 			 :true    (+ % 9.9)) 50))


;;trippy rings
(def *max-age* 120)
(let [rads*  (ref (cycle (concat erste-ringseq (reverse erste-ringseq))))]
   (defn next-rad []
     (dosync
      (let [a (first @rads*)]
	(alter rads* next)
	a))))

(defn draw-ring [x y r1 r2 c1 c2 c3 a]
  (fill-float   c1 c2 c3 a)
   ;; (stroke-float c1 c2 c3 a)
   ;; (stroke-weight (max (/ r2 4) 0))
  (no-stroke)
  (ellipse 0 0 r1 r2))

(let [angles* (ref (cycle (range 0 TWO_PI (/ TWO_PI 360))))]
  (defn next-angle1 []
    (dosync
     (let [a (first @angles*)]
       (alter angles* next)
       a))))

(defn draw-ring-cluster [[ring color1 color2]]
  (when ring
    (let [tmp (next-rad)
	  tmp-angle (next-angle1)
	  [x y] (:pos ring)
	  {h1 :h, s1 :s, b1 :b} color1
	  {h2 :h, s2 :s, b2 :b} color2
	  [r1 r2] [(- tmp 30) (abs (- *max-age* tmp 30))]
	  a (max (- 0.9  (/ tmp 120)) 0)
	  time (mod @*time* 16)]
      (color-mode HSB 1.0)
      (ellipse-mode CENTER)
      (with-translation [x y]
	(with-rotation [(* TWO_PI (/ tmp ;; (age ring)
			  	       *max-age*))
			;; tmp-angle
			]
	  (draw-ring 0 0 r1 r2 h1 s1 b1 a)
	  (draw-ring 0 0 r2 r1 h2 s2 b2 a)
	  (with-rotation [;; tmp-angle
			  (* TWO_PI (/ tmp ;; (age ring)
			  	       *max-age*))
			  ]
	    (with-translation [r1 r1]
	      (draw-ring 0 0 (/ r1 2) (/ r2 2) h1 s1 b1 a)
	      (draw-ring 0 0 (/ r2 2) (/ r1 2) h2 s2 b2 a)))
	  (with-rotation [;; tmp-angle
			  (* TWO_PI (/ tmp ;; (age ring)
			  	       *max-age*))
			  ]
	    (with-translation [(- r1) r1]
	      (draw-ring 0 0 (/ r1 2) (/ r2 2) h1 s1 b1 a)
	      (draw-ring 0 0 (/ r2 2) (/ r1 2) h2 s2 b2 a)))
	  (with-rotation [;; tmp-angle
			  (* TWO_PI (/ tmp ;; (age ring)
			  	       *max-age*))
			  ]
	    (with-translation [r1 (- r1)]
	      (draw-ring 0 0 (/ r1 2) (/ r2 2) h1 s1 b1 a)
	      (draw-ring 0 0 (/ r2 2) (/ r1 2) h2 s2 b2 a)))
	  (with-rotation [;; tmp-angle
			  (* TWO_PI (/ tmp ;; (age ring)
			  	       *max-age*))
			  ]
	    (with-translation [(- r1) (- r1)]
	      (draw-ring 0 0 (/ r1 2) (/ r2 2) h1 s1 b1 a)
	      (draw-ring 0 0 (/ r2 2) (/ r1 2) h2 s2 b2 a))))))))



(defn draw-boid [b]
  (let [[xp yp] (:pos b)
	r (radius b)]
    (fill-float 150 88 220 80)
    (ellipse-mode RADIUS)
    (with-translation [xp yp]
      (with-rotation  [(+ HALF_PI (dir b))]
	(stroke 0)
	(stroke-weight 2)
	(line 0 0 0 (* 2 r))
	(ellipse 0 0 r r)
	(with-translation [0 (* 2 r)]
	  (with-rotation [QUARTER_PI]
	    (line 0 0 0 (/ r 3)))
	  (with-rotation [(- QUARTER_PI)]
	    (line 0 0 0 (/ r 3)))))


      ;; debug force fields
      (when @*draw-fields*
	(no-stroke)
      
	(when @*vm-force?*
	  (stroke 80 70 180)
	  (stroke-weight 2)
	  (no-fill) ;; (fill-float 240 20 20 66)
	  (ellipse 0 0 @*vm-radius* @*vm-radius*))
      
	(when @*ca-force?*
	  (fill-float 120 120 255  66)
	  ;; (no-fill)
	  (ellipse 0 0 @*ca-radius* @*ca-radius*))

	(when @*fc-force?*
	  (stroke-float 0 0 0 96)
	  (no-fill)
	  (ellipse 0 0 @*fc-radius* @*fc-radius*))))))

(defn draw-boids [] (dorun (map draw-boid (vals @*flock*))))

(defn draw-cloids []
  (dorun (map #(draw-ring-cluster [ %
				   {:h 0.75 :s 0.75 :b 0.5}
				   {:h 0.75 :s 0.75 :b 0.5} ])
	      (vals @*flock*))))

(defn draw []
  (if (not @*show-path?*)
    (draw-background)
    (draw-msg-background))
  (draw-boids)
  ;; (draw-cloids)
  (draw-msg 20 (+ *screen-height* 20))
  (when @*running?* (update)))

;;;; User Interaction
(defn mouse-moved [evt]
  (let [x (.getX evt)
	y (.getY evt)]
    (reset! *mouse-position* [x y])))

(defn mouse-pressed [evt]
  (let [x (.getX evt)
	y (.getY evt)]
    ;; (println "type of event: " (type evt))
    ;; (if-let [obj (and (not @*held-obj*) (object-at [x y]))]
    ;;   (dosync (ref-set *held-obj* obj)))
    (reset! *mouse-dn?* true)))

(defn mouse-released [evt]
  (let [x (.getX evt)
	y (.getY evt)]
    (reset! *mouse-dn?* false)))

(defn mouse-dragged [evt]
  (let [x (.getX evt)
	y (.getY evt)]
    ;; (println "~~~__ Drag Event __~~~" "  mouse: " [x y])
    ;; (if-let [obj @*held-obj*]
    ;;   (reset-obj-value [x y]))
    (reset! *mouse-position* [x y]))) ;i wasn't doing this in the Life sime

(defn key-released [evt])

(defn key-pressed
  "Your simulator should act on this group of keyboard commands:

   a,A - Switch to attraction mode (for when mouse is held down).
   r,R - Switch to repulsion mode (for when mouse is held down).
   s,S - Cause all creatures to be instantly scattered to random positions in the window.
   p,P - Toggle whether to have creatures leave a path, that is, whether the window is cleared each display step or not.
   c,C - Clear the window (useful when creatures are leaving paths).
   1 - Toggle the flock centering forces on/off.
   2 - Toggle the velocity matching forces on/off.
   3 - Toggle the collision avoidance forces on/off.
   4 - Toggle the wandering force on/off.
   + - Add one new creature to the simulation. You should allow up to 100 creatures to be created.
   - (minus sign) - Remove one new creature from the simulation (unless there are none already).
   SPACE - Start or stop the simulation (toggle between these)."
  [evt]
  (let [char (.getKeyChar evt)]
    (println "key pressed: " char)
    (case char
  	  (\a \A) (toggle :attract)
	  
  	  (\r \R) (toggle :repulse)

  	  (\p \P) (toggle :path)

	  (\d \D) (toggle :draw-fields)

  	  (\c \C) (draw-background)		  ;; (toggle :clear-once)

  	  (\1)    (toggle :flock-centering)

  	  (\2)    (toggle :velocity-matching)

  	  (\3)    (toggle :collision-avoidance)

  	  (\4)    (toggle :wandering)

  	  \space  (toggle :pause)
	  
  	  (\+)    (add-boid)

  	  (\-)    (remove-boid)

  	  (\s \S) (scatter-boids)

  	  :unrecognized-key-command!)))




(defn setup
  "executes once."
  []
  (println ";;;;;;;;;;;;;;;;____________;;;;;;;;;;;;;;;;")
  (println ";;;;;;;;;;;;;;;;|=Flocking=|;;;;;;;;;;;;;;;;")
  (println ";;;;;;;;;;;;;;;;|__________|;;;;;;;;;;;;;;;;")
  (smooth)
  (init-flock)
  (framerate 17)
  (draw-background)
  (reset! *time* 0)
  (reset! *font* (load-font "Monaco-48.vlw"))
  (display-parameter-window))


(defapplet flocking
  :title "Caspary's Flocking Sim!"
  :setup setup
  :draw draw
  :size [*panel-width* *panel-height*]
  :mouse-moved mouse-moved
  :mouse-pressed mouse-pressed
  :mouse-released mouse-released
  :mouse-dragged mouse-dragged
  :key-pressed key-pressed
  :key-released key-released)


(defn -main [& args]
  (run flocking))

;; (stop flocking)
 (run flocking :interactive)