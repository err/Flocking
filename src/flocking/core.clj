(ns flocking.core
  (:use [flocking.boid]
	[flocking.utils]
	[flocking.events]
	[flocking.protocols]
	[rosado.processing]
	[rosado.processing.applet]))


(def *screen-width*    1000)
(def *screen-height*   1000)
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
(def *show-path?*  (atom true))
(def *clear-once*  (atom false))
(def *running?*    (atom false))
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

(def *min-speed*  0.15)
(def *max-speed*     6)
(def *max-accel*   1.1)

;;;; RADII
;;;
(def *fc-radius*   280) ;flock-centering radius
(def *ca-radius*   100) ;collision-avoidance radius
(def *vm-radius*   260) ;velocity-matching radius

;;;; WEIGHTS
;;; TODO: add knobs/sliders for
;;; adjusting these parameters
(def *ca-weight*   30)
(def *vm-weight*   50)
(def *fc-weight*   50)
(def *wa-weight*   80)
(def *usr-weight*   4)

(defn make-random-boid [id]
  (let [pos  [(* *screen-width*  (rand 1))
	      (* *screen-height* (rand 1))]
	vel  [(+ (* (dec *max-speed*) (dec (rand 2))) *min-speed*)
	      (+ (* (dec *max-speed*) (dec (rand 2))) *min-speed*)]]
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
       (dosync (alter *flock* assoc id (make-random-boid id)))
       (swap! *boid-count* inc))))

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
	  dx (- qx px)
	  dy (- qy py)
	  dx (if (< (abs dx) (- w dx)) dx (- w dx))
	  dy (if (< (abs dy) (- w dy)) dy (- w dy))]
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
  
  (defn- neighbors* [pos flock radius wrapped?]
    (if wrapped?
      (filter #(and (not= pos (:pos %))
		    (in-range? pos (:pos %) radius))
	      flock)))

  (defn neighbors
    "Returns a seq of boids within radius from pos. Note that the world is toroidal, hence the :wrapped flag"
    [boid radius]
    (neighbors* (:pos boid) (vals @*flock*) radius :wrapped)))


;;;; calculating forces

;;; Collision Avoidance
(defn f-ca [b]
  (let [pos (:pos b)
	epsilon 0.03
	neighbs (neighbors b *ca-radius*)]
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
(defn f-vm [b]
  (let [pos (:pos b)
	epsilon 0.03
	neighbs (neighbors b *vm-radius*)]
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
	neighbs (neighbors b *fc-radius*)]
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
;; 	      :neighbors (neighbors b *ca-radius*)
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
  (let [displ (displacement (:pos b) @*mouse-position*)]
    (case @*attract?*
	  :attract displ
	  :repulse (vec (map - displ))
	  	  )))

(defn f-next [b]
  (let [fca   [*ca-force?* *ca-weight*  (f-ca b)] ;collision-avoidance
	fvm   [*vm-force?* *vm-weight*  (f-vm b)] ;velocity-matching
	ffc   [*fc-force?* *fc-weight*  (f-fc b)] ;flock-centering
	fwa   [*wa-force?* *wa-weight*  (f-wa b)] ;wandering
	usr   [*mouse-dn?* *usr-weight* (f-usr b)] ;mouse-click 
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
			[[*ca-force?* *ca-weight*]
			 [*vm-force?* *vm-weight*]
			 [*fc-force?* *fc-weight*]
			 [*wa-force?* *wa-weight*]
			 [*mouse-dn?* *usr-weight*]])
	net-frc (vector (/ (first  sum-f)   sum-wts
			   )
			(/ (second sum-f)   sum-wts
			   ))]

    (when (zero? (mod @*time* 10))
      (println [:fca fca
    		:fvm fvm
    		:ffc ffc
    		:fwa fwa
    		:sum-f sum-f
    		:sum-wts sum-wts
    		:net-frc net-frc]))
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
	v-new   [(constrain vx (- *max-speed*) *max-speed*)
		 (constrain vy (- *max-speed*) *max-speed*)]
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
  (swap! *time* inc))

;;;; Rendering
;; (defn maybe-draw-background []
;;   (if @*clear-once*
;;     (do (reset! *clear-once* false)
;; 	(background-float 200 200 255))
;;     (if (not @*show-path?*)
;;       (background-float 200 200 255))))

(defn draw-background []
  (background-float 200 200 255))

(defn draw-boid [b]
  (let [[xp yp] (:pos b)
	r (radius b)]
    (fill-float 150 88 220)
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
	  (stroke 4)
	  (stroke-weight 1)
	  (no-fill) ;; (fill-float 240 20 20 66)
	  (ellipse 0 0 *vm-radius* *vm-radius*))
      
	(when @*ca-force?*
	  (fill-float 0 170 130 66)
	  
	  (ellipse 0 0 *ca-radius* *ca-radius*))

	(when @*fc-force?*
	  (no-stroke)
	  (fill-float 255 120 120 66)
	  (ellipse 0 0 *fc-radius* *fc-radius*))))))

(defn draw-boids []
  (dorun (map draw-boid (vals @*flock*))))

(defn draw []
  (when (not @*show-path?*)
    (draw-background))
  (draw-boids)
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
    (reset! *mouse-dn?* true)))

(defn mouse-released [evt]
  (let [x (.getX evt)
	y (.getY evt)]
    (reset! *mouse-dn?* false)))

(defn mouse-dragged [evt]
  (let [x (.getX evt)
	y (.getY evt)]
    ;; (println "~~~__ Drag Event __~~~" "  mouse: " [x y])
    (reset! *mouse-position* [x y])   ;i wasn't doing this in the Life sime
    ))

(defn key-released [evt]
  ;; (let [char (.getKeyChar evt)]
  ;;   (println "key pressed: " char)
  ;;   (case char
  ;; 	  (\a \A) (toggle :attract)
	  
  ;; 	  (\r \R) (toggle :repulse)

  ;; 	  (\p \P) (toggle :path)

  ;; 	  (\c \C) (toggle :clear-once)

  ;; 	  (\1)    (toggle :flock-centering)

  ;; 	  (\2)    (toggle :velocity-matching)

  ;; 	  (\3)    (toggle :collision-avoidance)

  ;; 	  (\4)    (toggle :wandering)

  ;; 	  \space  (toggle :pause)
	  
  ;; 	  (\+)    (add-boid)

  ;; 	  (\-)    (remove-boid)

  ;; 	  (\s \S) (scatter-boids)

  ;; 	  :unrecognized-key-command!))
  )

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
  (draw-background)
  (framerate 30)
  (init-flock)
  (reset! *time* 0)
  (reset! *boid-count* 0)
  ;; (reset! *debug-font* (load-font "Monaco-10.vlw"))
  )


(defapplet flock
  :title "Caspary's Flocking Sim!"
  :setup setup
  :draw draw
  :size [*screen-width*  *screen-height*]
  :mouse-moved mouse-moved
  :mouse-pressed mouse-pressed
  :mouse-released mouse-released
  :mouse-dragged mouse-dragged
  :key-pressed key-pressed
  :key-released key-released)

;; (defn -main [& args]
;;   (run flock))

;; (run flock :interactive)
;; (stop flock)