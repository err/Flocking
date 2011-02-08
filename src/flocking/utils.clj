(ns flocking.utils)

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

(defn cpu-count
  "Get the number of CPUs on this machine."
  []
  (.availableProcessors (Runtime/getRuntime)))

(defn arg-count
  "Get the arity of a function."
  [f]
  (let [m (first (filter #(= "invoke" (.getName %))
			 (.getDeclaredMethods (class f))))
        p (.getParameterTypes m)]
    (alength p)))

(defn run-handler [handler & args]
  (try
    (apply handler (take (arg-count handler) args))
    (catch Exception e
      (.printStackTrace e))))