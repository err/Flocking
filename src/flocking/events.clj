(ns flocking.events
  (:import (java.util.concurrent Executors LinkedBlockingQueue))
  (:use [flocking.utils]))

(def *NUM-THREADS*   (cpu-count))
(def *thread-pool*   (Executors/newFixedThreadPool *NUM-THREADS*))
(def *handlers*      (ref {}))
(def *sync-handlers* (ref {}))


(defn- on-event*
  [handler-ref* event-type key handler]
  (dosync
    (let [handlers (get @handler-ref* event-type {})]
      (alter handler-ref* assoc event-type (assoc handlers key handler))
      true)))

(defn on-event
  "Runs handler whenever events of type event-type are fired.  The handler can
  optionally except a single event argument, which is a map containing the
  :event-type property and any other properties specified when it was fired.

  (on-event ::booted #(do-stuff))
  (on-event ::midi-note-down (fn [event] (funky-bass (:note event))))

  Handlers can return :done to be removed from the handler list after execution."
  [event-type key handler]
  (on-event* *handlers* event-type key handler))


(defn on-sync-event
  "Synchronously runs handler whenever events of type event-type are fired.  The handler can
  optionally except a single event argument, which is a map containing the
  :event-type property and any other properties specified when it was fired."
  [event-type key handler]
  (on-event* *sync-handlers* event-type key handler))

(defn remove-handler
  "Remove an event handler previously registered to handle events of event-type.

  (defn my-foo-handler [event] (do-stuff (:val event)))

  (on-event ::foo my-foo-handler)
  (event ::foo :val 200) ; my-foo-handler gets called with {:event-type ::foo :val 200}
  (remove-handler ::foo my-foo-handler)
  (event ::foo :val 200) ; my-foo-handler no longer called
  "
  [event-type key]
  (dosync
    (doseq [handler-ref* [*handlers* *sync-handlers*]]
      (let [handlers (get @handler-ref* event-type {})]
        (alter handler-ref* assoc event-type (dissoc handlers key))))))

(defn clear-handlers
  "Remove all handlers for events of type event-type."
  [event-type]
  (dosync
    (alter *handlers* dissoc event-type)
    (alter *sync-handlers* dissoc event-type))
  nil)

(defn- handle-event
  "Runs the event handlers for the given event, and removes any handler that returns :done."
  [handlers* event]
  (let [event-type (:event-type event)
        handlers (get @handlers* event-type {})
        drop-keys (doall (map first
                              (filter (fn [[k handler]]
                                        (= :done (run-handler handler event)))
                                      handlers)))]
    (dosync
      (alter handlers* assoc event-type
             (dissoc (get @handlers* event-type) drop-keys)))))

(defn event
  "Fire an event of type event-type with any number of additional properties.
  NOTE: an event requires key/value pairs, and everything gets wrapped into an
  event map.  It will not work if you just pass values.

  (event ::my-event)
  (event ::filter-sweep-done :instrument :phat-bass)"
  [event-type & args]
  {:pre [(even? (count args))]}
  (let [event (apply hash-map :event-type event-type args)]
    (handle-event *sync-handlers* event)
    (.execute *thread-pool* #(handle-event *handlers* event))))