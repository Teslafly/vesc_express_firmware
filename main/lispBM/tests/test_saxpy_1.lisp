

(define x (vector 1.0 2.0 3.0))
(define y (vector 0.1 0.2 0.3))
(define alpha 2)

(define r (saxpy alpha x y))

(eq (vector-to-list r) (list 2.1 4.2 6.3))
