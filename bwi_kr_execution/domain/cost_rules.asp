#program step(n).

state(is_in(1,R), n-1) :- is_in(1,R,n-1).
state(is_near(1,L), n-1) :- is_near(1,L,n-1).
action(navigate_to(R), n) :- navigate_to(R,n).

eval_cost(C, n) :- c((), A, C), action(A, n), #count{S:state(S, n-1)}=0.
eval_cost(C, n) :- c((S1), A, C), state(S1, n-1), action(A, n), #count{S:state(S, n-1)}=1.
eval_cost(C, n) :- c((S1, S2), A, C), state(S1, n-1), state(S2, n-1), action(A, n), #count{S:state(S, n-1)}=2.
eval_cost(C, n) :- c((S1, S2, S3), A, C), state(S1, n-1), state(S2, n-1), state(S3, n-1), action(A, n), #count{S:state(S, n-1)}=3.
eval_cost(C, n) :- c((S1, S2, S3, S4), A, C), state(S1, n-1), state(S2, n-1), state(S3, n-1), state(S4, n-1), action(A, n), #count{S:state(S, n-1)}=4.

cost(C, n) :- eval_cost(C, n).
cost(10, n) :- #count{C:eval_cost(C, n)}=0.

:~ cost(X,Y). [X@1,Y]

#show cost/2.

#show is_in/3.
#show is_near/3.