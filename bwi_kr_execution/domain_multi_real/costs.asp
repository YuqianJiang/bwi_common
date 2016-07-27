#program base.
dist(l1,l2,1).
dist(l2,l1,1).
dist(l2,l5,1).
dist(l5,l2,1).
dist(l3,l8,1).
dist(l8,l3,1).
dist(l3,l9,3).
dist(l9,l3,3).
dist(l4,l5,2).
dist(l5,l4,2).
dist(l5,l6,1).
dist(l6,l5,1).
dist(l7,l9,2).
dist(l9,l7,2).

cumucost(0,0).
cumuparam(0,0).

nowgotocost(L1,L2,X) :- nowgotocost(L1,L2,X).
nowopendoorcost(D,X) :- nowopendoorcost(D,X).
nowgoto(L1,L2) :- nowgotocost(L1,L2,X).
nowopendoor(D) :- nowopendoorcost(D,X).

#program cumulative(n).
cost(20,1) :- opendoor(D,1), not nowopendoor(D), n=2.
cost(X,1) :- opendoor(D,1), nowopendoorcost(D,X), n=2.
cost(X*4,1) :- goto(L1,L2,1), nowgotocost(L1,L2,X), n=2.
cost(X*4,1) :- goto(L1,L2,1), dist(L1,L2,X), not nowgoto(L1,L2), n=2.
%cost(X*4,1) :- wait(X,1), n=2.
:- goto(L,L,I), not nowgoto(L,L), I>0,I=n-1.

cost(X*4,I) :- goto(L1,L2,I), dist(L1,L2,X),I>1,I=n-1.
cost(1,I) :- approach(D,I),I>0,I=n-1.
cost(2,I) :- gothrough(D,I),I>0,I=n-1. 
cost(20,I) :- opendoor(D,I),door(D),I>1,I=n-1.

cost(0,I) :- noop(I),I>0,I=n-1.

param(X*2,1) :- goto(L1,L2,1), nowgotocost(L1,L2,X), n=2.
param(X*2,1) :- goto(L1,L2,1), dist(L1,L2,X), not nowgoto(L1,L2), n=2.
param(X*2,I) :- goto(L1,L2,I), dist(L1,L2,X),I>1,I=n-1.
param(0,I) :- gothrough(D,I),I>0,I=n-1.
param(0,I) :- approach(D,I),I>0,I=n-1.
param(0,I) :- opendoor(D,I),I>0,I=n-1.

shared(I) :- shared(I-1),I>0,I=n-1.

cumucost(X,n-1) :- X = #sum { Y,I: cost(Y,I) }, n>1.
cumuparam(X,n-1) :- not shared(n-1), X = #sum { Y,I: param(Y,I) }, n>1.
cumuparam(X,n-1) :- shared(n-1), X = #sum { Y,I: param(Y,I) , shared(I) }, n>1.

%:~ cost(X,Y). [X@1,Y]
%:~ collisioncost(X,Y). [X@1,Y]
%:~ param(X,Y). [X/2@1,Y]

#show cost/2.
%#show param/2.
#show cumucost/2.
#show cumuparam/2.
