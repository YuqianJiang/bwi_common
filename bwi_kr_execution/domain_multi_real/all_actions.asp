#program cumulative(n).

1{
approach(D1,I) : door(D1);
gothrough(D2,I) : door(D2);
opendoor(D3,I) : door(D3);
goto(L1,L2,I) : location(L1), location(L2);
waitforopen(D4,I) : door(D4)
%wait(X,I) : X=1..6, I=1
}1 :- not noop(I), I>0, I=n-1.

noop(I) :- noop(I), I>0, I=n-1.
