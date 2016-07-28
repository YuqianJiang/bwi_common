#program base.
location(l1).
location(l2).
location(l3).
location(l4).
location(l5).
location(l6).
location(l7).
location(l8).
location(l9).

door(d1).
door(d2).

hasdoor(l2,d1).
hasdoor(l3,d1).
hasdoor(l6,d2).
hasdoor(l7,d2).

acc(l1,l2).
acc(l2,l5).
acc(l3,l8).
acc(l3,l9).
acc(l4,l5).
acc(l5,l6).
acc(l7,l9).

dooracc(L1,D,L2) :- hasdoor(L1,D), hasdoor(L2,D), L1 != L2, door(D), location(L1), location(L2).
dooracc(L1,D,L2) :- dooracc(L2,D,L1).

acc(L1,L1) :- location(L1).
acc(L1,L2) :- acc(L2,L1).

#program cumulative(n).
:- goto(L1,L2,I), at(L2,I-2), I>1, I=n-1.
%1{goto(l1,l2,I); goto(l1,l3,I); goto(l1,l1,I)}1 :- not noop(I), at(l1,I-1), I>0, I=n-1.
%1{goto(l2,l1,I); goto(l2,l4,I); goto(l2,l2,I)}1 :- not noop(I), at(l2,I-1), I>0, I=n-1.
%1{goto(l7,l5,I); goto(l7,l8,I); goto(l7,l7,I)}1 :- not noop(I), at(l7,I-1), I>0, I=n-1.
%1{goto(l8,l6,I); goto(l8,l7,I); goto(l8,l8,I)}1 :- not noop(I), at(l8,I-1), I>0, I=n-1.
