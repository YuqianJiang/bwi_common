#include <iclingo>.
#program cumulative(n).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Actions
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
facing(D,I) :- approach(D,I),I>0,I=n-1.
%beside(D,I) :- approach(D,I),I>0,I=n-1.
:- approach(D,I), facing(D,I-1),I>0,I=n-1.
:- approach(D,I), at(L,I-1), not hasdoor(L,D),I>0,I=n-1.

at(L2,I) :- gothrough(D,I), dooracc(L1,D,L2), at(L1,I-1), I>0,I=n-1.
-facing(D,I) :- gothrough(D,I),I>0,I=n-1.
:- gothrough(D,I), not facing(D,I-1),I>0,I=n-1.
:- gothrough(D,I), not open(D,I-1),I>0,I=n-1.
:- gothrough(D,I), at(L,I-1), not hasdoor(L,D),I>0,I=n-1.

open(D,I) :- opendoor(D,I),I>0,I=n-1.
:- opendoor(D,I), not facing(D,I-1),I>0,I=n-1.
:- opendoor(D,I), open(D,I-1),I>0,I=n-1.

at(L2,I) :- goto(L1,L2,I),I>0,I=n-1.
:- goto(L1,L2,I), not at(L1,I-1),I>0,I=n-1.
:- goto(L1,L2,I), not acc(L1,L2),I>0,I=n-1.
:- goto(L1,L2,1), exgoto(R,L2,L1,1).

open(D,I) :- waitforopen(D,I),I>0,I=n-1.
facing(D,I) :- waitforopen(D,I),I>0,I=n-1.
%:- waitforopen(D,I), not facing(D,I-1),I>0,I=n-1.
:- waitforopen(D,I), facing(D,I-1),I>0,I=n-1.
:- waitforopen(D,I), open(D,I-1),I>0,I=n-1.
:- waitforopen(D,I), at(L,I-1), not hasdoor(L,D),I>0,I=n-1.
:- waitforopen(D,I), 0{exopendoor(R,D,J)}0,I>0,I=n-1.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Static laws
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%you can't be at two places at the some time
-at(L2,I):- at(L1,I), location(L2), L1 != L2,I>0,I=n-1.
%you can be facing only one door at a time
-facing(D2,I):- facing(D1,I), door(D1), door(D2), D1 != D2,I>0,I=n-1.
%you can only be beside a door at any given time (if you delete this,
%the observations must also return -beside which doesn't happen at the moment.
%-beside(D2,I):- beside(D1,I), door(D1), door(D2), D1 != D2,I>0,I=n-1.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Inertia
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%at is inertial
at(L,I) :- at(L,I-1), not -at(L,I),I>0,I=n-1.
%facing is inertial
facing(D,I) :- facing(D,I-1), not -facing(D,I),I>0,I=n-1.
-facing(D,I) :- -facing(D,I-1), not facing(D,I),I>0,I=n-1.
% open is inertial
%open(D,I) :- open(D,I-1), not -open(D,I),I>0,I=n-1.
-open(D,I) :- -open(D,I-1), not open(D,I),I>0,I=n-1.
% beside is inertial
%beside(D,I) :- beside(D,I-1), not -beside(D,I),I>0,I=n-1.
%-beside(D,I) :- -beside(D,I-1), not beside(D,I),I>0,I=n-1.

#show at/2.
#show open/2.
#show -open/2.
#show facing/2.
%#show -facing/2.
%#show beside/2.

#show approach/2.
#show gothrough/2.
#show opendoor/2.
#show waitforopen/2.
#show goto/3.
#show wait/2.
