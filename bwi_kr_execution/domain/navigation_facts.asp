#program base.
room(l3_414b). 
room(l3_414a). 
room(l3_402). 
room(l3_520). 
room(l3_400). 
room(l3_508). 
room(l3_428). 
room(l3_404). 
room(l3_424).
room(l3_502). 
room(l3_426). 
room(l3_500). 
room(l3_420). 
room(l3_506). 
room(l3_422). 
room(l3_504). 
room(l3_200). 
room(l3_300). 
room(l3_303).
room(l3_302). 
room(l3_406). 
room(l3_250). 
room(l3_410). 
room(l3_412). 
room(l3_518). 
room(l3_414). 
room(l3_416). 
room(l3_514). 
room(l3_418).
room(l3_516). 
room(l3_430). 
room(l3_510). 
room(l3_436). 
room(l3_512). 
room(l3_434). 
room(l3_432). 
room(l3_408). 
room(l2_302). 


door(d3_404). 
door(d3_400). 
door(d3_508). 
door(d3_402). 
door(d3_500). 
door(d3_502). 
door(d3_430). 
door(d3_422). 
door(d3_420).
door(d3_414a2). 
door(d3_414a3). 
door(d3_414a1). 
door(d3_416). 
door(d3_516). 
door(d3_418). 
door(d3_512). 
door(d3_510). 
door(d3_414b3).
door(d3_414b2). 
door(d3_414b1). 
door(d3_432). 
door(d3_436).


hasdoor(l3_404,d3_404). 
hasdoor(l3_400,d3_404). 

hasdoor(l3_400,d3_400). 
hasdoor(l3_300,d3_400). 

hasdoor(l3_500,d3_508). 
hasdoor(l3_508,d3_508). 

hasdoor(l3_402,d3_402). 
hasdoor(l3_400,d3_402). 

hasdoor(l3_500,d3_500). 
hasdoor(l3_200,d3_500). 

hasdoor(l3_500,d3_502). 
hasdoor(l3_502,d3_502). 

hasdoor(l3_430,d3_430). 
hasdoor(l3_400,d3_430). 

hasdoor(l3_422,d3_422). 
hasdoor(l3_400,d3_422). 

hasdoor(l3_420,d3_420). 
hasdoor(l3_400,d3_420). 

%please be coherent with the simulation domain with a1 and a2
hasdoor(l3_500,d3_414a2).  
hasdoor(l3_414a,d3_414a2). 

hasdoor(l3_414,d3_414a3). 
hasdoor(l3_414a,d3_414a3). 

%please be coherent with the simulation domain with a1 and a2
hasdoor(l3_414a,d3_414a1). 
hasdoor(l3_400,d3_414a1). 

hasdoor(l3_416,d3_416). 
hasdoor(l3_400,d3_416). 

hasdoor(l3_500,d3_516). 
hasdoor(l3_516,d3_516). 

hasdoor(l3_418,d3_418). 
hasdoor(l3_400,d3_418). 

hasdoor(l3_500,d3_512). 
hasdoor(l3_512,d3_512). 

hasdoor(l3_500,d3_510). 
hasdoor(l3_510,d3_510). 

hasdoor(l3_414b,d3_414b3). 
hasdoor(l3_414,d3_414b3). 

hasdoor(l3_400,d3_414b2). 
hasdoor(l3_414b,d3_414b2). 

hasdoor(l3_414b,d3_414b1). 
hasdoor(l3_500,d3_414b1). 

hasdoor(l3_432,d3_432). 
hasdoor(l3_400,d3_432). 

hasdoor(l3_436,d3_436). 
hasdoor(l3_400,d3_436). 


acc(l3_434, l3_400). 
acc(l3_434, l3_500). 
acc(l3_518, l3_500). 
acc(l3_514, l3_500). 
acc(l3_504, l3_500). 
acc(l3_520, l3_500). 
acc(l3_410, l3_500). 
acc(l3_424, l3_400). 
acc(l3_408, l3_400). 
acc(l3_410, l3_400). 
acc(l3_200, l3_303). 
acc(l3_302, l3_303). 
acc(l3_302, l3_300). 
acc(l3_250, l3_300). 
acc(l3_250, l3_303). 


dooracc(R1,D,R2) :- hasdoor(R1,D), hasdoor(R2,D), R1 != R2, door(D), room(R1), room(R2).
dooracc(R1,D,R2) :- dooracc(R2,D,R1).

acc(R1,R1) :- room(R1).                                                         
acc(R1,R2) :- acc(R2,R1), room(R1), room(R2).                                   
acc(R1,R2) :- acc(R1,R3), acc(R2,R3), room(R1), room(R2), room(R3).             


object(coffee_counter).                                                         
inside(coffee_counter, l2_302).

