#program base.
person(peter). 
person(ray). 
person(dana). 
person(justin). 
person(garrett).
person(jivko). 
person(stacy).
person(yuqian).
person(rolando).

hasoffice(peter,l3_508). 
hasoffice(ray,l3_512).
hasoffice(dana,l3_510). 
hasoffice(justin,l3_402). 
hasoffice(garrett,l3_422).
hasoffice(jivko,l3_432). 
hasoffice(stacy,l3_502).

%students in the lab
hasoffice(yuqian,l3_414b).
hasoffice(rolando,l3_414b).

group(bwi).

ingroup(peter,bwi).
ingroup(jivko,bwi).
ingroup(garrett,bwi).
ingroup(justin,bwi).
ingroup(yuqian,bwi).
ingroup(rolando,bwi).

canbeinroom(P,R) :- hasoffice(P,R), person(P), room(R).
%canbeinroom(P,l3_414b) :- ingroup(P,bwi).

canknow(P1,P2) :- ingroup(P1,G), ingroup(P2,G), P1 != P2, group(G).

meeting(M,G,R) :- meeting(M,G,R). %here for when not using meetings


