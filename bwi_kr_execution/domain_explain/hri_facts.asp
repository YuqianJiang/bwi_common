#program base.
person(peter). 
person(ray). 
person(dana). 
person(justin). 
person(garrett).
person(jivko). 
person(stacy).
%person(yuqian).
%person(rolando).

hasoffice(peter,l3_508). 
hasoffice(ray,l3_512).
hasoffice(dana,l3_510). 
hasoffice(justin,l3_402). 
hasoffice(garrett,l3_422).
hasoffice(jivko,l3_432). 
hasoffice(stacy,l3_502).

group(bwi).

ingroup(peter,bwi).
ingroup(jivko,bwi).
ingroup(garrett,bwi).
ingroup(justin,bwi).
%ingroup(yuqian,bwi).
%ingroup(rolando,bwi).

possiblelocation(P,R) :- hasoffice(P,R), person(P), room(R).
%possiblelocation(P,l3_414b) :- ingroup(P,bwi).

possiblelocation(P) :- possiblelocation(P,R).
:- possiblelocation(P,R,0), not room(R).

object(O) :- locationmarker(P,O,0).
inside(O,R) :- locationmarker(P,O,0), inroom(P,R,0).
person(P) :- possiblelocation(P,R,0).
possiblelocation(P,R) :- possiblelocation(P,R,0).

meeting(M,G,R) :- meeting(M,G,R). %here for when not using meetings


