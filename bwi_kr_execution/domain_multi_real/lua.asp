#program cumulative(n).
collisioncost(@overlappenalty(T1,P1,T2,P2,X,mu),J) :- goto(L2,L1,J), exgoto(R,L1,L2,I), excumucost(R,T1,I-1), excumuparam(R,P1,I-1), cumucost(T2,J-1), cumuparam(P2,J-1), dist(L1,L2,X), I=1..10, J>0, J=n-1.

cost(@waitingtime(T1,P1,T2,P2,rho),J) :- waitforopen(D,J), exopendoor(R,D,I), excumucost(R,T1,I), excumuparam(R,P1,I), cumucost(T2,J-1), cumuparam(P2,J-1), I=1..10, J>0, J=n-1.

:- waitforopen(D,J), exopendoor(R,D,I), excumucost(R,T1,I), excumuparam(R,P1,I), cumucost(T2,J-1), cumuparam(P2,J-1), I=1..10, J>0, J=n-1, @waitingtime(T1,P1,T2,P2,rho)=0.

param(P1,J) :- waitforopen(D,J), exopendoor(R,D,I), excumuparam(R,P1,I), J>0, J=n-1.

shared(J) :- waitforopen(D,J), J>0, J=n-1.

#script (lua)
facts = {1,1,2,6,24,120,720,5040,40320,362880,3628800}
function overlappenalty(c1,p1,c2,p2,d,mu)
    lambda1_s = p1 * 0.1
    lambda2_s = p2 * 0.1
    if (lambda1_s > 2) then
      return 0
    end
    if (lambda2_s > 2) then
      return 0
    end
    lambda1_c = lambda1_s + d/5
    lambda2_c = lambda2_s + d/5
    c1_c = c1+4*d
    c2_c = c2+4*d
    if ((c2-c1_c+(lambda2_s-lambda1_c)*5)>10) then
      return 0    
    end
    if ((c1-c2_c+(lambda1_s-lambda2_c)*5)>10) then
      return 0
    end
    f1_c = math.exp(-lambda1_c)
    f2_c = math.exp(-lambda2_c)
    f1_s = math.exp(-lambda1_s)
    f2_s = math.exp(-lambda2_s)
    s = 1
    for i = 0,10 do
        a = math.max(math.ceil((c1_c-c2)/5) + i, 0)
        b = math.max(math.ceil((c2_c-c1)/5) + i, 0)
        prob1_c = math.pow(lambda1_c,i)*f1_c/facts[i+1]
        prob2_c = math.pow(lambda2_c,i)*f2_c/facts[i+1]
        for j = a,10 do
            prob2_s = math.pow(lambda2_s,j)*f2_s/facts[j+1]
            s = s - prob1_c*prob2_s
        end
        for j = b,10 do
            prob1_s = math.pow(lambda1_s,j)*f1_s/facts[j+1]
            s = s - prob2_c*prob1_s
        end
        if (s < 0) then return 0 end
    end
    return math.floor(s*mu)
end

function waitingtime(c1,p1,c2,p2,rho)
    lambda1_c = p1*0.1
    lambda2_s = p2*0.1
    if (lambda1_c > 2) then
      return 0
    end
    if (lambda2_s > 2) then
      return 0
    end
    ex = (c1-c2+(lambda1_c-lambda2_s)*5)
    if (ex>17) then
      return 0
    end
    if (ex<=0) then
      return 0
    end
    if ((lambda1_c == 0) and (lambda2_s == 0)) then
      return ex+3
    end
    s1 = 0
    s2 = 0
    f2_s = math.exp(-lambda2_s)
    f1_c = math.exp(-lambda1_c)
    c = math.ceil((c2-c1)/5)
    for i = 0,10 do
        prob2_s = math.pow(lambda2_s,i)*f2_s/facts[i+1]
        a = math.min(math.max(c+i, -1),10)
        for j = 0,a do
            prob1_c = math.pow(lambda1_c,j)*f1_c/facts[j+1]
            s1 = s1 + prob2_s*prob1_c
        end
        for j = a+1,10 do
            prob1_c = math.pow(lambda1_c,j)*f1_c/facts[j+1]
            s2 = s2 + (c1-c2+(j-i)*5)*prob2_s*prob1_c
        end
    end
    s1 = (1-s1)*rho/10+s1
    if s1>0.33 then
      return 0
    end
    if (s2>17) then
      return 0
    end
    return math.floor(s2+3)
end
#end.

#program base.
exgoto(R,L1,L2,I) :- exgoto(R,L1,L2,I).
exopendoor(R,D,I) :- exopendoor(R,D,I).
excumucost(R,T,I) :- excumucost(R,T,I).
excumuparam(R,P,I) :- excumuparam(R,P,I).

#show collisioncost/2.