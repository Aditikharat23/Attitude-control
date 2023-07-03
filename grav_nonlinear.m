function dstatedt = grav_nonlinear(t,state,K,I,B)
%State = [psi,theta,phi,psid,thetad,phid]
%Input is a row, output is a column
r = 7000; %Radius of orbit
n = sqrt(398600/r^3);

u = -K*state;
temp = B*u;
psi = state(1);
theta=state(2);
phi=state(3);
psid=state(4);
thetad=state(5);
phid=state(6);


psidd = (((I(1,1)-I(2,2))*(phid+(n*psi))*(thetad+n)) + (n*phid))/I(3,3);
thetadd = -(((I(1,1)-I(3,3))*(psid-(n*phi))*(phid+(n*psi)))+(3*theta*(I(1,1)-I(3,3))*n^2))/I(2,2);
phidd =(((I(2,2)-I(3,3))*(psid-(n*phi))*(thetad+n))+(3*phi*(I(3,3)-I(2,2))*n^2) - (n*psid))/I(2,2);

dstatedt = [psid;thetad;phid;psidd;thetadd;phidd] + temp;
end
