syms z m g r x T u Kv dx U

% dx^2 = -Kv*dx 1/(m*r*r)  -


s1 = -dx + (U-x*m*g*r - Kv*dx)/(m*r*r)*T*(z+1)/2/(z-1);
% s3 = -u -x - Kv*dx;
dx = solve(s1,dx);
simplify(dx)

r = collect(dx*T*(z+1)/2/(z-1),z)-x;
x = collect(solve(r,x),z);

m=0.3;
Kv=0.03;
r=0.2;
T=0.05;

% g*K +K1*g+K2*g=g
% K +K1+K2=1


% syms So Cp D qp yp Kmp dx
% осевая сила резанья So - подача 
% Po = 10 *Cp * (D^qp) * ((So)^yp) *Kmp;
% Po = 10 *Cp * (D^qp) * ((vo*to)^yp) *Kmp;
% Po = 10 *Cp * (D^qp) * ((vo*2*pi/w)^yp) *Kmp;
% Mk = 10*Cm*D^qm*So^ym*Km
% Mk = 10*Cm*D^qm*(vo*to)^ym*Kmp
% Mk = 10*Cm*D^qm*(vo*2*pi/w)^ym*Kmp

% So = (Po/(10 *Cp * (D^qp) * Kmp))^(-yp)
% Mk = 10*Cm*D^qm*(So)^ym*Kmp
% So = vo*2*pi/w
% vo = So*w/(2*pi)
Cp = 68;
yp=0.7;
qp=1;
D=36;
So=0.04;
Kmp = 1;
Po = 10 *Cp * (D^qp) * ((So)^yp) *Kmp;
Cm = 0.03;
ym = 0.8;
qm = 2;
Mk = 10*Cm*(D^qm)*(So^ym)*Kmp;
P=Mk*30;

zz = pi/0.07
Po=0.4*100*zz
So= (Po/(10 *Cp * (D^qp) * Kmp))^(1/yp)
Mk = 10*Cm*D^qm*(So)^ym*Kmp
nn=10000
W = Mk*nn*pi/30
v=So*nn/60
tt=36/v
% So=

syms ITmin ITmax T10min T10max S K B
s1 = -T10min*S + (ITmin-B) *K;
s2 = -T10max*S + (ITmax-B) *K;
r = solve(s1,s2, K,B)
