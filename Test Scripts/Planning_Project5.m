global x y x0 y0 m a b c n s dist valid;

% Parameters only initialized for program, In real life, x0,y0,m are found
% by sensors

x0=7;  %------> Initial Position X0
y0=3;   %------> Inital Position Y0
m=0;    %------> Initial Slope
n=0;    %------> Initial Curvature
dist=0.15; %Resolution for straight movement
s=0.005; %Measurement noise variance w.r.t position, slope etc. 



hold off
plot(x0,y0,'o');
hold on;

CheckPoint(x0,y0);
if valid == true
%     while (x0>0)
   [x0,y0,m]= sensor_t();
   [a,b,c]  = calc_abc(x0,y0,m,n);
   [x,y]    = straight(x0,y0,m,dist);
   plot(x,y,'o'); %Plot Current Position of Car
   [x,y,m]  = sensor_s();
    n       = calc_n(x0,a,b,c);
    m       = turn(m,n,x,x0);
    x0=x;
    y0=y;
%     end
end











