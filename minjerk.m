function [p,v,a] = minjerk(t)
% compute a minimum jerk trajectory
t2 = t*t;
t3 = t2*t;
t4 = t3*t;
t5 = t4*t;
p = 6*t5 - 15*t4 + 10*t3;
v = 30*t4 - 60*t3 + 30*t2;
a = 120*t3 - 180*t2 + 60*t;