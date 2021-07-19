% constants

T = 0.01;

% set up desired trajectory

i = 1;

for t = 0:T:0.1
posd(i) = 0;
veld(i) = 0;
accd(i) = 0;
i = i + 1;
end

for t = 0:T:1.0
[p,v,a] = minjerk(t);
posd(i) = p;
veld(i) = v;
accd(i) = a;
i = i + 1;
end

for t = 0:T:0.4
[p,v,a] = minjerk(t);
posd(i) = 1;
veld(i) = 0;
accd(i) = 0;
i = i + 1;
end

% Show how differentiation blows up noise

mpos = posd + 0.001*randn(size(posd));
figure
plot(mpos)                          
title('position with added noise')
xlabel('sample (100Hz)')

figure
plot(diff(mpos)/T)                  
hold
plot(veld,'r')
hold
legend('differentiated position','true velocity')
xlabel('sample (100Hz)')

% Create idealized system to match filter assumptions
% state is (position, velocity)'
% We are assuming Q, R, and P0 to be diagonal

A = [ 1 T
      0 1 ];

B = [ 0
      T ];

C = [ 1 0 ];

% process variance
Q = [ 1e-6 0
      0 1e-5 ];

% sensor noise variance
R = [ 1e-5 ];

% initial state estimate variance
P0 = [ 1e-4 0
       0 1e-4 ];

% Create some data
state = [ sqrt(P0(1,1))*randn(1)
          sqrt(P0(2,2))*randn(1) ];
for i = 1:length(posd)
 postrue(i) = state(1);
 veltrue(i) = state(2);
 % simulate noisy measurement
 measurement(i) = C*state + sqrt(R(1,1))*randn(1);
 torque(i) = accd(i);
 process_noise = [ sqrt(Q(1,1))*randn(1)
                   sqrt(Q(2,2))*randn(1) ];
 state = A*state + B*accd(i) + process_noise;
end

% Design filter
% Note that we can design filter in advance of seeing the data.
Pm = P0;
for i = 1:1000
 % measurement step
 S = C*Pm*C' + R;
 K = Pm*C'*inv(S);
 Pp = Pm - K*C*Pm;
 % prediction step
 Pm = A*Pp*A' + Q;
end

% Run the filter to create example output
sem = [ 0 0 ]';
for i = 1:length(posd)
 % measurement step
 sep = sem + K*(measurement(i)-C*sem);
 pose(i) = sep(1);
 vele(i) = sep(2);	   
 % prediction step
 sem = A*sep + B*torque(i);
end

% Let's plot the Kalman filter output
ii = 1:length(pose);
figure
plot(ii,pose,'b',ii,postrue,'r')
hold
plot(ii,measurement,'b',ii,postrue,'r')
hold
plot(ii,vele,'b',ii,veltrue,'r')
hold
legend('KF velocity','true velocity')
xlabel('sample (100Hz)')

% Let's compare to directly filtering the output.
vel1 = diff( measurement )/T;
% get length right
vel1(length(veltrue)) = 0;
plot(ii,vel1,'b',ii,vele,'r')

% How well can we do with a first order filter?
[B,A]=butter(1,0.1);
vel2 = filter(B,A,vel1);
%plot(ii,vel2,'b',ii,veltrue,'m',ii,vele,'r')
figure
plot(ii,vel2,'b',ii,veltrue,'r')
legend('1st order filtered velocity','true velocity')
xlabel('sample (100Hz)')


% How well can we do with a second order filter?
[B,A]=butter(2,0.1);
vel2 = filter(B,A,vel1);
% plot(ii,vel2,'b',ii,veltrue,'m',ii,vele,'r')
figure
plot(ii,vel2,'b',ii,veltrue,'r')
legend('2nd order filtered velocity','true velocity')
xlabel('sample (100Hz)')

% How well can we do with a third order filter?
[B,A]=butter(3,0.1);
vel2 = filter(B,A,vel1);
% plot(ii,vel2,'b',ii,veltrue,'m',ii,vele,'r')
figure
plot(ii,vel2,'b',ii,veltrue,'r')
legend('3rd order filtered velocity','true velocity')
xlabel('sample (100Hz)')
