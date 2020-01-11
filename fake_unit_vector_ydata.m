%%%% Infineon raw data (ydata) output matrix - simulated dummy data
% raw data contains 64x2x16 matrix, 64 samples per chirp, 2 receivers, and 16 chirps per frame
% with a 300 microsecond chirp time

% A = rand(64,2,16) + 1i*rand(64,2,16)
% z = x + j*y;
% v = x.*cos(2*pi*Fc*t) - y.*sin(2*pi*Fc*t);
% I = real(z);
% Q = imag(z);

t = transpose(0:(300/64)*10^-6:(300-(300/64))*10^-6); % units in microseconds
samples = transpose(0:1:63)
f = 15000;
Rx1_Idata = 0.5*cos(2*pi*f*t) + 0.5
Rx1_Qdata = 0.5*sin(2*pi*f*t) + 0.5
ydata_Rx1 = Rx1_Idata + 1i*Rx1_Qdata % simulated receiver 1 ydata matrix
plot(t,Rx1_Idummydata,t,Rx1_Qdummydata)
clear xlabel
xlabel('Time (seconds)')
ylabel('Unit Magnitude')
title('I and Q data points from simulated ydata')

legend({'I Data','Q Data'},'Location','southeast')




