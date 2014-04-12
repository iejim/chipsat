%% Figure out the filter for the gyro

L = length(d.time);
Fs = 50; %Sampling rate in hertz

% Y = fft(d.y_dot)/L; % Get the FFT and normalize
Y = fft(d.speedmeas)/L; % Get the FFT and normalize
FFT = abs(Y(1:floor(L/2)));

freqs = [0:1/(floor(L/2)-1):1]*Fs/2;
figure(1)
plot(freqs, FFT);
%% From this plot, choose the cutoff frequency of the filter

%Choose
f_co = 1; %1Hz
w_co = 2*pi*f_co;

% The filter has structure
%    1
% -------
%  s + a

filt = tf(w_co,[1 w_co])
figure(2)
bode(filt)

%% Get the discrete version using Tustin's transformation
% This transformations replaces
%      2  z - 1
%  s = - --------
%      T  z + 1

Dfilt = c2d(filt,1/Fs, 'tustin')
figure(3)
bode(Dfilt)
% Has structure
%  az + a
% --------
%  z - b


% This is the filter to be implemented in C.
% The integrator implements a = 1/2*1/Fs, b = 1
