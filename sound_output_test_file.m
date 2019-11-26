% raw_data = rand(10, 2) + i*(rand(10,2))
% 
% % raw_data = 1 + 1*i;
% 
% angle_degree1 = (angle(raw_data(1)))*(180/pi)
% angle_degree2 = (angle(raw_data(2)))*(180/pi)

syms y Fs
[y,Fs] = audioread('FoolInTheRain.wav')
player = audioplayer(y,Fs);
% sound(y,Fs)
% clear sound
i = 1;
while i == 1
target = 1
if target == 1
    play(player);
elseif target == 0
    pause(player);
end
end
stop(player);



% % while loop
% target = 1; % target is either in or out. When target is in, target = 1. When target is out, target = 0.
% 
% when target == 1

% info = audioinfo('FoolInTheRain.mp3')
% audiowrite('FoolInTheRain.wav',y,Fs)
% clear y Fs

% elseif target == 0 pause music
% while loop end
% Do a continuous run through to process the target as in or out