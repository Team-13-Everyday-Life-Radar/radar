%%%%% Sound output test file with self-generated raw data output.
    % ydata - 10 by 2 matrix of self-generated raw data
    % target - 10 by 1 matrix with target angle calculted by subtracting
    % the angle from the first angle with teh angle from the second column
% Audio data - customize audio file used and find file type
syms y Fs
[y,Fs] = audioread('Fool_In_The_Rain.wav')
player = audioplayer(y,Fs);
info = audioinfo('Fool_In_The_Rain.wav')

% Simulating matrix to act as raw data from radar. Takes information on radar angle.
target = zeros(10,1)
% ydata = zeros(10,2) % cuts off sound to test whether "raw data" detects a target
ydata = rand(10,2) % allows for sound output
for i = 1:1:10
    if mod(i,2) == 1
    ydata(i,1) = rand + 1i*rand; % filling first column of "raw data" with complex #s in order to simulate a target angle
    end
    target(i,1) = angle(ydata(i,1)) - angle(ydata(i,2));
    if target(i) > 0
        play(player); % Refers to chosen audio file at beginning of code
        % insert code for Karplus Strong Algorithm % another option is to
        % use the K-S algorithm rather than an audio file
        pause(10) % wait 10 seconds before moving on to the next line
    else
        stop(player); % Refers to chosen audio file at beginning of code
        pause(10) % wait 10 seconds before moving on to the next line
    end
end
disp(ydata)
disp(target*(180/pi))
% stop(player)


