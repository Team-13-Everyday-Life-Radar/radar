%%%% Data to sound test file with self-generated data

% %% Portion of script that plays sound once target is detected in angle range. Uses hypothetical data matrix of target's angle and distance from the radar
% ydata = rand(10,2) + 1i*rand(10,2)
% rx_angle = angle(ydata)*(180/pi)
% target_angle = zeros(10,1);
% for i = 1:1:10
%     target_angle(i,1) = rx_angle(i,1) - rx_angle(i,2)
%     if (target_angle(i) > 0) || (target_angle(i) < 0)
%         play(player); % Refers to chosen audio file at beginning of code
%         % insert code for Karplus Strong Algorithm % another option is to
%         % use the K-S algorithm rather than an audio file
%         %pause(2) % wait 10 seconds before moving on to the next line
%     else
%         stop(player); % Refers to chosen audio file at beginning of code
%         %pause(10) % wait 10 seconds before moving on to the next line
%     end
% end
% stop(player)

%% Data to sound test file - part 2 - plays certain pitches of a guitar string as the angle beamwidth changes. Uses hypothetical data matrix of target's angle and distance from the radar
ydata = rand(10,2) + 1i*rand(10,2)
rx_angle = angle(ydata)*(180/pi)
target_angle = zeros(10,1);
syms fret_value
for i = 1:1:10
    target_angle(i,1) = rx_angle(i,1) - rx_angle(i,2)
    if (target_angle(i,1) > 10) && (target_angle(i,1) < 25)
        fret_value = 15;
    elseif (target_angle(i,1) > -25) && (target_angle(i,1) < -10)
        fret_value = 5;
    elseif (target_angle(i,1) >= -10) && (target_angle(i,1) >= 10)
        fret_value = 10;
    end
    if (target_angle(i,1)>-25) && (target_angle(i,1)<25)
        Fs       = 44100;
        A        = 110; % The A string of a guitar is normally tuned to 110 Hz
        Eoffset  = -5;
        Doffset  = 5;
        Goffset  = 10;
        Boffset  = 14;
        E2offset = 19;
        F = linspace(1/Fs, 1000, 2^12);
        x = zeros(Fs*4, 1);
        delay = round(Fs/A);
        b  = firls(42, [0 1/delay 2/delay 1], [0 0 1 1]);
        a  = [1 zeros(1, delay) -0.5 -0.5];
        [H,W] = freqz(b, a, F, Fs);
        plot(W, 20*log10(abs(H)));
        title('Harmonics of an open A string');
        xlabel('Frequency (Hz)');
        ylabel('Magnitude (dB)');
        zi = rand(max(length(b),length(a))-1,1);
        note = filter(b, a, x, zi);
        note = note-mean(note);
        note = note/max(abs(note));
        % hplayer = audioplayer(note, Fs); play(hplayer)
        
        fret  = 4;
        delay = round(Fs/(A*2^(fret/fret_value)));
        
        b  = firls(42, [0 1/delay 2/delay 1], [0 0 1 1]);
        a  = [1 zeros(1, delay) -0.5 -0.5];
        
        [H,W] = freqz(b, a, F, Fs);
        hold on
%         plot(W, 20*log10(abs(H)));
%         title('Harmonics of the A string');
%         legend(['Open A string'], ['A string on the ',num2str(fret_value),' fret']);
        zi = rand(max(length(b),length(a))-1,1);
        note = filter(b, a, x, zi);
        note = note-mean(note);
        note = note/max(note);
        hplayer = audioplayer(note, Fs); play(hplayer)
        pause(2)
    end
end





