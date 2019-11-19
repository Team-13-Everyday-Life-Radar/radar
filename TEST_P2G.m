%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Copyright (c) 2018-2019, Infineon Technologies AG
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
% following conditions are met:
%
% Redistributions of source code must retain the above copyright notice, this list of conditions and the following
% disclaimer.
%
% Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
% disclaimer in the documentation and/or other materials provided with the distribution.
%
% Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
% products derived from this software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
% INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
% SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
% SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
% WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
% OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% DESCRIPTION:
%
% This examples shows the range-Doppler processing for the collected Raw
% data and computation of range, speed, and angle of the target.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% NOTES:
%
% For FMCW modulation and one chirp per frame only the range FFT can be
% computed and the Doppler FFT has to be omitted.
% For Doppler modulation there is no FMCWEndpoint in the XML file, the
% range FFT has to be omitted and the Doppler FFT has to be directly computed.
% 
% Tracking has not been used in this code. Range, Doppler and angle
% estimates are obatined for every frame and plotted. To obtain better
% results tracking has to be used.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Startup
clc;
clear;
close all;

%% Constants
global lambda;
global antenna_spacing;

antenna_spacing = 6.22e-3; % in meters
c0 = 3e8; % Speed of light in vacuum

down_chirp_duration = 100e-6; % Time required for down chirp
chirp_to_chirp_delay = 100e-6; % Standby time interval between consequitive chirps

%% Raw Data Name
fdata = 'data_P2G_1person';

%% !!!!!!!! 
%  to parse the XML file, the package XML2STRUCT is requried.
%  Please download the package from
%  https://de.mathworks.com/matlabcentral/fileexchange/28518-xml2struct
%  unzip it and copy the files into this folder
%  the function f_parse_data is not compatible with the build-in matlab
%  function!
%
if not(isfile("xml2struct.m"))
   error("Please install xml2struct.m, please see comments in the source file above!") 
end
%% Load the Raw Data file
[frame, frame_count, calib_data, sXML] = f_parse_data(fdata); % Data Parser

%% Extract FMCW chirp configuration from device data
% Frame duration
frame_time = str2double(sXML.Device.Acquisition.Attributes.acquisitionRate) * 1e-3;

% Pulse repetition time
up_chirp_duration = str2double(sXML.Device.BaseEndpoint.chirpDuration_ns.Text) * 1e-9;
PRT = up_chirp_duration + down_chirp_duration + chirp_to_chirp_delay; % Pulse repetition time : Delay between the start of two chirps

% Bandwidth
BW = (str2double(sXML.Device.FmcwEndpoint.FmcwConfiguration.upperFrequency_kHz.Text) - str2double(sXML.Device.FmcwEndpoint.FmcwConfiguration.lowerFrequency_kHz.Text)) * 1e3;

num_Tx_antennas = str2double(sXML.Device.BaseEndpoint.DeviceInfo.numAntennasTx.Text); % Number of Tx antennas
num_Rx_antennas = length(strfind(dec2bin(str2double(sXML.Device.BaseEndpoint.FrameFormat.rxMask.Text)),'1')); % Number of Rx antennas

% Carier frequency
fC = (str2double(sXML.Device.FmcwEndpoint.FmcwConfiguration.upperFrequency_kHz.Text) + str2double(sXML.Device.FmcwEndpoint.FmcwConfiguration.lowerFrequency_kHz.Text)) / 2 * 1e3;

% Number of ADC smaples per chrip
NTS = str2double(sXML.Device.BaseEndpoint.FrameFormat.numSamplesPerChirp.Text);

% Number of chirps per frame
PN = str2double(sXML.Device.BaseEndpoint.FrameFormat.numChirpsPerFrame.Text);

% Sampling frequency
fS = str2double(sXML.Device.AdcxmcEndpoint.AdcxmcConfiguration.samplerateHz.Text);

%% Algorithm Settings
range_fft_size = 256; % Zero padding by 2
Doppler_fft_size = 32; % Zero padding by 2

range_threshold = 100; % Amplitude threshold to find peaks in range FFT
Doppler_threshold = 50; % Amplitude threshold to find peaks in Doppler FFT

min_distance =  0.9; % Minimum distance of the target from the radar (recommended to be at least 0.9 m)
max_distance = 15.0; % Maximum distance of the target from the radar (recommended to be maximum 25.0 m)

max_num_targets = 5; % Maximum number of targets that can be detected

%% Calculate Derived Parameters
lambda = c0 / fC;

Hz_to_mps_constant = lambda / 2; % Conversion factor from frequency to speed in m/s
IF_scale = 16 * 3.3 * range_fft_size / NTS; % Scaling factor for signal strength

range_window_func = 2 * blackman(NTS); % Window function for range
doppler_window_func = 2 * chebwin(PN); % Window function for Doppler

R_max = NTS * c0 / (2 * BW); % Maximum theoretical range for the system in m
dist_per_bin = R_max / range_fft_size; % Resolution of every range bin in m
array_bin_range = (0:range_fft_size-1) * dist_per_bin; % Vector of Range in m

fD_max = 1 / (2 * PRT); % Maximum theoretical calue of the Doppler
fD_per_bin = fD_max / (Doppler_fft_size/2); % Value of doppler resolution per bin
array_bin_fD = ((1:Doppler_fft_size) - Doppler_fft_size/2 - 1) * -fD_per_bin * Hz_to_mps_constant; % Vector of speed in m/s

%% Initialize Structures & Data
target_measurements.strength = zeros(max_num_targets,frame_count);
target_measurements.range    = zeros(max_num_targets,frame_count);
target_measurements.speed    = zeros(max_num_targets,frame_count);
target_measurements.angle    = zeros(max_num_targets,frame_count);

range_tx1rx1_max = zeros(range_fft_size,1);

range_tx1rx1_complete = zeros(range_fft_size,PN,frame_count);
range_tx1rx2_complete = zeros(range_fft_size,PN,frame_count);

%% ADC Calibration Data
N_cal = length(calib_data) / (2 * num_Rx_antennas);

dec_idx = N_cal / NTS;

calib_i1 = calib_data(1:dec_idx:N_cal);
calib_q1 = calib_data(N_cal+1:dec_idx:2*N_cal);

calib_i2 = calib_data(2*N_cal+1:dec_idx:3*N_cal);
calib_q2 = calib_data(3*N_cal+1:dec_idx:4*N_cal);

calib_rx1 = (calib_i1 + j * calib_q1).';
calib_rx2 = (calib_i2 + j * calib_q2).';

%% Process Frames
for fr_idx = 1:frame_count % Loop over all data frames, while the output window is still open
    
    matrix_raw_data = frame(fr_idx).Chirp; % Raw data for the frame being processed
    
    %% Antenna 1 & 2 Fast Time Processing
    %--------------------------- RX1 ----------------------------
    matrix_tx1rx1 = matrix_raw_data(:,:,1);   % data of first rx. antenna, first tx. ant
    
    matrix_tx1rx1 = (matrix_tx1rx1 - repmat(calib_rx1,1,PN)).*IF_scale;
    
    matrix_tx1rx1 = bsxfun(@minus, matrix_tx1rx1, mean(matrix_tx1rx1)); % Mean removal across range for RX1
    
    range_tx1rx1 = fft(matrix_tx1rx1.*repmat(range_window_func,1,PN),range_fft_size,1); % Windowing across range and range FFT for RX1
    % Please note: Since human target detection at far distances is barely
    % feasable, the computation of the FFT in the firmware is limited  to
    % the first half of the spectrum to save memory (also for RX2).
    
    range_tx1rx1_complete(:,:,fr_idx) = range_tx1rx1; % Save Range FFT for RX1 for every Frame
    
    %--------------------------- RX2 ----------------------------
    matrix_tx1rx2 = matrix_raw_data(:,:,2); %data of second rx. antenna, first tx. ant
    
    matrix_tx1rx2 = (matrix_tx1rx2 - repmat(calib_rx2,1,PN)).*IF_scale;
    
    matrix_tx1rx2 = bsxfun(@minus, matrix_tx1rx2, mean(matrix_tx1rx2)); % Mean removal across Range for Rx2
    
    range_tx1rx2 = fft(matrix_tx1rx2.*repmat(range_window_func,1,PN),range_fft_size,1); % Windowing across range and range FFT
    
    range_tx1rx2_complete(:,:,fr_idx) = range_tx1rx2; % Save range FFT for RX1 for every Frame
    
    %% Range Target Detection
    % Detect the targets in range by applying contant amplitude threshold over range
    
    range_tx1rx1_max = abs(max(range_tx1rx1,[],2)); % Data integration of range FFT over the chrips for target range detection
    
    [tgt_range_idx, tgt_range_mag] = f_search_peak(range_tx1rx1_max, length(range_tx1rx1_max), range_threshold, max_num_targets, min_distance, max_distance, dist_per_bin);
    
    num_of_targets = length(tgt_range_idx);
    
    %% Antenna 1 & 2 Slow Time Processing
    %--------------------------- RX1 ----------------------------
    range_Doppler_tx1rx1 = zeros(range_fft_size, Doppler_fft_size);
    
    rx1_doppler_mean = mean(range_tx1rx1(tgt_range_idx,:),2); % Compute mean across doppler
    
    range_tx1rx1(tgt_range_idx,:) = range_tx1rx1(tgt_range_idx,:) - rx1_doppler_mean(1:num_of_targets); % Mean removal across Doppler
    
    range_Doppler_tx1rx1(tgt_range_idx,:) = fftshift(fft(range_tx1rx1(tgt_range_idx,:).*repmat(doppler_window_func.',num_of_targets,1),Doppler_fft_size,2),2); % Windowing across Doppler and Doppler FFT
    
    Rx_spectrum(:,:,1) = range_Doppler_tx1rx1; % Range Doppler spectrum
    
    %--------------------------- RX2 ----------------------------
    range_Doppler_tx1rx2 = zeros(range_fft_size, Doppler_fft_size);
    
    rx2_doppler_mean = mean(range_tx1rx2(tgt_range_idx,:),2); % Compute mean across Ddoppler
    
    range_tx1rx2(tgt_range_idx,:) = range_tx1rx2(tgt_range_idx,:) - rx2_doppler_mean(1:num_of_targets);% Mean removal across Doppler
    
    range_Doppler_tx1rx2(tgt_range_idx,:) = fftshift(fft(range_tx1rx2(tgt_range_idx,:).*repmat(doppler_window_func.',num_of_targets,1),Doppler_fft_size,2),2);% Windowing across Doppler and Doppler FFT
    
    Rx_spectrum(:,:,2) = range_Doppler_tx1rx2;  % Range Doppler spectrum
    
    %% Extraction of Indices from Range-Doppler Map
    tgt_doppler_idx = zeros(1,num_of_targets);
    
    z1 =  zeros(1,num_of_targets);
    z2 =  zeros(1,num_of_targets);
    
    for j = 1:num_of_targets
        [val, doppler_idx] = max(abs(range_Doppler_tx1rx1(tgt_range_idx(j), :)));
        % Consider the value of the range Doppler map for the two receivers for targets with non
        % zero speed to compute angle of arrival.
        % For zero Doppler (targets with zero speed) calculate mean
        % over Doppler to compute angle of arrival. Index 17 corresponds to zero Doppler
        if (val >= Doppler_threshold && doppler_idx ~= 17)
            tgt_doppler_idx(j) = doppler_idx;
            
            z1(j) = Rx_spectrum(tgt_range_idx(j),tgt_doppler_idx(j),1);
            z2(j) = Rx_spectrum(tgt_range_idx(j),tgt_doppler_idx(j),2);
        else
            tgt_doppler_idx(j) = 17;
            
            z1(j) = rx1_doppler_mean(j);
            z2(j) = rx2_doppler_mean(j);
        end
    end
    
    %%  Measurement Update
    if (num_of_targets > 0)
        for j = 1:num_of_targets
            target_measurements.strength(fr_idx,j) = tgt_range_mag(j);
            target_measurements.range(   fr_idx,j) = (tgt_range_idx(j) - 1) * dist_per_bin;
            target_measurements.speed(   fr_idx,j) = (tgt_doppler_idx(j)- Doppler_fft_size/2 - 1) * -fD_per_bin * Hz_to_mps_constant;
            target_measurements.angle(   fr_idx,j) = f_estimate_angle(z1(j), z2(j));
        end
    end
end

%% Visualization
range_tx1rx1_max_abs = squeeze(abs(max(range_tx1rx1_complete,[],2)));
range_tx1rx2_max_abs = squeeze(abs(max(range_tx1rx2_complete,[],2)));

%%% Plot range FFT amplitude heatmap
% This figure illustrates the distance information of the target(s) over
% all frames within the pre-defined minimum and maximum ranges. The
% brigther the color of a range FFT bin, the higher the range FFT amplitude
% in this bin. The upper and lower subplot shows the information of Rx1 and
% Rx2, respectively. Information on the exemplary data set is given at the
% bottom of this file.
figure;

ax1 = subplot(2,1,1);
imagesc(1:frame_count,array_bin_range,range_tx1rx1_max_abs);
title('Range FFT Amplitude Heatmap for RX1');
xlabel('Frames');
ylabel('Range (m)');
set(gca,'YDir','normal');
ylim([min_distance, max_distance]);

ax2 = subplot(2,1,2);
imagesc(1:frame_count,array_bin_range,range_tx1rx2_max_abs);
title('Range FFT Amplitude Heatmap for RX2');
xlabel('Frames');
ylabel('Range (m)');
set(gca,'YDir','normal');
ylim([min_distance, max_distance]);

linkaxes([ax1,ax2],'xy')

%%% Plot the target detection results (amplitude, range, speed, angle)
% This figure illustrates the target information in four subplots:
%    1) Range FFT amplitude depictes the signal strength of the reflected
%       wave from the target and is dependent on the RCS and the distance
%       of the target. The larger the RCS and the smaller the distance to
%       the antenna, the higher the FFT amplitude.
%       NOTE: A target is only detected if its amplitude is larger than
%       the range_threshold! Otherwise, the FFT amplitude is set to zero.
%    2) Range information of the target. Targets are deteced only within
%       min_distance and max_distance.
%    3) Speed/velocity of the target. Positiv value for an approaching
%       target, negative value for a departing target.
%       NOTE: If the maximum Doppler FFT amplitude is below the
%       Doppler_threshold, the speed is set to zero. This does not
%       influence the target detection, but can be used in tracking
%       algorithms to extinguish static targets.
%    4) Angle of the target. Positive value if the target is on the left
%       side, negative value if the target is on the right side.
figure;
leg = [];

for i = 1:max_num_targets
    leg = [leg; 'Target ', num2str(i)];
    
    subplot(4,1,1);
    hold on;
    plot(target_measurements.strength(:,i));
    
    subplot(4,1,2);
    hold on;
    plot(target_measurements.range(:,i));
    
    subplot(4,1,3);
    hold on;
    plot(target_measurements.speed(:,i));
    
    subplot(4,1,4);
    hold on;
    plot(target_measurements.angle(:,i));
end

ax1 = subplot(4,1,1);
plot([0,frame_count],[range_threshold,range_threshold],'k');
title ('FFT Amplitude');
xlabel('Frames');
ylabel('Amplitude');
leg_range = [leg; 'Range TH'];
legend(leg_range,'Location','EastOutside');

ax2 = subplot(4,1,2);
title ('Range');
xlabel('Frames');
ylabel('Range (m)');
legend(leg,'Location','EastOutside');

ax3 = subplot(4,1,3);
title ('Speed');
xlabel('Frames')
ylabel('Speed (m/s)');
legend(leg,'Location','EastOutside');

ax4 = subplot(4,1,4);
title ('Angle');
xlabel('Frames')
ylabel('Angle (°)');
legend(leg,'Location','EastOutside');

linkaxes([ax1,ax2,ax3,ax4],'x')

%%% Information on the exemplary data set (data_P2G_1person) for all frames:
%    0 -  100: tangentially movement from right to left at 3m
%  100 -  200: tangentially movement from left to right at 5m
%  200 -  400: tangentially movement from right to left at 8m
%  500 -  600: azimuth movement from left to right at 3m for zero degree
%  600 -  700: azimuth movement from right to left at 5m for zero degree
%  700 -  800: azimuth movement from left to right at 8m for zero degree
%  800 - 1400: repeated approaching from the left side to the right side
%              and departing the same way back for different ranges.
% 1400 - 1600: departing and approaching at approximately zero degree
%
% NOTE on the range FFT amplitude:
% Due to positive and negative interferences of multi-path reflections and
% specle, the amplitude is fluctuating for human targets. In worst case,
% the amplitude is below the range_threshold in some frames and the target
% sporadically disappears. This can be prevented by further signal
% processing like tracking.
%
% NOTE on the speed data:
% Since the obervation time is only 8ms per frame,only an instant is shown.
% For tangentially movements, when the target is step-by-step approaching
% and departing, the sign of the velocity is fluctuating. Even for radial
% movements, specle can induce a wrong direction of movement. 
