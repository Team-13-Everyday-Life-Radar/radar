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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% DESCRIPTION:
% Parse data file to extract raw data for further processing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Frame, Frame_count, Calib_data, sXML] = f_parse_data(sFile)

fIDRaw = fopen([sFile '.raw'], 'r', 'b'); % open file handle, big-endian style
sXML = xml2struct([sFile '.xml']); % parese xml file to struct

%% read in relevant header information from parsed xml fields
Header.NumChirpsPerFrame = str2double(sXML.Device.BaseEndpoint.FrameFormat.numChirpsPerFrame.Text); % The number of chirps that are processed consecutively in a frame. Must not be 0
Header.NumSamplesPerChirp = str2double(sXML.Device.BaseEndpoint.FrameFormat.numSamplesPerChirp.Text); % The number of samples acquired in each chirp for each enabled RX antenna
Header.NumAntennasRX = str2double(sXML.Device.BaseEndpoint.DeviceInfo.numAntennasRx.Text); % The number of RX signals that have been acquired with each chirp

Calib_data_str = sXML.Device.CalibrationEndpoint.callibrationData.Text; % Calibration data used
Calib_data_str  = strip(Calib_data_str ,'[');
Calib_data_str  = strip(Calib_data_str ,']');
Calib_data_str  = strsplit(Calib_data_str ,', ');
Calib_data = str2double(Calib_data_str);

switch (sXML.Device.BaseEndpoint.FrameFormat.signalPart.Text)
    case{'RADAR_SIGNAL_ONLY_I' 'RADAR_SIGNAL_ONLY_Q'}
        Header.SignalPart = 1; % Only one signal (I or Q) is captured during radar data frame acquisition.
    case('RADAR_SIGNAL_I_AND_Q')
        Header.SignalPart = 2; % Both, I and Q signal are captured as a complex signal during radar data frame acquisition.
end
NumChirpData = Header.NumSamplesPerChirp * Header.NumAntennasRX * Header.SignalPart; % Number of sample values per chirp
NumBlockData = Header.NumSamplesPerChirp * Header.NumChirpsPerFrame * Header.SignalPart; % number of sample values per block for p2g mode
Header.NumData = Header.NumChirpsPerFrame * NumChirpData; % Number of sample values per frame



%% read in each frame
n = 1;
fread(fIDRaw, 1, 'uint8'); % peek into the next frame data block if there is any data available
while(~feof(fIDRaw))
    fseek(fIDRaw, -1, 'cof');
    Frame(n).RawData = fread(fIDRaw, Header.NumData, '*single'); %#ok<*SAGROW> The buffer containing the radar data
       
    Frame(n).FrameNo = fread(fIDRaw, 1, 'uint32'); % The running number of the data frame. The frame counter is, reset every time ep_radar_base_set_automatic_frame_trigger is called. If automatic frame trigger is not active, the frame counter may not work, and this could be 0.
    Frame(n).NumChirps = fread(fIDRaw, 1, 'uint32'); % The number of chirps in this frame.
    Frame(n).NumRXAntenna = fread(fIDRaw, 1, 'uint32'); % The number of RX signals that have been acquired with each chirp.
    Frame(n).Samples = fread(fIDRaw, 1, 'uint32'); % The number of samples acquired in each chirp for each enabled RX antenna.
    Frame(n).RXMask = fread(fIDRaw, 1, 'uint32'); % Each antenna is reperesnted by a bit in this mask. If the bit is set, the according RX antenna was used to capture data in this frame.
    Frame(n).ADCResolution = fread(fIDRaw, 1, 'uint32'); % The ADC resolution of the data in sample_data.
    Frame(n).RXInterleaved = fread(fIDRaw, 1, 'uint32'); % If this is 0, the radar data of multiple RX antennas is stored in consecutive data blocks, where each block holds data of one antenna. If this is non-zero, the radar data of multiple RX antennas is stored in one data block, where for each point in time the samples from all RX antennas are stored consecutively before the data of the next point in time follows.
    Frame(n).DataFormat = fread(fIDRaw, 1, 'uint32'); % This indicates if the data is pDataBuffer is real or complex, and if complex data is interleaved. 0: real, 1: complex, 2: complex interleaved

    % dispatch data
    Chirp = zeros(Frame(n).Samples, Frame(n).NumChirps, Frame(n).NumRXAntenna);
	SamplesPerChirp = Frame(n).Samples;
	
    sn = 0:Frame(n).Samples-1; % zero based sample number
	
	if Frame(n).RXInterleaved % interleaved antenna data
		switch Frame(n).DataFormat
			case 0 % EP_RADAR_BASE_RX_DATA_REAL: The frame data contains only I or Q signal
				% data_value = pFrameStart[SAMPLE_NUMBER * num_rx_antennas + ANTENNA_NUMBER];
				for nc = 0:Frame(n).NumChirps-1
					for na = 0:Frame(n).NumRXAntenna-1
						IData = Frame(n).RawData(1+ sn*Frame(n).NumRXAntenna + na + NumChirpData*nc); % real
						% QData = []; % imag
						Chirp(:,nc+1,na+1) = IData;
					end
				end
				
			case 1 % EP_RADAR_BASE_RX_DATA_COMPLEX: The frame data contains I and Q signals in separate data blocks
				% data_value_real = frame_start[SAMPLE_NUMBER * num_rx_antennas + ANTENNA_NUMBER];
				% data_value_imag = frame_start[(num_samples_per_chirp + SAMPLE_NUMBER) * num_rx_antennas + ANTENNA_NUMBER];
				for nc = 0:Frame(n).NumChirps-1
					for na = 0:Frame(n).NumRXAntenna-1
						IData = Frame(n).RawData(1+ sn *Frame(n).NumRXAntenna + na + NumChirpData*nc); % real
						QData = Frame(n).RawData(1+ (Frame(n).Samples + sn)*Frame(n).NumRXAntenna + na + NumChirpData*nc); % imag
						Chirp(:,nc+1,na+1) = IData + 1i*QData;
					end
				end
				
			case 2 % EP_RADAR_BASE_RX_DATA_COMPLEX_INTERLEAVED: The frame data contains I and Q signals in one interleaved data block
				% data_value_real = frame_start[2 * SAMPLE_NUMBER * num_rx_antennas + ANTENNA_NUMBER];
				% data_value_imag = frame_start[2 * SAMPLE_NUMBER * num_rx_antennas + ANTENNA_NUMBER + 1];
				for nc = 0:Frame(n).NumChirps-1
					for na = 0:Frame(n).NumRXAntenna-1
						IData = Frame(n).RawData(1+ 2*sn*Frame(n).NumRXAntenna + na     + NumChirpData*nc); % real
						QData = Frame(n).RawData(1+ 2*sn*Frame(n).NumRXAntenna + na + 1 + NumChirpData*nc); % imag
						Chirp(:,nc+1,na+1) = IData + 1i*QData;
					end
				end
		end
		
	else % non interleaved antenna data
		switch Frame(n).DataFormat
			case 0 % EP_RADAR_BASE_RX_DATA_REAL: The frame data contains only I or Q signal
				% data_value = frame_start[ANTENNA_NUMBER * num_samples_per_chirp + SAMPLE_NUMBER];
				for nc = 0:Frame(n).NumChirps-1
					for na = 0:Frame(n).NumRXAntenna-1
						IData = Frame(n).RawData(1+ na*Frame(n).Samples + sn + NumChirpData*nc); % real
						% QData = []; % imag
						Chirp(:,nc+1,na+1) = IData;
					end
				end
				
			case 1 % EP_RADAR_BASE_RX_DATA_COMPLEX: The frame data contains I and Q signals in separate data blocks
				% data_value_real = frame_start[(2 * ANTENNA_NUMBER    ) * num_samples_per_chirp + SAMPLE_NUMBER];
				% data_value_imag = frame_start[(2 * ANTENNA_NUMBER + 1) * num_samples_per_chirp + SAMPLE_NUMBER];
				for nc = 0:Frame(n).NumChirps-1
					for na = 0:Frame(n).NumRXAntenna-1
						IData = Frame(n).RawData(1+  2*na   *Frame(n).Samples + sn + NumChirpData*nc); % real
						QData = Frame(n).RawData(1+ (2*na+1)*Frame(n).Samples + sn + NumChirpData*nc); % imag
						Chirp(:,nc+1,na+1) = IData + 1i*QData;
					end
				end
				
			case 2 % EP_RADAR_BASE_RX_DATA_COMPLEX_INTERLEAVED: The frame data contains I and Q signals in one interleaved data block
				% data_value_real = frame_start[2 * ANTENNA_NUMBER * num_samples_per_chirp + 2*SAMPLE_NUMBER];
				% data_value_imag = frame_start[2 * ANTENNA_NUMBER * num_samples_per_chirp + 2*SAMPLE_NUMBER + 1];
				for nc = 0:Frame(n).NumChirps-1
					for na = 0:Frame(n).NumRXAntenna-1
						IData = Frame(n).RawData(1+ 2*na*Frame(n).Samples + 2*sn     + NumChirpData*nc); % real
						QData = Frame(n).RawData(1+ 2*na*Frame(n).Samples + 2*sn + 1 + NumChirpData*nc); % imag
						Chirp(:,nc+1,na+1) = IData + 1i*QData;
					end
				end
		end
	end
	
    Frame(n).Chirp = Chirp;
    fread(fIDRaw, 1, 'uint8'); % peek into the next frame data block if there is any data available
    n = n+1;
end

Frame_count = n - 1;

%% close file
fclose(fIDRaw);
