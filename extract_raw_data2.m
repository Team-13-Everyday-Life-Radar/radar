%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function out = extract_raw_data (in)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Copyright (c) 2014-2019, Infineon Technologies AG
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
% This simple example demos the acquisition of data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% cleanup and init
% Before starting any kind of device the workspace must be cleared and the
% MATLAB Interface must be included into the code. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
disp('******************************************************************');
addpath('C:\IFX_P2G-HW-SW_V1.0.2\Firmware_Software\Communication Library\ComLib_Matlab_Interface\RadarSystemImplementation'); % add Matlab API
clear all; %#ok<CLSCR>
close all
resetRS; % close and delete ports

% 1. Create radar system object
szPort = findRSPort;  %scan all available ports
oRS = RadarSystem(szPort); % setup object and connect to board

disp('Connected RadarSystem:');
oRS %#ok<*NOPTS>

% 2. Enable automatic trigger with frame time 1s
oRS.oEPRadarBase.set_automatic_frame_trigger(1000000);

while true
    % 3. Trigger radar chirp and get the raw data
    [mxRawData, sInfo] = oRS.oEPRadarBase.get_frame_data;
    ydata = mxRawData; % get raw data
    
     disp((ydata));
     
     figure(1)
     subplot(1,2,1);
     plot([real(ydata(:,1)), imag(ydata(:,1))]);
     axis ([0 100 -0.1 1.2])
     title('Reciever 1 Output Data')
     grid on
     subplot(1,2,2)
     plot([real(ydata(:,2)), imag(ydata(:,2))]);
     axis ([0 100 -0.1 1.2])
     title('Reciever 1 Output Data')
     grid on
     
     
      figure(2)
      subplot(1,2,1);
      plot((abs(fft(ydata(:,1))).^2),'b')
      axis ([0 100 0 3])
      grid on
      subplot(1,2,2);
      plot((abs(fft(ydata(:,2))).^2),'r')
      axis ([0 100 0 3])
      grid on

end

% 
% subplot(1,2,1);plot([real(mxRawData(:,1)), imag(mxRawData(:,1))]); % plot data
%     subplot(1,2,2);plot([real(mxRawData(:,2)), imag(mxRawData(:,2))]); % plot data
%     
%     
% 
% xmlwrite('infoUAT.xml',docNode);
% type('infoUAT.xml');
% 
% 
% save RawData.xml ydata
% 
% docNode = com.mathworks.xml.XMLUtils.createDocument('toc');
% toc = docNode.getDocumentElement;
% toc.setAttribute('version','1.0');
% product = docNode.createElement('tocitem');
% product.setAttribute('target','upslope_product_page.html');
% product.appendChild(docNode.createTextNode('Upslope Area Toolbox'));
% toc.appendChild(product);
% functions = {'demFlow','facetFlow','flowMatrix','pixelFlow'};
% for idx = 1:numel(functions)
%     curr_node = docNode.createElement('tocitem');
%     
%     curr_file = [functions{idx} '_help.html']; 
%     curr_node.setAttribute('target',curr_file);
%     
%     % Child text is the function name.
%     curr_node.appendChild(docNode.createTextNode(functions{idx}));
%     product.appendChild(curr_node);
% end
% xmlwrite('infoUAT.xml',docNode);
% type('infoUAT.xml');
% 
