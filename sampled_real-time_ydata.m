%%%% Infineon raw data sample output matrix
% raw data contiains 64x2x16 matrix, 64 samples per chirp, 2 receivers, and 16 chirps per frame

% Copied and pasted raw data matrix directly from radar (only contains data on one chirp with 64 samples
M = [0.5827 + 0.5607i   0.4386 + 0.5844i;
   0.7658 + 0.8430i   0.1519 + 0.6847i;
   0.5399 + 0.9878i   0.1326 + 0.5431i;
   0.4371 + 0.8010i   0.1915 + 0.4789i;
   0.4107 + 0.7624i   0.2139 + 0.3814i;
   0.2969 + 0.7534i   0.3363 + 0.1731i;
   0.1780 + 0.6413i   0.6440 + 0.1055i;
   0.0945 + 0.4850i   0.8598 + 0.2513i;
   0.1514 + 0.2847i   0.9570 + 0.4479i;
   0.2520 + 0.2239i   0.9404 + 0.5719i;
   0.2913 + 0.1695i   0.9563 + 0.7507i;
   0.3766 + 0.1179i   0.8252 + 0.8796i;
   0.3968 + 0.0190i   0.7797 + 0.9621i;
   0.7321 + 0.0037i   0.5851 + 0.9915i;
   0.9851 + 0.0034i   0.2161 + 0.9919i;
   0.9915 + 0.5897i   0.0110 + 0.9919i;
   0.9919 + 0.9573i   0.0039 + 0.5050i;
   0.5878 + 0.9910i   0.0860 + 0.3245i;
   0.3973 + 0.9397i   0.2308 + 0.1653i;
   0.3365 + 0.8696i   0.4403 + 0.0706i;
   0.2020 + 0.8684i   0.5966 + 0.1006i;
   0.0166 + 0.7477i   0.7656 + 0.1162i;
   0.0037 + 0.4369i   0.8955 + 0.2484i;
   0.0037 + 0.2718i   0.9770 + 0.3817i;
   0.0151 + 0.0957i   0.9912 + 0.5756i;
   0.2549 + 0.0054i   0.9915 + 0.9155i;
   0.5726 + 0.0034i   0.7270 + 0.9905i;
   0.9519 + 0.0034i   0.3941 + 0.9919i;
   0.9907 + 0.2120i   0.2120 + 0.9919i;
   0.9917 + 0.5602i   0.1458 + 0.6589i;
   0.8357 + 0.6813i   0.2928 + 0.5868i;
   0.7509 + 0.7778i   0.2020 + 0.6027i;
   0.6571 + 0.8689i   0.1050 + 0.4247i;
   0.4659 + 0.9692i   0.1917 + 0.1629i;
   0.2940 + 0.9324i   0.4105 + 0.0269i;
   0.0576 + 0.8557i   0.6918 + 0.0042i;
   0.0044 + 0.5658i   0.9565 + 0.1499i;
   0.0034 + 0.3328i   0.9907 + 0.4308i;
   0.0037 + 0.0862i   0.9917 + 0.7092i;
   0.3204 + 0.0054i   0.8071 + 0.9001i;
   0.5714 + 0.0034i   0.5629 + 0.9165i;
   0.5617 + 0.1558i   0.4564 + 0.8361i;
   0.5797 + 0.1079i   0.4088 + 0.7678i;
   0.7338 + 0.0542i   0.3744 + 0.7631i;
   0.9631 + 0.1143i   0.2852 + 0.7438i;
   0.9910 + 0.5079i   0.1851 + 0.6269i;
   0.9917 + 0.8891i   0.1678 + 0.4469i;
   0.6557 + 0.9880i   0.2840 + 0.2725i;
   0.3683 + 0.9309i   0.5087 + 0.1836i;
   0.2452 + 0.7172i   0.7026 + 0.4000i;
   0.3226 + 0.6254i   0.5324 + 0.5619i;
   0.2237 + 0.6845i   0.4269 + 0.4081i;
   0.0801 + 0.5355i   0.5805 + 0.2833i;
   0.0144 + 0.3492i   0.7482 + 0.3336i;
   0.0044 + 0.0425i   0.8759 + 0.5099i;
   0.3348 + 0.0042i   0.8127 + 0.7155i;
   0.6315 + 0.0034i   0.7084 + 0.8244i;
   0.8957 + 0.0034i   0.6337 + 0.9182i;
   0.9900 + 0.0034i   0.4168 + 0.9900i;
   0.9915 + 0.4542i   0.1719 + 0.9915i;
   0.9917 + 0.6847i   0.0547 + 0.6657i;
   0.9919 + 0.7873i   0.1932 + 0.5172i;
   0.9919 + 0.9741i   0.1748 + 0.5451i;
   0.5817 + 0.9915i   0.0642 + 0.3170i;
]

t = transpose(0:(300/64)*10^-6:(300-(300/64))*10^-6); % units in microseconds
plot(t,real(M(:,1)),t,imag(M(:,1))) % plotted I and Q data with respect to the 64 samples/chirp with 300 microsecond chirp time in matrix M
