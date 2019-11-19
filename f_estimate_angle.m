function target_angle_deg = f_estimate_angle(rx1, rx2)
    % Estimation of AOA for target based on monopulse method
    global antenna_spacing;
    global lambda;
    
    ang_Rx1 = angle(rx1); % Phase of received signal for Rx1
    ang_Rx2 = angle(rx2); % Phase of received signal for Rx2
    
    d_phi = ang_Rx1 - ang_Rx2; % Phase difference between the received signal at the two receivers
    
    if (d_phi <= 0)
        d_phi = d_phi + 2*pi;
    end
    d_phi = d_phi - pi;
    
    target_angle = asin(d_phi * lambda / antenna_spacing / (2*pi)); % AOA in radians
    
    target_angle_deg = (target_angle * 180 / pi); % AOA in degrees
    
end

