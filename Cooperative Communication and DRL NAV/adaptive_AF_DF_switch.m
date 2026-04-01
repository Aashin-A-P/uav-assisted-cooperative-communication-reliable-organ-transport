function mode = adaptive_AF_DF_switch(SNR_dB, delay_s, priority)

% Thresholds (you can tune)
SNR_low  = 8;
SNR_high = 18;
delay_th = 1.0;

if priority == 3
    % HIGH PRIORITY (organ / emergency)
    if SNR_dB >= SNR_low && delay_s <= delay_th
        mode = "AF";   % low delay
    else
        mode = "DF";   % reliability
    end

elseif priority == 2
    % MEDIUM PRIORITY
    if SNR_dB >= SNR_high
        mode = "AF";
    else
        mode = "DF";
    end

else
    % LOW PRIORITY
    if SNR_dB >= SNR_high
        mode = "AF";
    else
        mode = "DF";
    end
end

end