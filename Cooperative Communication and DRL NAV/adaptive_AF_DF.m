function mode = adaptive_AF_DF(priority, conf, relay)

if priority == "HIGH"
    if conf > 0.8
        mode = "DF";
    else
        mode = "AF";
    end

elseif priority == "MEDIUM"
    if relay.SNR_dB > 10
        mode = "DF";
    else
        mode = "AF";
    end

else
    if relay.Delay_s < 1
        mode = "AF";
    else
        mode = "DF";
    end
end

end