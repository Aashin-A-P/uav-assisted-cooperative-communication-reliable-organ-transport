function p = snr_to_psucc(SNR_dB, gamma_dB, a)
% Logistic mapping: p = 1 / (1 + exp(-a*(SNR-gamma)))
p = 1 ./ (1 + exp(-a .* (SNR_dB - gamma_dB)));
p = min(max(p, 0.01), 0.999); % clamp
end