function out = best_access_relay_by_snr(U, prevRelayIdx)

% Choose relay with strongest SNR
[bestSNR, idx] = max(U.SNR_dB);
bestRelayIdx = idx(1);

% Optional hysteresis to prevent ping-pong switching
hysteresis_dB = 0.5;

if ~isempty(prevRelayIdx)
    if U.SNR_dB(prevRelayIdx) >= bestSNR - hysteresis_dB
        bestRelayIdx = prevRelayIdx;
        bestSNR = U.SNR_dB(prevRelayIdx);
    end
end

out.bestRelayIdx = bestRelayIdx;
out.bestSNR_dB   = bestSNR;

end